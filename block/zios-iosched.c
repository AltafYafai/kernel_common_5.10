// SPDX-License-Identifier: GPL-2.0
/*
 * ZIOS I/O Scheduler — Zenith I/O Scheduler
 *
 * A game-aware, top-app-cgroup-aware multi-queue I/O scheduler for
 * mobile devices (UFS / NVMe).  Designed for the Zenith kernel.
 *
 * Core ideas:
 *   - O(1) insert and dispatch (no rbtrees, no sorting, no RCU)
 *   - Game mode detection from the zenith CPU governor
 *   - Top-app cgroup awareness (foreground app I/O priority)
 *   - Read-first dispatch that becomes aggressive during gaming
 *   - Write absorption / batching during game mode
 *   - Sector-aware dispatch for better sequential throughput
 *
 * Copyright (C) 2026
 */
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/blkdev.h>
#include <linux/blk-mq.h>
#include <linux/elevator.h>
#include <linux/bio.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/jiffies.h>
#include <linux/sched.h>
#include <linux/cgroup.h>
#include <linux/cpufreq_zenith.h>
#include <linux/list_sort.h>

#include "blk.h"
#include "blk-mq.h"
#include "blk-mq-sched.h"
#include "blk-mq-debugfs.h"

#define ZIOS_VERSION "1.1.0"

/* Per-request scheduler data */
struct zios_rq_data {
	struct request *rq;
	struct list_head top_app_node;	/* link in top_app_list if top-app */
};

/*
 * Per-queue scheduler data.
 */
struct zios_data {
	/* I/O lists */
	struct list_head	sync_read_list;
	struct list_head	async_read_list;
	struct list_head	write_list;
	struct list_head	top_app_list;

	/* Counters */
	unsigned int		nr_sync_reads;
	unsigned int		nr_async_reads;
	unsigned int		nr_writes;
	unsigned int		nr_top_app;

	/* Stats (for diagnostics) */
	atomic64_t		dispatched_reads;
	atomic64_t		dispatched_writes;
	atomic64_t		merged_requests;

	/* Last dispatched sector (elevator hint) */
	sector_t		last_sector;
	bool			last_sector_valid;

	/* Dispatch accounting */
	unsigned int		dispensed;
	unsigned int		write_starve_count; /* nr of read dispatches while writes pending */

	/* Game mode — cached from governor, rechecked every 100ms */
	bool			game_mode;
	unsigned long		last_game_check;

	/* Parameter sets — game vs normal */
	struct {
		unsigned int	read_batch_max;
		unsigned int	write_batch_max;
		unsigned int	top_app_bias_pct;
		unsigned int	write_starve_max;
	} game_params, normal_params;

	/* Sector coalescing — scan up to N entries for near-sector match */
	unsigned int		read_coalesce_limit;

	/* Write pressure cap — max pending writes before forced flush */
	unsigned int		max_write_pending;

	/* Write list sort flag — set when batch needs sorting */
	bool			writes_dirty;

	/* Async queue depth */
	unsigned int		async_depth;

	/* Top-app cgroup name */
	char			top_app_cgroup_name[256];

	/* Per-request data slab */
	struct kmem_cache	*rq_data_cache;

	struct request_queue	*queue;
	spinlock_t		lock;
};

static int zios_to_word_depth(struct blk_mq_hw_ctx *hctx, unsigned int qdepth)
{
	struct sbitmap_queue *bt = hctx->sched_tags->bitmap_tags;
	const unsigned int nrr = hctx->queue->nr_requests;

	return ((qdepth << bt->sb.shift) + nrr - 1) / nrr;
}

static inline struct zios_rq_data *get_rq_data(struct request *rq)
{
	return rq->elv.priv[0];
}

/* ---------- cgroup helpers ---------- */

static inline bool task_in_cgroup_named(struct task_struct *task,
					const char *cgroup_name)
{
	struct cgroup *cgrp;
	bool ret = false;

	rcu_read_lock();
	cgrp = task_dfl_cgroup(task);

	while (cgrp) {
		if (cgrp->kn && !strcmp(cgrp->kn->name, cgroup_name)) {
			ret = true;
			goto out;
		}
		cgrp = cgroup_parent(cgrp);
	}
out:
	rcu_read_unlock();
	return ret;
}

/* ---------- LBA sort for write batching ---------- */

static int zios_cmp_write_lba(void *priv, struct list_head *a,
			      struct list_head *b)
{
	struct request *rq_a = list_entry(a, struct request, queuelist);
	struct request *rq_b = list_entry(b, struct request, queuelist);

	return (int)(blk_rq_pos(rq_a) > blk_rq_pos(rq_b)) -
	       (int)(blk_rq_pos(rq_a) < blk_rq_pos(rq_b));
}

/*
 * Scan a list for the request with sector nearest to @target.
 * Scans at most @limit entries. Returns the closest match or
 * the first entry if limit is 1.
 */
static struct request *zios_find_nearest(struct list_head *list,
					 sector_t target,
					 unsigned int limit)
{
	struct request *rq, *best = NULL;
	sector_t best_delta = SECTOR_MAX;
	unsigned int scanned = 0;

	list_for_each_entry(rq, list, queuelist) {
		sector_t delta;

		if (target > blk_rq_pos(rq))
			delta = target - blk_rq_pos(rq);
		else
			delta = blk_rq_pos(rq) - target;

		if (delta < best_delta) {
			best_delta = delta;
			best = rq;
		}

		if (++scanned >= limit)
			break;
	}

	return best;
}

/* ---------- request lifecycle ---------- */

static void zios_prepare_request(struct request *rq)
{
	struct zios_data *zd = rq->q->elevator->elevator_data;
	struct zios_rq_data *rd;

	rd = kmem_cache_alloc(zd->rq_data_cache, GFP_ATOMIC);
	if (!rd)
		return;

	memset(rd, 0, sizeof(*rd));
	rd->rq = rq;
	INIT_LIST_HEAD(&rd->top_app_node);
	rq->elv.priv[0] = rd;
}

static void zios_finish_request(struct request *rq)
{
	struct zios_data *zd = rq->q->elevator->elevator_data;
	struct zios_rq_data *rd = get_rq_data(rq);

	if (rd) {
		rq->elv.priv[0] = NULL;
		kmem_cache_free(zd->rq_data_cache, rd);
	}
}

/* ---------- insert / classify ---------- */

static void zios_insert_request(struct blk_mq_hw_ctx *hctx,
				struct request *rq, bool at_head)
{
	struct request_queue *q = hctx->queue;
	struct zios_data *zd = q->elevator->elevator_data;
	struct zios_rq_data *rd = get_rq_data(rq);
	bool is_sync, is_write;
	bool is_top_app;

	lockdep_assert_held(&zd->lock);

	blk_req_zone_write_unlock(rq);

	if (!rd)
		return;

	/* Try merge first */
	if (blk_mq_sched_try_insert_merge(q, rq))
		return;

	blk_mq_sched_request_inserted(rq);

	is_sync = rq->cmd_flags & REQ_SYNC;
	is_write = op_is_write(req_op(rq));

	is_top_app = task_in_cgroup_named(current, zd->top_app_cgroup_name);

	if (at_head) {
		list_add(&rq->queuelist, &zd->sync_read_list);
		zd->nr_sync_reads++;
	} else if (is_top_app) {
		list_add_tail(&rd->top_app_node, &zd->top_app_list);
		list_add_tail(&rq->queuelist, &zd->top_app_list);
		zd->nr_top_app++;
	} else if (is_sync && !is_write) {
		list_add_tail(&rq->queuelist, &zd->sync_read_list);
		zd->nr_sync_reads++;
	} else if (!is_write) {
		list_add_tail(&rq->queuelist, &zd->async_read_list);
		zd->nr_async_reads++;
	} else {
		list_add_tail(&rq->queuelist, &zd->write_list);
		zd->nr_writes++;
		zd->writes_dirty = true;
	}
}

static void zios_insert_requests(struct blk_mq_hw_ctx *hctx,
				 struct list_head *list, bool at_head)
{
	struct zios_data *zd = hctx->queue->elevator->elevator_data;

	spin_lock(&zd->lock);
	while (!list_empty(list)) {
		struct request *rq;

		rq = list_first_entry(list, struct request, queuelist);
		list_del_init(&rq->queuelist);
		zios_insert_request(hctx, rq, at_head);
	}
	spin_unlock(&zd->lock);
}

/* ---------- dispatch ---------- */

static struct request *zios_dispatch_request(struct blk_mq_hw_ctx *hctx)
{
	struct zios_data *zd = hctx->queue->elevator->elevator_data;
	struct request *rq = NULL;
	bool game_mode;
	unsigned int rbatch, wbatch, bias_pct, starve_max;
	unsigned int coalesce;

	spin_lock(&zd->lock);

	/* Refresh game mode every 100 ms */
	if (time_after(jiffies, zd->last_game_check + HZ / 10)) {
		zd->game_mode = zenith_is_game_mode_active();
		zd->last_game_check = jiffies;
	}
	game_mode = zd->game_mode;

	if (game_mode) {
		rbatch    = zd->game_params.read_batch_max;
		wbatch    = zd->game_params.write_batch_max;
		bias_pct  = zd->game_params.top_app_bias_pct;
		starve_max = zd->game_params.write_starve_max;
	} else {
		rbatch    = zd->normal_params.read_batch_max;
		wbatch    = zd->normal_params.write_batch_max;
		bias_pct  = zd->normal_params.top_app_bias_pct;
		starve_max = zd->normal_params.write_starve_max;
	}
	coalesce = zd->read_coalesce_limit;

	/* ----- Tier 0: Top-app priority ----- */
	if (zd->nr_top_app > 0) {
		unsigned int reserve = (zd->dispensed * bias_pct) / 100;

		if (zd->nr_top_app > reserve) {
			struct zios_rq_data *rd;

			rd = list_first_entry(&zd->top_app_list,
					      struct zios_rq_data, top_app_node);
			rq = rd->rq;
			list_del_init(&rd->top_app_node);
			list_del_init(&rq->queuelist);
			zd->nr_top_app--;

			if (zd->nr_writes > 0)
				zd->write_starve_count++;

			goto done;
		}
	}

	/* ----- Tier 1: Sync reads (sector-aware) ----- */
	if (zd->nr_sync_reads > 0) {
		if (game_mode || zd->write_starve_count < starve_max) {
			if (coalesce > 1 && zd->last_sector_valid)
				rq = zios_find_nearest(&zd->sync_read_list,
						       zd->last_sector, coalesce);
			else
				rq = list_first_entry_or_null(&zd->sync_read_list,
						    struct request, queuelist);

			if (rq) {
				list_del_init(&rq->queuelist);
				zd->nr_sync_reads--;
				if (zd->nr_writes > 0)
					zd->write_starve_count++;
				goto done;
			}
		}
	}

	/* ----- Tier 2: Async reads (sector-aware) ----- */
	if (zd->nr_async_reads > 0) {
		if (game_mode || zd->write_starve_count < starve_max) {
			if (coalesce > 1 && zd->last_sector_valid)
				rq = zios_find_nearest(&zd->async_read_list,
						       zd->last_sector, coalesce);
			else
				rq = list_first_entry_or_null(&zd->async_read_list,
						    struct request, queuelist);

			if (rq) {
				list_del_init(&rq->queuelist);
				zd->nr_async_reads--;
				if (zd->nr_writes > 0)
					zd->write_starve_count++;
				goto done;
			}
		}
	}

	/* ----- Tier 3: Writes (LBA-sorted flush) ----- */
	if (zd->nr_writes > 0) {
		if (zd->write_starve_count >= starve_max ||
		    zd->nr_writes >= wbatch ||
		    zd->nr_writes >= zd->max_write_pending ||
		    (!zd->nr_sync_reads && !zd->nr_async_reads && !zd->nr_top_app)) {

			/* Sort writes by LBA for sequential throughput */
			if (zd->writes_dirty && zd->nr_writes > 1) {
				list_sort(NULL, &zd->write_list,
					  zios_cmp_write_lba);
				zd->writes_dirty = false;
			}

			rq = list_first_entry_or_null(&zd->write_list,
						      struct request, queuelist);
			if (rq) {
				list_del_init(&rq->queuelist);
				zd->nr_writes--;
				zd->write_starve_count = 0;
				goto done;
			}
		}
	}

done:
	if (rq) {
		zd->dispensed++;
		if (zd->dispensed > 256)
			zd->dispensed = 0;

		/* Track last sector for elevator hint */
		zd->last_sector = blk_rq_pos(rq) + blk_rq_sectors(rq);
		zd->last_sector_valid = true;

		/* Update stats */
		if (op_is_write(req_op(rq)))
			atomic64_inc(&zd->dispatched_writes);
		else
			atomic64_inc(&zd->dispatched_reads);

		blk_req_zone_write_lock(rq);
		rq->rq_flags |= RQF_STARTED;
		spin_unlock(&zd->lock);
		return rq;
	}

	spin_unlock(&zd->lock);
	return NULL;
}

/* ---------- has work / depth limit ---------- */

static bool zios_has_work(struct blk_mq_hw_ctx *hctx)
{
	struct zios_data *zd = hctx->queue->elevator->elevator_data;

	return !list_empty_careful(&zd->sync_read_list) ||
	       !list_empty_careful(&zd->async_read_list) ||
	       !list_empty_careful(&zd->write_list) ||
	       !list_empty_careful(&zd->top_app_list);
}

static void zios_limit_depth(unsigned int op, struct blk_mq_alloc_data *data)
{
	struct zios_data *zd = data->q->elevator->elevator_data;

	if (op_is_sync(op) && !op_is_write(op))
		return;

	data->shallow_depth = zios_to_word_depth(data->hctx, zd->async_depth);
}

static void zios_depth_updated(struct blk_mq_hw_ctx *hctx)
{
	struct request_queue *q = hctx->queue;
	struct zios_data *zd = q->elevator->elevator_data;
	struct blk_mq_tags *tags = hctx->sched_tags;

	zd->async_depth = q->nr_requests;
	sbitmap_queue_min_shallow_depth(tags->bitmap_tags, 1);
}

static int zios_init_hctx(struct blk_mq_hw_ctx *hctx, unsigned int hctx_idx)
{
	zios_depth_updated(hctx);
	return 0;
}

/* ---------- merge & completion callbacks ---------- */

static bool zios_bio_merge(struct request_queue *q, struct bio *bio,
			   unsigned int nr_segs)
{
	struct zios_data *zd = q->elevator->elevator_data;
	struct request *free = NULL;
	bool ret;

	spin_lock(&zd->lock);
	ret = blk_mq_sched_try_merge(q, bio, nr_segs, &free);
	spin_unlock(&zd->lock);

	if (free)
		blk_mq_free_request(free);

	return ret;
}

static void zios_request_merged(struct request_queue *q, struct request *req,
				enum elv_merge type)
{
}

static void zios_merged_requests(struct request_queue *q, struct request *req,
				 struct request *next)
{
	struct zios_data *zd = q->elevator->elevator_data;

	lockdep_assert_held(&zd->lock);
	atomic64_inc(&zd->merged_requests);
	zios_finish_request(next);
}

static void zios_completed_request(struct request *rq, u64 now)
{
}

/* ---------- init / exit ---------- */

static void zios_exit_sched(struct elevator_queue *e)
{
	struct zios_data *zd = e->elevator_data;

	WARN_ON_ONCE(!list_empty(&zd->sync_read_list));
	WARN_ON_ONCE(!list_empty(&zd->async_read_list));
	WARN_ON_ONCE(!list_empty(&zd->write_list));
	WARN_ON_ONCE(!list_empty(&zd->top_app_list));

	kmem_cache_destroy(zd->rq_data_cache);
	kfree(zd);
}

static int zios_init_sched(struct request_queue *q, struct elevator_type *e)
{
	struct zios_data *zd;
	struct elevator_queue *eq;

	eq = elevator_alloc(q, e);
	if (!eq)
		return -ENOMEM;

	zd = kzalloc_node(sizeof(*zd), GFP_KERNEL, q->node);
	if (!zd) {
		kobject_put(&eq->kobj);
		return -ENOMEM;
	}

	zd->rq_data_cache = kmem_cache_create("zios_rq_data",
					      sizeof(struct zios_rq_data),
					      0, SLAB_HWCACHE_ALIGN, NULL);
	if (!zd->rq_data_cache) {
		kfree(zd);
		kobject_put(&eq->kobj);
		return -ENOMEM;
	}

	INIT_LIST_HEAD(&zd->sync_read_list);
	INIT_LIST_HEAD(&zd->async_read_list);
	INIT_LIST_HEAD(&zd->write_list);
	INIT_LIST_HEAD(&zd->top_app_list);

	/* Game mode defaults — read-firehose */
	zd->game_params.read_batch_max   = 64;
	zd->game_params.write_batch_max  = 4;
	zd->game_params.top_app_bias_pct = 80;
	zd->game_params.write_starve_max = 10;

	/* Normal mode defaults — balanced */
	zd->normal_params.read_batch_max   = 16;
	zd->normal_params.write_batch_max  = 16;
	zd->normal_params.top_app_bias_pct = 50;
	zd->normal_params.write_starve_max = 3;

	/* Sector coalescing — scan 8 entries for near-sector match */
	zd->read_coalesce_limit = 8;

	/* Write pressure cap — flush writes when 64 queued */
	zd->max_write_pending = 64;

	zd->async_depth = q->nr_requests;

	strscpy(zd->top_app_cgroup_name, "top-app",
		sizeof(zd->top_app_cgroup_name));

	zd->last_game_check = jiffies;
	zd->game_mode = false;
	zd->last_sector_valid = false;
	zd->writes_dirty = false;

	spin_lock_init(&zd->lock);
	zd->queue = q;
	eq->elevator_data = zd;
	q->elevator = eq;

	return 0;
}

/* ---------- sysfs attributes ---------- */

#define ZIOS_ATTR_RW_SHOW(name, field)					\
static ssize_t zios_##name##_show(struct elevator_queue *e, char *page)	\
{									\
	struct zios_data *zd = e->elevator_data;			\
	return sysfs_emit(page, "%u\n", zd->field);			\
}

#define ZIOS_ATTR_RW_STORE(name, field, min, max)			\
static ssize_t zios_##name##_store(struct elevator_queue *e,		\
				   const char *page, size_t count)	\
{									\
	struct zios_data *zd = e->elevator_data;			\
	unsigned int val;						\
	int ret;							\
	ret = kstrtouint(page, 10, &val);				\
	if (ret || val < (min) || val > (max))				\
		return -EINVAL;						\
	zd->field = val;						\
	return count;							\
}

#define ZIOS_RO(name, field) \
	ZIOS_ATTR_RW_SHOW(name, field)

#define ZIOS_RW(name, field, min, max)		\
	ZIOS_ATTR_RW_SHOW(name, field)		\
	ZIOS_ATTR_RW_STORE(name, field, min, max)

ZIOS_RW(game_read_batch_max,     game_params.read_batch_max,     1, 256)
ZIOS_RW(game_write_batch_max,    game_params.write_batch_max,    1, 256)
ZIOS_RW(game_top_app_bias_pct,   game_params.top_app_bias_pct,   0, 100)
ZIOS_RW(game_write_starve_max,   game_params.write_starve_max,   1, 100)

ZIOS_RW(normal_read_batch_max,   normal_params.read_batch_max,   1, 256)
ZIOS_RW(normal_write_batch_max,  normal_params.write_batch_max,  1, 256)
ZIOS_RW(normal_top_app_bias_pct, normal_params.top_app_bias_pct, 0, 100)
ZIOS_RW(normal_write_starve_max, normal_params.write_starve_max, 1, 100)

ZIOS_RO(game_mode, game_mode)
ZIOS_RO(nr_sync_reads,  nr_sync_reads)
ZIOS_RO(nr_async_reads, nr_async_reads)
ZIOS_RO(nr_writes, nr_writes)
ZIOS_RO(nr_top_app, nr_top_app)

ZIOS_RW(read_coalesce_limit, read_coalesce_limit, 1, 64)
ZIOS_RW(max_write_pending, max_write_pending, 1, 4096)
ZIOS_RW(async_depth, async_depth, 1, INT_MAX)

/* Version (need a real function since version is a macro constant) */
static ssize_t zios_version_show(struct elevator_queue *e, char *page)
{
	return sysfs_emit(page, "%s\n", ZIOS_VERSION);
}

/* Stats (read-only, atomic64) */
static ssize_t zios_dispatched_reads_show(struct elevator_queue *e, char *page)
{
	struct zios_data *zd = e->elevator_data;
	return sysfs_emit(page, "%llu\n", atomic64_read(&zd->dispatched_reads));
}

static ssize_t zios_dispatched_writes_show(struct elevator_queue *e, char *page)
{
	struct zios_data *zd = e->elevator_data;
	return sysfs_emit(page, "%llu\n", atomic64_read(&zd->dispatched_writes));
}

static ssize_t zios_merged_requests_show(struct elevator_queue *e, char *page)
{
	struct zios_data *zd = e->elevator_data;
	return sysfs_emit(page, "%llu\n", atomic64_read(&zd->merged_requests));
}

/* Top-app cgroup name */
static ssize_t zios_top_app_cgroup_name_show(struct elevator_queue *e,
					     char *page)
{
	struct zios_data *zd = e->elevator_data;
	return sysfs_emit(page, "%s\n", zd->top_app_cgroup_name);
}

static ssize_t zios_top_app_cgroup_name_store(struct elevator_queue *e,
					      const char *page, size_t count)
{
	struct zios_data *zd = e->elevator_data;
	char buf[256];
	int ret;

	ret = sscanf(page, "%255s", buf);
	if (ret != 1)
		return -EINVAL;

	strscpy(zd->top_app_cgroup_name, buf,
		sizeof(zd->top_app_cgroup_name));
	return count;
}

#define ZIOS_ATTR(name) __ATTR(name, 0644, zios_##name##_show, zios_##name##_store)

static struct elv_fs_entry zios_attrs[] = {
	/* Game mode params */
	ZIOS_ATTR(game_read_batch_max),
	ZIOS_ATTR(game_write_batch_max),
	ZIOS_ATTR(game_top_app_bias_pct),
	ZIOS_ATTR(game_write_starve_max),

	/* Normal mode params */
	ZIOS_ATTR(normal_read_batch_max),
	ZIOS_ATTR(normal_write_batch_max),
	ZIOS_ATTR(normal_top_app_bias_pct),
	ZIOS_ATTR(normal_write_starve_max),

	/* Runtime state */
	__ATTR(game_mode, 0444, zios_game_mode_show, NULL),
	__ATTR(nr_sync_reads, 0444, zios_nr_sync_reads_show, NULL),
	__ATTR(nr_async_reads, 0444, zios_nr_async_reads_show, NULL),
	__ATTR(nr_writes, 0444, zios_nr_writes_show, NULL),
	__ATTR(nr_top_app, 0444, zios_nr_top_app_show, NULL),

	/* Sector coalescing & write pressure */
	ZIOS_ATTR(read_coalesce_limit),
	ZIOS_ATTR(max_write_pending),

	/* Config */
	ZIOS_ATTR(async_depth),
	ZIOS_ATTR(top_app_cgroup_name),

	/* Stats */
	__ATTR(dispatched_reads, 0444, zios_dispatched_reads_show, NULL),
	__ATTR(dispatched_writes, 0444, zios_dispatched_writes_show, NULL),
	__ATTR(merged_requests, 0444, zios_merged_requests_show, NULL),

	/* Info */
	__ATTR(version, 0444, zios_version_show, NULL),

	__ATTR_NULL
};

/* ---------- elevator type ---------- */

static struct elevator_type mq_zios = {
	.ops = {
		.depth_updated		= zios_depth_updated,
		.limit_depth		= zios_limit_depth,
		.insert_requests	= zios_insert_requests,
		.dispatch_request	= zios_dispatch_request,
		.prepare_request	= zios_prepare_request,
		.finish_request		= zios_finish_request,
		.completed_request	= zios_completed_request,
		.bio_merge		= zios_bio_merge,
		.request_merged		= zios_request_merged,
		.requests_merged	= zios_merged_requests,
		.has_work		= zios_has_work,
		.init_sched		= zios_init_sched,
		.exit_sched		= zios_exit_sched,
		.init_hctx		= zios_init_hctx,
		.next_request		= elv_rb_latter_request,
		.former_request		= elv_rb_former_request,
	},

	.elevator_attrs = zios_attrs,
	.elevator_name = "zios",
	.elevator_owner = THIS_MODULE,
};
MODULE_ALIAS("mq-zios-iosched");

static int __init zios_init(void)
{
	pr_info("ZIOS I/O Scheduler v%s loaded\n", ZIOS_VERSION);
	return elv_register(&mq_zios);
}

static void __exit zios_exit(void)
{
	elv_unregister(&mq_zios);
}

module_init(zios_init);
module_exit(zios_exit);

MODULE_AUTHOR("Zenith Kernel");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("ZIOS - Zenith I/O Scheduler, game-aware and top-app-aware");
