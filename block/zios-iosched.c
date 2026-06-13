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
#include <linux/power_supply.h>
#include <linux/list_sort.h>

#include "blk.h"
#include "blk-mq.h"
#include "blk-mq-sched.h"
#include "blk-mq-debugfs.h"

#define ZIOS_VERSION "1.2.0"

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
	struct list_head	writeback_list;
	struct list_head	top_app_list;

	/* Counters */
	unsigned int		nr_sync_reads;
	unsigned int		nr_async_reads;
	unsigned int		nr_writes;
	unsigned int		nr_writeback;
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
	unsigned int		writeback_starve_count; /* nr of dispatches while writeback pending */
	unsigned int		writeback_starve_max; /* starve limit before flushing writeback */
	bool			writeback_dirty;

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

	/* ---------- Power efficiency ---------- */
	bool			power_efficient;
	bool			auto_power_efficient;
	unsigned int		power_efficient_battery_pct;
	unsigned long		last_power_check;
	struct {
		unsigned int	read_batch_max;
		unsigned int	write_batch_max;
		unsigned int	top_app_bias_pct;
		unsigned int	write_starve_max;
	} power_efficient_params;

	/* ---------- Workload detection ---------- */

	/* Debug logging toggle */
	bool			debug_log;

	/* Sampling interval (ms) */
	unsigned int		sample_interval_ms;
	unsigned long		last_sample;

	/* I/O size tracking in current window */
	u64			window_read_sectors;
	u64			window_write_sectors;
	u64			window_read_ops;
	u64			window_write_ops;

	/* Sequential vs random detection — compare each dispatch to last */
	sector_t		last_dispatch_sector;
	bool			last_dispatch_valid;
	u64			window_seq_ops;
	u64			window_rand_ops;

	/* Write pressure in current window */
	unsigned int		window_peak_write_pending;

	/* Current detected workload profile */
	enum {
		ZIOS_WORKLOAD_BALANCED	= 0,
		ZIOS_WORKLOAD_SEQUENTIAL,
		ZIOS_WORKLOAD_RANDOM,
		ZIOS_WORKLOAD_WRITE_HEAVY,
	} workload;
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
	sector_t best_delta = U64_MAX;
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

/* Forward declaration for workload analysis */
static void zios_analyze_workload(struct zios_data *zd);

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
	} else if (current->flags & PF_SWAPWRITE) {
		/* Writeback (flusher thread) — lowest priority */
		list_add_tail(&rq->queuelist, &zd->writeback_list);
		zd->nr_writeback++;
		zd->writeback_dirty = true;
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

/* ---------- power efficiency helpers ---------- */

/*
 * Check battery level and update power_efficient flag.
 * Called from dispatch context, throttled to every 60 seconds.
 */
static void zios_check_battery(struct zios_data *zd)
{
	struct power_supply *psy;
	union power_supply_propval val;
	int ret;

	if (time_before(jiffies, zd->last_power_check + 60 * HZ))
		return;
	zd->last_power_check = jiffies;

	psy = power_supply_get_by_name("battery");
	if (IS_ERR_OR_NULL(psy))
		return;

	ret = power_supply_get_property(psy, POWER_SUPPLY_PROP_CAPACITY, &val);
	if (ret == 0) {
		bool low = val.intval <= (int)zd->power_efficient_battery_pct;
		bool on_ac;

		on_ac = power_supply_is_system_supplied() > 0;

		if (!on_ac && low && !zd->power_efficient) {
			zd->power_efficient = true;
			if (zd->debug_log)
				pr_debug("zios: battery %d%% <= %u%% -> power efficient ON\n",
					 val.intval, zd->power_efficient_battery_pct);
		} else if ((on_ac || !low) && zd->power_efficient) {
			zd->power_efficient = false;
			if (zd->debug_log)
				pr_debug("zios: battery %d%% > %u%% or AC -> power efficient OFF\n",
					 val.intval, zd->power_efficient_battery_pct);
		}
	} else if (zd->power_efficient) {
		/* Can't read battery — disable auto mode */
		zd->power_efficient = false;
	}

	power_supply_put(psy);
}

/* ---------- dispatch ---------- */

static struct request *zios_dispatch_request(struct blk_mq_hw_ctx *hctx)
{
	struct zios_data *zd = hctx->queue->elevator->elevator_data;
	struct request *rq = NULL;
	bool game_mode;
	unsigned int rbatch, wbatch, bias_pct, starve_max;
	unsigned int coalesce;

	/* Refresh game mode every 100 ms */
	if (time_after(jiffies, zd->last_game_check + HZ / 10)) {
		zd->game_mode = zenith_is_game_mode_active();
		zd->last_game_check = jiffies;
	}
	game_mode = zd->game_mode;

	/*
	 * Check battery before taking spinlock — power supply API may sleep.
	 * Auto-detection is throttled to every 60 seconds.
	 */
	if (zd->auto_power_efficient &&
	    time_after(jiffies, zd->last_power_check + 60 * HZ))
		zios_check_battery(zd);

	spin_lock(&zd->lock);

	if (game_mode) {
		rbatch    = zd->game_params.read_batch_max;
		wbatch    = zd->game_params.write_batch_max;
		bias_pct  = zd->game_params.top_app_bias_pct;
		starve_max = zd->game_params.write_starve_max;
	} else if (zd->power_efficient) {
		/* Power-efficient mode: conservative settings to save battery */
		rbatch    = zd->power_efficient_params.read_batch_max;
		wbatch    = zd->power_efficient_params.write_batch_max;
		bias_pct  = zd->power_efficient_params.top_app_bias_pct;
		starve_max = zd->power_efficient_params.write_starve_max;
	} else {
		rbatch    = zd->normal_params.read_batch_max;
		wbatch    = zd->normal_params.write_batch_max;
		bias_pct  = zd->normal_params.top_app_bias_pct;
		starve_max = zd->normal_params.write_starve_max;
	}
	coalesce = zd->power_efficient ?
		min(zd->read_coalesce_limit, 4u) :
		zd->read_coalesce_limit;

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

	/*
	 * ----- Tier 4: Writeback (absolute lowest priority) -----
	 * Dispatched only when nothing else pending OR starved beyond limit.
	 */
	if (zd->nr_writeback > 0) {
		if (zd->writeback_starve_count >= zd->writeback_starve_max ||
		    (!zd->nr_sync_reads && !zd->nr_async_reads &&
		     !zd->nr_writes && !zd->nr_top_app)) {

			if (zd->writeback_dirty && zd->nr_writeback > 1) {
				list_sort(NULL, &zd->writeback_list,
					  zios_cmp_write_lba);
				zd->writeback_dirty = false;
			}

			rq = list_first_entry_or_null(&zd->writeback_list,
						      struct request, queuelist);
			if (rq) {
				list_del_init(&rq->queuelist);
				zd->nr_writeback--;
				zd->writeback_starve_count = 0;
				goto done;
			}
		}

		/* Bump writeback starve count on every dispatch when pending */
		zd->writeback_starve_count++;
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
		if (op_is_write(req_op(rq))) {
			atomic64_inc(&zd->dispatched_writes);

			/* Workload: track write I/O size */
			zd->window_write_sectors += blk_rq_sectors(rq);
			zd->window_write_ops++;
		} else {
			atomic64_inc(&zd->dispatched_reads);

			/* Workload: track read I/O size */
			zd->window_read_sectors += blk_rq_sectors(rq);
			zd->window_read_ops++;
		}

		/* Workload: sequential vs random detection */
		if (zd->last_dispatch_valid) {
			sector_t pos = blk_rq_pos(rq);
			sector_t last = zd->last_dispatch_sector;
			sector_t delta = (pos > last) ? (pos - last) : (last - pos);

			if (delta <= 32) /* within 16KB = sequential */
				zd->window_seq_ops++;
			else
				zd->window_rand_ops++;
		}
		zd->last_dispatch_sector = blk_rq_pos(rq) + blk_rq_sectors(rq);
		zd->last_dispatch_valid = true;

		/* Workload: peak write pressure */
		if (zd->nr_writes > zd->window_peak_write_pending)
			zd->window_peak_write_pending = zd->nr_writes;

		/*
		 * Sample workload periodically.
		 * Skip during game mode — game_params already tune for gaming.
		 */
		if (!zd->game_mode &&
		    time_after(jiffies, zd->last_sample +
			       msecs_to_jiffies(zd->sample_interval_ms))) {
			zios_analyze_workload(zd);
			zd->last_sample = jiffies;
		}

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
	       !list_empty_careful(&zd->writeback_list) ||
	       !list_empty_careful(&zd->top_app_list);
}

static void zios_limit_depth(unsigned int op, struct blk_mq_alloc_data *data)
{
	struct zios_data *zd = data->q->elevator->elevator_data;
	unsigned int depth;

	if (op_is_sync(op) && !op_is_write(op))
		return;

	/* When battery-saving, cap queue depth to reduce I/O concurrency */
	depth = zd->async_depth;
	if (zd->power_efficient)
		depth = min(depth, zd->async_depth / 2);

	data->shallow_depth = zios_to_word_depth(data->hctx, depth);
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

/* ---------- workload analysis ---------- */

/*
 * zios_analyze_workload - analyze recent I/O patterns and tune params.
 * Called periodically from dispatch context holding zd->lock.
 */
static void zios_analyze_workload(struct zios_data *zd)
{
	u64 total_read_sz, total_write_sz;
	u64 total_ops, read_ops, write_ops;
	unsigned int seq_ratio, rand_ratio;
	unsigned int write_ratio;
	unsigned int avg_read_sz_kb, avg_write_sz_kb;

	total_read_sz  = zd->window_read_sectors;
	total_write_sz = zd->window_write_sectors;
	read_ops  = zd->window_read_ops;
	write_ops = zd->window_write_ops;
	total_ops = read_ops + write_ops;

	if (total_ops < 8) {
		/* Not enough data — keep previous profile */
		goto reset_window;
	}

	seq_ratio  = zd->window_seq_ops * 100 / total_ops;
	rand_ratio = zd->window_rand_ops * 100 / total_ops;
	write_ratio = write_ops * 100 / total_ops;

	avg_read_sz_kb  = read_ops  ? (total_read_sz  * 512 / 1024 / read_ops)  : 0;
	avg_write_sz_kb = write_ops ? (total_write_sz * 512 / 1024 / write_ops) : 0;

	if (zd->debug_log)
		pr_debug("zios: workload: seq=%u%% rand=%u%% write=%u%% "
			 "avg_r=%uKB avg_w=%uKB peak_w=%u\n",
			 seq_ratio, rand_ratio, write_ratio,
			 avg_read_sz_kb, avg_write_sz_kb,
			 zd->window_peak_write_pending);

	/* ---- heuristics ---- */

	if (seq_ratio > 70 && write_ratio < 40) {
		/* Strong sequential read pattern — benchmark / streaming */
		zd->workload = ZIOS_WORKLOAD_SEQUENTIAL;
		zd->read_coalesce_limit = 24;
		zd->normal_params.read_batch_max = 32;

		if (zd->debug_log)
			pr_debug("zios: -> SEQUENTIAL (read coalesce=%u, rbatch=%u)\n",
				 zd->read_coalesce_limit,
				 zd->normal_params.read_batch_max);

	} else if (rand_ratio > 60 && read_ops > write_ops) {
		/* Random-read dominated — app launch, UI */
		zd->workload = ZIOS_WORKLOAD_RANDOM;
		zd->read_coalesce_limit = 4;
		zd->normal_params.read_batch_max = 8;
		zd->normal_params.write_starve_max = 5;

		if (zd->debug_log)
			pr_debug("zios: -> RANDOM (coalesce=%u, rbatch=%u, starve=%u)\n",
				 zd->read_coalesce_limit,
				 zd->normal_params.read_batch_max,
				 zd->normal_params.write_starve_max);

	} else if (write_ratio > 55 && total_write_sz > 0) {
		/* Write-heavy — install, download, file copy */
		zd->workload = ZIOS_WORKLOAD_WRITE_HEAVY;
		zd->normal_params.write_batch_max = 32;
		zd->normal_params.write_starve_max = 2;
		zd->max_write_pending = 128;

		if (zd->debug_log)
			pr_debug("zios: -> WRITE_HEAVY (wbatch=%u, starve=%u, maxw=%u)\n",
				 zd->normal_params.write_batch_max,
				 zd->normal_params.write_starve_max,
				 zd->max_write_pending);

	} else {
		/* Mixed — balanced defaults */
		if (zd->workload != ZIOS_WORKLOAD_BALANCED) {
			zd->read_coalesce_limit = 8;
			zd->normal_params.read_batch_max = 16;
			zd->normal_params.write_batch_max = 16;
			zd->normal_params.write_starve_max = 3;
			zd->max_write_pending = 64;

			if (zd->debug_log)
				pr_debug("zios: -> BALANCED (restored defaults)\n");
		}
		zd->workload = ZIOS_WORKLOAD_BALANCED;
	}

reset_window:
	zd->window_read_sectors  = 0;
	zd->window_write_sectors = 0;
	zd->window_read_ops      = 0;
	zd->window_write_ops     = 0;
	zd->window_seq_ops       = 0;
	zd->window_rand_ops      = 0;
	zd->window_peak_write_pending = 0;
	zd->last_dispatch_valid  = false;
}

/* ---------- init / exit ---------- */

static void zios_exit_sched(struct elevator_queue *e)
{
	struct zios_data *zd = e->elevator_data;

	WARN_ON_ONCE(!list_empty(&zd->sync_read_list));
	WARN_ON_ONCE(!list_empty(&zd->async_read_list));
	WARN_ON_ONCE(!list_empty(&zd->write_list));
	WARN_ON_ONCE(!list_empty(&zd->writeback_list));
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
	INIT_LIST_HEAD(&zd->writeback_list);
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
	zd->nr_writeback = 0;
	zd->writeback_starve_count = 0;
	zd->writeback_starve_max = 20;
	zd->writeback_dirty = false;

	/* Power efficiency */
	zd->power_efficient = false;
	zd->auto_power_efficient = true;
	zd->power_efficient_battery_pct = 15;
	zd->last_power_check = jiffies;
	zd->power_efficient_params.read_batch_max   = 8;
	zd->power_efficient_params.write_batch_max  = 24;
	zd->power_efficient_params.top_app_bias_pct = 30;
	zd->power_efficient_params.write_starve_max = 8;

	/* Workload detection */
	zd->debug_log = false;
	zd->sample_interval_ms = 500;
	zd->last_sample = jiffies;
	zd->window_read_sectors = 0;
	zd->window_write_sectors = 0;
	zd->window_read_ops = 0;
	zd->window_write_ops = 0;
	zd->window_seq_ops = 0;
	zd->window_rand_ops = 0;
	zd->window_peak_write_pending = 0;
	zd->last_dispatch_sector = 0;
	zd->last_dispatch_valid = false;
	zd->workload = ZIOS_WORKLOAD_BALANCED;

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

ZIOS_RW(power_efficient_read_batch_max,	power_efficient_params.read_batch_max,	1, 256)
ZIOS_RW(power_efficient_write_batch_max,	power_efficient_params.write_batch_max,	1, 256)
ZIOS_RW(power_efficient_top_app_bias_pct,	power_efficient_params.top_app_bias_pct,	0, 100)
ZIOS_RW(power_efficient_write_starve_max,	power_efficient_params.write_starve_max,	1, 100)

ZIOS_RO(game_mode, game_mode)
ZIOS_RO(nr_sync_reads,  nr_sync_reads)
ZIOS_RO(nr_async_reads, nr_async_reads)
ZIOS_RO(nr_writes, nr_writes)
ZIOS_RO(nr_writeback, nr_writeback)
ZIOS_RO(nr_top_app, nr_top_app)

ZIOS_RW(read_coalesce_limit, read_coalesce_limit, 1, 64)
ZIOS_RW(max_write_pending, max_write_pending, 1, 4096)
ZIOS_RW(async_depth, async_depth, 1, INT_MAX)

ZIOS_RW(writeback_starve_max, writeback_starve_max, 1, 100)

ZIOS_RW(power_efficient_battery_pct, power_efficient_battery_pct, 1, 100)

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

/* Workload profile (read-only) */
static ssize_t zios_workload_show(struct elevator_queue *e, char *page)
{
	struct zios_data *zd = e->elevator_data;
	const char *profile;

	switch (zd->workload) {
	case ZIOS_WORKLOAD_SEQUENTIAL:
		profile = "sequential";
		break;
	case ZIOS_WORKLOAD_RANDOM:
		profile = "random";
		break;
	case ZIOS_WORKLOAD_WRITE_HEAVY:
		profile = "write_heavy";
		break;
	default:
		profile = "balanced";
		break;
	}

	return sysfs_emit(page, "%s\n", profile);
}

/* Power efficiency toggle */
static ssize_t zios_power_efficient_show(struct elevator_queue *e, char *page)
{
	struct zios_data *zd = e->elevator_data;
	return sysfs_emit(page, "%d\n", zd->power_efficient);
}

static ssize_t zios_power_efficient_store(struct elevator_queue *e,
					  const char *page, size_t count)
{
	struct zios_data *zd = e->elevator_data;
	bool val;
	int ret;

	ret = kstrtobool(page, &val);
	if (ret)
		return -EINVAL;

	zd->power_efficient = val;
	if (zd->debug_log)
		pr_debug("zios: power_efficient set to %d\n", val);
	return count;
}

/* Auto power efficiency toggle */
static ssize_t zios_auto_power_efficient_show(struct elevator_queue *e, char *page)
{
	struct zios_data *zd = e->elevator_data;
	return sysfs_emit(page, "%d\n", zd->auto_power_efficient);
}

static ssize_t zios_auto_power_efficient_store(struct elevator_queue *e,
					       const char *page, size_t count)
{
	struct zios_data *zd = e->elevator_data;
	bool val;
	int ret;

	ret = kstrtobool(page, &val);
	if (ret)
		return -EINVAL;

	zd->auto_power_efficient = val;
	if (zd->debug_log)
		pr_debug("zios: auto_power_efficient set to %d\n", val);
	return count;
}

/* Debug log toggle */
static ssize_t zios_debug_log_show(struct elevator_queue *e, char *page)
{
	struct zios_data *zd = e->elevator_data;
	return sysfs_emit(page, "%d\n", zd->debug_log);
}

static ssize_t zios_debug_log_store(struct elevator_queue *e,
				    const char *page, size_t count)
{
	struct zios_data *zd = e->elevator_data;
	bool val;
	int ret;

	ret = kstrtobool(page, &val);
	if (ret)
		return -EINVAL;

	zd->debug_log = val;
	return count;
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

	/* Power efficiency params */
	ZIOS_ATTR(power_efficient),
	ZIOS_ATTR(auto_power_efficient),
	ZIOS_ATTR(power_efficient_battery_pct),
	ZIOS_ATTR(power_efficient_read_batch_max),
	ZIOS_ATTR(power_efficient_write_batch_max),
	ZIOS_ATTR(power_efficient_top_app_bias_pct),
	ZIOS_ATTR(power_efficient_write_starve_max),

	/* Runtime state */
	__ATTR(game_mode, 0444, zios_game_mode_show, NULL),
	__ATTR(nr_sync_reads, 0444, zios_nr_sync_reads_show, NULL),
	__ATTR(nr_async_reads, 0444, zios_nr_async_reads_show, NULL),
	__ATTR(nr_writes, 0444, zios_nr_writes_show, NULL),
	__ATTR(nr_writeback, 0444, zios_nr_writeback_show, NULL),
	__ATTR(nr_top_app, 0444, zios_nr_top_app_show, NULL),

	/* Sector coalescing & write pressure */
	ZIOS_ATTR(read_coalesce_limit),
	ZIOS_ATTR(max_write_pending),
	ZIOS_ATTR(writeback_starve_max),

	/* Workload & debugging */
	__ATTR(workload, 0444, zios_workload_show, NULL),
	ZIOS_ATTR(debug_log),

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
