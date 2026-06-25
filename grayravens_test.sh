#!/system/bin/sh
# GrayRavens — Post-Flash Test Suite v3
# Push: adb push grayravens_test.sh /sdcard/
# Run:  adb shell su -c sh /sdcard/grayravens_test.sh

OUTPUT=/sdcard/grayravens_test_results.txt
PASS=0
FAIL=0

log()   { echo "[$(date +%T)] $1" | tee -a $OUTPUT; }
pass()  { echo "  ✅ $1" | tee -a $OUTPUT; PASS=$((PASS+1)); }
fail()  { echo "  ❌ $1" | tee -a $OUTPUT; FAIL=$((FAIL+1)); }
sep()   { echo "" | tee -a $OUTPUT; echo "──── $1 ────" | tee -a $OUTPUT; echo "" | tee -a $OUTPUT; }

echo "" > $OUTPUT
echo "╔════════════════════════════════════════╗" | tee -a $OUTPUT
echo "║  GrayRavens Test Suite  $(date)  ║" | tee -a $OUTPUT
echo "╚════════════════════════════════════════╝" | tee -a $OUTPUT
echo "" | tee -a $OUTPUT

# Helper: check if driver is loaded (module or built-in)
check_driver() {
    local name=$1 modname=$2 sysfs_check=$3 sysfs_expected=$4
    local found=0

    # Check if module loaded
    [ -d "/sys/module/$modname" ] && found=1

    # Check via sysfs if provided
    if [ -n "$sysfs_check" ] && [ -f "$sysfs_check" ]; then
        local val=$(cat "$sysfs_check" 2>/dev/null)
        if [ -z "$sysfs_expected" ] || [ "$val" = "$sysfs_expected" ]; then
            found=1
        fi
    fi

    # Check dmesg as last resort (best-effort)
    if [ "$found" -eq 0 ]; then
        dmesg | grep -qi "$name" && found=1
    fi

    [ "$found" -eq 1 ] && pass "$name" || fail "$name NOT detected"
}

# ════════════════════════════════════════════════════
# Phase 0 — Boot Verification
# ════════════════════════════════════════════════════
sep "Phase 0 — Boot Verification"

check_driver "Zenith"     ""  "/sys/devices/system/cpu/cpu0/cpufreq/scaling_governor" "zenith"
check_driver "Hikari"     ""  "/sys/kernel/hikari/enable" "1"
check_driver "Vindicator" "vindicator" "" ""
check_driver "Vindicator Targets" "vindicator_targets" "" ""
check_driver "Shun"       "shun" "" ""
check_driver "Nocturne"   "nocturne" "" ""
check_driver "Oto"        "oto" "" ""
check_driver "Sen"        "sen" "" ""
check_driver "Equilibrium" "equilibrium" "" ""
check_driver "Kiryuu"     "kiryuu" "" ""
check_driver "Kage"       "kage" "" ""
check_driver "Tsuki"      "tsuki" "" ""
check_driver "Kasumi"     ""  "/sys/kernel/kasumi/enabled" "1"
check_driver "Iyashi"     ""  "/sys/kernel/iyashi/enabled" "1"
check_driver "Charger Guard" "thermal_charger_guard" "" ""
check_driver "GPU Switch" "zenith_gpu_switch" "" ""

# ════════════════════════════════════════════════════
# Phase 1 — Plist Fix (crash test)
# ════════════════════════════════════════════════════
sep "Phase 1 — Plist Fix (crash test)"

log "Writing to scaling_min_freq (this crashed before the fix)..."
echo 1000000 > /sys/devices/system/cpu/cpu0/cpufreq/scaling_min_freq 2>/dev/null
MIN=$(cat /sys/devices/system/cpu/cpu0/cpufreq/scaling_min_freq 2>/dev/null)
[ -n "$MIN" ] && pass "scaling_min_freq = $MIN — NO CRASH ✅" || fail "scaling_min_freq write failed"

log "Writing to scaling_max_freq..."
echo 2000000 > /sys/devices/system/cpu/cpu0/cpufreq/scaling_max_freq 2>/dev/null
MAX=$(cat /sys/devices/system/cpu/cpu0/cpufreq/scaling_max_freq 2>/dev/null)
[ -n "$MAX" ] && pass "scaling_max_freq = $MAX — NO CRASH ✅" || fail "scaling_max_freq write failed"

# ════════════════════════════════════════════════════
# Phase 2 — Module Params + Sysfs
# ════════════════════════════════════════════════════
sep "Phase 2 — Params & Sysfs"

check_param() {
    local val=$(cat "$1" 2>/dev/null)
    [ "$val" = "$2" ] && pass "$3 = $val" || fail "$3 = $val (expected $2)"
}

check_param /sys/module/shun/parameters/shun_enabled Y "shun_enabled"
check_param /sys/module/nocturne/parameters/enabled Y "nocturne_enabled"
check_param /sys/module/oto/parameters/oto_enabled Y "oto_enabled"
check_param /sys/module/sen/parameters/sen_enabled Y "sen_enabled"
check_param /sys/kernel/iyashi/enabled 1 "iyashi (sysfs)"
check_param /sys/kernel/hikari/enable 1 "hikari (sysfs)"

# ════════════════════════════════════════════════════
# Phase 3 — Vindicator Enforcement
# ════════════════════════════════════════════════════
sep "Phase 3 — Vindicator Enforcement"

GOV=$(cat /sys/devices/system/cpu/cpu0/cpufreq/scaling_governor 2>/dev/null)
[ "$GOV" = "zenith" ] && pass "Governor = zenith" || fail "Governor = $GOV"

log "Testing governor override..."
echo performance > /sys/devices/system/cpu/cpu0/cpufreq/scaling_governor 2>/dev/null
sleep 15
GOV=$(cat /sys/devices/system/cpu/cpu0/cpufreq/scaling_governor 2>/dev/null)
[ "$GOV" = "zenith" ] && pass "Governor enforced → zenith" || fail "Governor = $GOV"

log "Testing TCP override..."
echo cubic > /proc/sys/net/ipv4/tcp_congestion_control 2>/dev/null
sleep 15
TCP=$(cat /proc/sys/net/ipv4/tcp_congestion_control 2>/dev/null)
[ "$TCP" = "bbr" ] && pass "TCP enforced → bbr" || fail "TCP = $TCP"

log "Testing sched_latency override..."
LAT_BEFORE=$(cat /proc/sys/kernel/sched_latency_ns 2>/dev/null)
echo 5000000 > /proc/sys/kernel/sched_latency_ns 2>/dev/null
sleep 15
LAT=$(cat /proc/sys/kernel/sched_latency_ns 2>/dev/null)
if [ "$LAT" = "10000000" ]; then
    pass "sched_latency enforced → 10000000"
elif [ "$LAT" = "5000000" ]; then
    fail "sched_latency NOT enforced = $LAT (need reboot, initial=$LAT_BEFORE)"
else
    fail "sched_latency = $LAT (expected 10000000, initial was $LAT_BEFORE)"
fi

# ════════════════════════════════════════════════════
# Phase 4 — Other Drivers
# ════════════════════════════════════════════════════
sep "Phase 4 — Other Drivers"

SWAP=$(cat /sys/module/equilibrium/parameters/swappiness_balanced 2>/dev/null)
[ -n "$SWAP" ] && pass "Equilibrium swappiness = $SWAP" || fail "Equilibrium not responding"

KSU=$(cat /sys/module/kage/parameters/ksud_path 2>/dev/null)
[ -n "$KSU" ] && pass "Kage ksud_path = $KSU" || fail "Kage not responding"
KSU2=$(cat /sys/module/tsuki/parameters/ksud_path 2>/dev/null)
[ -n "$KSU2" ] && pass "Tsuki ksud_path = $KSU2" || fail "Tsuki not responding"

dmesg | grep -q "oto: PM QoS latency locked" && \
    pass "Oto PM QoS engaged" || log "Oto not triggered (needs audio)"

# ════════════════════════════════════════════════════
# Phase 5 — Thermal
# ════════════════════════════════════════════════════
sep "Phase 5 — Thermal"

IYA=$(cat /sys/kernel/iyashi/enabled 2>/dev/null)
[ "$IYA" = "1" ] && pass "Iyashi = $IYA" || fail "Iyashi = $IYA"
FLOOR=$(cat /sys/kernel/iyashi/floor_pct 2>/dev/null)
[ -n "$FLOOR" ] && pass "Iyashi floor_pct = $FLOOR" || fail "Iyashi floor missing"

KASUMI=$(cat /sys/kernel/kasumi/enabled 2>/dev/null)
[ "$KASUMI" = "1" ] && pass "Kasumi = $KASUMI" || fail "Kasumi = $KASUMI"

CG=$(cat /sys/module/thermal_charger_guard/parameters/batt_temp_thresh_decicelsius 2>/dev/null)
[ -n "$CG" ] && pass "Charger Guard threshold = $CG" || fail "Charger Guard threshold missing"

# ════════════════════════════════════════════════════
# Phase 6 — Scheduler
# ════════════════════════════════════════════════════
sep "Phase 6 — Scheduler"

GOV=$(cat /sys/devices/system/cpu/cpu0/cpufreq/scaling_governor 2>/dev/null)
[ "$GOV" = "zenith" ] && pass "Zenith governor active" || fail "Governor = $GOV"
HIK=$(cat /sys/kernel/hikari/enable 2>/dev/null)
[ "$HIK" = "1" ] && pass "Hikari = $HIK" || fail "Hikari = $HIK"

# ════════════════════════════════════════════════════
# Summary
# ════════════════════════════════════════════════════
sep "Results"
echo "  ✅ Passed: $PASS" | tee -a $OUTPUT
echo "  ❌ Failed: $FAIL" | tee -a $OUTPUT
echo "  📁 $OUTPUT"       | tee -a $OUTPUT
echo "" | tee -a $OUTPUT

if [ "$FAIL" -eq 0 ]; then
    echo "  🎉 ALL TESTS PASSED" | tee -a $OUTPUT
    echo "  GrayRavens is running clean!" | tee -a $OUTPUT
else
    echo "  ⚠️  $FAIL test(s) failed" | tee -a $OUTPUT
fi
echo "" | tee -a $OUTPUT
