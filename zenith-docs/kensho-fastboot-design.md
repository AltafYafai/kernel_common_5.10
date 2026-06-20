# Kenshō — Fastboot OEM Dump Protocol

## Overview

When a kernel panic occurs, Kenshō writes the crash log to a reserved
DRAM region that survives reboots. If the device cannot boot Android
or recovery (early bootloop), the **bootloader (fastboot mode)** is the
last point of access.

This document describes how a bootloader engineer can implement
a `fastboot oem kensho dump` command that retrieves the crash log
from Kenshō's reserved memory region.

---

## Physical Memory Layout

Kenshō reserves 128 KB of physical DRAM at early boot via
`memblock_phys_alloc()`.  The physical address is printed to the
kernel log at every boot:

```
kensho: persistent region at 0xABCD0000 (131072 bytes)
```

The region has the following layout:

```
Offset  Size    Field
──────  ──────  ───────────────────────────────────
0x0000     8    magic     — u64: 0x4B454E53484F (\"KENSHO\")
0x0008     8    timestamp — u64: ktime_get_real_seconds()
0x0010     4    size      — u32: bytes of log data
0x0014    20    reserved  (zeroed)
0x0028   ───    log data  (up to 131072 - 40 = 131032 bytes)
```

### Magic Value

The magic is `0x4B454E53484F` — the ASCII encoding of "KENSHO"
padded with a null byte in the upper bits.  If the bootloader reads
this value at offset 0, there is valid crash data.

### Determining the Address

The bootloader needs to know the physical address of the reserved
region.  There are several ways to communicate this:

1. **Kernel cmdline** (recommended)
   Kenshō can append `kensho.addr=0xABCD0000 kensho.size=0x20000`
   to the kernel cmdline in the `/chosen/bootargs` DT property.
   
2. **Device Tree** (cleanest)
   Kenshō can add a `/chosen/kensho` node:
   ```
   / {
       chosen {
           kensho {
               phys-addr = <0xABCD0000>;
               size = <0x00020000>;
           };
       };
   };
   ```

3. **Hardcoded per-platform** (simplest for bootloader)
   The bootloader can scan for the `kensho:` dmesg pattern via
   a known pstore/ramoops region, or the address can be hardcoded
   per device if the memory layout is deterministic.

---

## Fastboot Protocol

### Command: `fastboot oem kensho dump`

**Behavior:**
1. Bootloader reads 128 KB from Kenshō's reserved physical address.
2. Checks `magic == 0x4B454E53484F`.
3. If magic matches:
   - Reads `size` (u32 at offset 0x10).
   - Reads `size` bytes of log data starting at offset 0x28.
   - Outputs the raw text to the fastboot console:
     ```
     (bootloader) === Kenshō Crash Log ===
     (bootloader) Timestamp: 1701234567
     (bootloader) --- LOG START ---
     (bootloader) [log content]
     (bootloader) --- LOG END ---
     OKAY [0.050s]
     ```
   - **Optionally** clears the magic so the log is consumed.

4. If magic does NOT match:
   ```
   (bootloader) No Kenshō crash log found.
   OKAY [0.001s]
   ```

### Implementation Notes (Qualcomm ABL / Little Kernel)

For **Qualcomm ABL** (UEFI-based) or **Little Kernel (LK)**:

```c
/* Pseudo-code for fastboot oem handler */
void oem_kensho_dump(void)
{
    uint64_t addr = KENSHO_PERSIST_ADDR;  /* from DT or hardcoded */
    struct kensho_header *hdr = (void *)addr;
    char *data = (void *)(addr + sizeof(*hdr));

    if (hdr->magic != 0x4B454E53484FULL) {
        fastboot_okay("No Kenshō crash log found.");
        return;
    }

    fastboot_info("=== Kenshō Crash Log ===");
    fastboot_info("Timestamp: %llu", hdr->timestamp);

    /* Output the log in chunks (fastboot has line limits) */
    char *p = data;
    for (uint32_t i = 0; i < hdr->size; i += 80) {
        char line[81];
        memcpy(line, p + i, min(80u, hdr->size - i));
        line[min(80u, hdr->size - i)] = '\0';
        fastboot_info(line);
    }

    fastboot_okay("Log dumped.");
}
```

### Additional Commands

| Command | Purpose |
|---------|---------|
| `fastboot oem kensho status` | Returns "CRASH LOG FOUND" or "No data" |
| `fastboot oem kensho clear` | Clears the reserved region (consumes the log) |
| `fastboot oem kensho info` | Returns address, size, timestamp |

---

## Boot Flow Integration

```
Power On
   ↓
Bootloader (ABL/LK)
   ├── Initialises DRAM
   ├── Checks Kenshō reserved memory ← fastboot access here
   ├── Reads Kenshō magic            ← "is there a log?"
   └── Normal boot or fastboot
         ↓
Kernel 5.10 boots
   ├── early_initcall: Kenshō reserves 128 KB via memblock
   ├── Checks reserved region ← "is there a log from last boot?"
   └── If found: exposes via /sys/kernel/kensho/
```

The reserved memory region is at a FIXED physical address from boot
to boot.  Kenshō's `early_initcall` reserves it, and the bootloader
can read the same address without any coordination needed.

---

## Testing

1. Trigger a kernel panic:
   ```bash
   echo c > /proc/sysrq-trigger
   ```

2. Force reboot to bootloader:
   ```bash
   adb reboot bootloader
   ```

3. Dump the log:
   ```bash
   fastboot oem kensho dump
   ```

4. Verify the log contains the expected metadata and stack trace.
