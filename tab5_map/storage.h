// storage.h - one place that decides which filesystem everything else uses.
//
// The choice was previously made in three files independently, each with its
// own copy of the same SD_MMC-else-SD test. That works only while there is
// exactly one answer; the moment a second backing store exists, three copies
// are three chances to disagree about which one is mounted, and the failure
// would be a subset of the code silently reading from a different device.
//
// Everything here is deliberately thin: it does not mount anything, it only
// reports what the boot sequence already mounted.

#pragma once
#include <FS.h>

// The filesystem holding /t (tile cache, manifests) and /wifi.bin.
// Never null - falls back to the SPI SD object, whose calls fail cleanly if
// nothing is mounted, which is the same behaviour the callers had before.
fs::FS *storage_fs();

// Human-readable name of the active store, for the boot line.
const char *storage_name();

// Re-run the selection. Call after mounting or removing a device.
void storage_rescan();
