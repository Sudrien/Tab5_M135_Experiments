// idf_usb_msc.cpp - USB mass storage, IDF builds only.
//
// UNTESTED. This is the shape of the driver rather than known-working code -
// I have no IDF toolchain or a drive to try it against. The API names below
// follow esp_usb_msc / usb_host_msc as documented; check them against the
// version you actually pull in.
//
// Why this file exists at all: storage.cpp has a single gated branch calling
// usb_msc_mounted() and usb_msc_fs(). Everything else in the project already
// works through fs::FS, so nothing else needs to change - the tile cache, the
// manifest reader and the credential store all follow automatically.
//
// The design point worth keeping: mount the drive at a VFS path and hand back
// an fs::FS over it. That is exactly what SD_MMC is - a VFSImpl pointed at
// /sdcard - so the rest of the code cannot tell the difference.

#include "../features.h"

#if MAP_HAVE_USB_MSC

#include <FS.h>
#include <vfs_api.h>
#include <Arduino.h>

#include "usb/usb_host.h"
#include "usb/msc_host.h"
#include "usb/msc_host_vfs.h"

static const char *USB_MOUNT = "/usb";

static bool                 s_mounted = false;
static msc_host_device_handle_t s_dev = nullptr;
static msc_host_vfs_handle_t    s_vfs = nullptr;

// fs::FS over the VFS mount, mirroring how SD_MMC is constructed.
static fs::FS s_fs = fs::FS(fs::FSImplPtr(new VFSImpl()));

static void msc_event(const msc_host_event_t *e, void *arg) {
    (void)arg;
    if (e->event == MSC_DEVICE_CONNECTED) {
        Serial.println("usb: mass storage connected");
    } else if (e->event == MSC_DEVICE_DISCONNECTED) {
        Serial.println("usb: mass storage removed");
        s_mounted = false;
        // Deliberately not remounting from the callback: storage_rescan()
        // must be the thing that re-picks, or the tile cache would keep
        // writing through a handle to a device that is gone.
    }
}

// Bring up the host stack and mount the first drive found. Safe to call when
// nothing is plugged in - it simply reports false.
bool usb_msc_begin() {
    if (s_mounted) return true;

    const usb_host_config_t host_cfg = { .skip_phy_setup = false,
                                         .intr_flags = ESP_INTR_FLAG_LEVEL1 };
    if (usb_host_install(&host_cfg) != ESP_OK) {
        Serial.println("usb: host install failed");
        return false;
    }

    const msc_host_driver_config_t drv_cfg = {
        .create_backround_task = true,
        .task_priority = 5,
        .stack_size = 4096,
        .callback = msc_event,
    };
    if (msc_host_install(&drv_cfg) != ESP_OK) {
        Serial.println("usb: msc driver install failed");
        return false;
    }

    // Enumeration is asynchronous; the caller is expected to retry rather than
    // block boot on a device that may not be present at all.
    return false;
}

bool usb_msc_mounted() { return s_mounted; }

fs::FS *usb_msc_fs() { return &s_fs; }

#endif  // MAP_HAVE_USB_MSC
