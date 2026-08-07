#include "storage.h"
#include "features.h"
#include <SD.h>
#include <SD_MMC.h>

#if MAP_HAVE_USB_MSC
// Defined in main/idf_usb_msc.cpp, which only exists in the IDF build.
bool     usb_msc_mounted();
fs::FS  *usb_msc_fs();
#endif

static fs::FS     *g_fs   = nullptr;
static const char *g_name = "none";

// Adding USB mass storage
// -----------------------
// The consumers of this file are already written against fs::FS, so the
// application side of USB support is this function and nothing else - add a
// branch, and the tile cache, the manifest reader and the credential store all
// follow with no changes.
//
// The work that remains is entirely below Arduino:
//
//   1. USB host MSC is not exposed by the Arduino ESP32 core. It lives in
//      ESP-IDF's usb_host_msc component, which mounts a drive through FATFS at
//      a VFS path. Pulling an IDF managed component into a .ino sketch is the
//      awkward part - it generally means moving to an IDF component build, or
//      wrapping the driver in a library the sketch can include.
//
//   2. Once mounted at a VFS path, exposing it as an fs::FS is straightforward:
//      that is exactly what SD_MMC is - a VFSImpl pointed at /sdcard.
//
//   3. Hardware: confirmed available. The Tab5 block diagram shows USBA wired
//      to the P4 as USB2_OTG_D+/D- - the OTG-capable controller, not the
//      USB-Serial/JTAG that USBC uses for flashing - and SYS_USB5V feeding the
//      USBA connector, so the port can both host and supply bus power to a
//      drive. Nothing in the hardware blocks this; the obstacle is entirely
//      the driver plumbing in 1 and 2.
//
//      Note this is the same controller the board exposes for USB host in
//      general, so enabling it does not conflict with flashing over USBC.
//
// Order matters here. SD is checked first so that a card, when present, keeps
// winning - a user who has both plugged in should not have their cache move
// depending on enumeration timing.
static void pick() {
    if (SD_MMC.cardType() != CARD_NONE) { g_fs = &SD_MMC; g_name = "SD (SDMMC)"; return; }

#if MAP_HAVE_USB_MSC
    if (usb_msc_mounted()) { g_fs = usb_msc_fs(); g_name = "USB"; return; }
#endif

    g_fs = &SD; g_name = "SD (SPI)";
}

fs::FS *storage_fs() {
    if (!g_fs) pick();
    return g_fs;
}

const char *storage_name() {
    if (!g_fs) pick();
    return g_name;
}

void storage_rescan() {
    g_fs = nullptr;
    pick();
}
