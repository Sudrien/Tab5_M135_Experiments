// features.h - what this build can do.
//
// The Arduino ESP32 core is ESP-IDF underneath, so this is not two codebases.
// The same sources build either way; the difference is that an IDF project can
// pull in managed components (USB host MSC, for one) that the Arduino build has
// no mechanism to add. Anything in that category is gated here rather than
// scattered through the files that use it.
//
// MAP_BUILD_IDF is set by the IDF build (see CMakeLists.txt). It is not derived
// from ARDUINO, because arduino-esp32 used as an IDF component defines ARDUINO
// too - the two are not mutually exclusive and testing for it would silently
// select the wrong branch in exactly the configuration this exists to support.

#pragma once

#ifndef MAP_BUILD_IDF
#define MAP_BUILD_IDF 0
#endif

// ---- USB mass storage ------------------------------------------------------
// Needs ESP-IDF's usb_host_msc component. The hardware is capable either way:
// the Tab5 block diagram shows USBA on the P4's USB2_OTG_D+/D- with SYS_USB5V
// feeding the connector, so it can host a drive and power it. Only the driver
// is out of reach from Arduino.
#ifndef MAP_HAVE_USB_MSC
#  if MAP_BUILD_IDF
#    define MAP_HAVE_USB_MSC 1
#  else
#    define MAP_HAVE_USB_MSC 0
#  endif
#endif

// ---- what is NOT gained by building under IDF -------------------------------
// exFAT. Worth stating explicitly because it is the obvious assumption and it
// is wrong: ESP-IDF hardcodes FF_FS_EXFAT to 0 in components/fatfs/src/ffconf.h
// and exposes no Kconfig option for it, so neither build can read an exFAT
// card. Changing that means vendoring the fatfs component and maintaining the
// patch, which is a fork rather than a setting.
//
// This is why a card over 32 GB, formatted by Windows or a camera, will not
// mount on either build and mountSD() offers to reformat it as FAT instead.

// A short description of the build, for the boot banner - so a log always says
// which variant produced it.
static inline const char *map_build_flavour() {
#if MAP_BUILD_IDF
    return MAP_HAVE_USB_MSC ? "idf+usb" : "idf";
#else
    return "arduino";
#endif
}
