# The sources live one level up, shared verbatim with the Arduino build.
#
# tab5_map.ino is deliberately not listed: it is an empty placeholder that
# exists only because arduino-cli requires a .ino matching the folder name.
# The sketch itself is tab5_map.cpp.

idf_component_register(
    SRCS
        "../tab5_map.cpp"
        "../mapengine.cpp"
        "../netsource.cpp"
        "../tilecache.cpp"
        "../wifistore.cpp"
        "../portal.cpp"
        "../gnss.cpp"
        "../storage.cpp"
        "../raster.c"
        "../mvt.c"
        "../inflate.c"
        "../pmtiles.c"
        "../style.c"
        "../tile_grid.c"
        "idf_usb_msc.cpp"
    INCLUDE_DIRS ".." "."
    REQUIRES
        arduino
        usb                      # USB host stack
        fatfs
        vfs
)

# Tells features.h which build this is. Deliberately explicit rather than
# inferred from ARDUINO, which is defined in both configurations.
target_compile_definitions(${COMPONENT_LIB} PUBLIC MAP_BUILD_IDF=1)
