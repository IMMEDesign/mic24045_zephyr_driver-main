# Subdirectories
add_subdirectory(drivers)

# Include headers
zephyr_include_directories(drivers)

# Register the custom driver as a Zephyr module
list(APPEND ZEPHYR_EXTRA_MODULES ${CMAKE_CURRENT_SOURCE_DIR}/drivers)

# Initialize the Zephyr build system, ensuring the module is included in the build process
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
