cmake_minimum_required(VERSION 3.15)
set(CMAKE_TOOLCHAIN_FILE ${CMAKE_SOURCE_DIR}/cmake/ArduinoToolchain.cmake)

set(CMAKE_CXX_STANDARD 17)
set(PROJECT_NAME EqMount)

set(${PROJECT_NAME}_BOARD nano)
set(ARDUINO_CPU atmega328old)
project(${PROJECT_NAME})

# Define additional source and header files or default arduino sketch files
set(${PROJECT_NAME}_SRCS eq_mount.cpp)
set(${PROJECT_NAME}_HDRS CA4998.hpp)

### Additional static libraries to include in the target.
# set(${PROJECT_NAME}_LIBS)

### Main sketch file
# set(${PROJECT_NAME}_SKETCH eq_mount.ino)

### Add project directories into the build
# add_subdirectory()

### Additional settings to add non-standard or your own Arduino libraries.
# For this example (libs will contain additional arduino libraries)
# An Arduino library my_lib will contain files in libs/my_lib/: my_lib.h, my_lib.cpp + any other cpp files
link_directories(${CMAKE_CURRENT_SOURCE_DIR}/../libraries)

# For nested library sources replace ${LIB_NAME} with library name for each library
# set(_RECURSE true)

#### Additional settings for programmer. From programmers.txt
set(${PROJECT_NAME}_PROGRAMMER avrispmkii)

# See documentation for udevadm and /etc/udev/rules.d/10-usb-serial.rules
# contents: SUBSYSTEM=="tty", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="7523", SYMLINK+="ttyNano"
if(LINUX)
    set(${PROJECT_NAME}_PORT /dev/ttyNano)
    message("Me Thinks LINUX")
else()
    set(${PROJECT_NAME}_PORT /dev/cu.usbserial-141310)
    message("Me Thinks NOT LINUX")
endif()

# set(nano.upload.speed 9600)

## Verbose build process
set(${PROJECT_NAME}_AFLAGS -v)

generate_arduino_firmware(${PROJECT_NAME})

print_board_list()
print_programmer_list()
