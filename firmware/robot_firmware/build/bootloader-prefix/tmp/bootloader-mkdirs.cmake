# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

file(MAKE_DIRECTORY
  "/home/pushkar/.espressif/v6.0.1/esp-idf/components/bootloader/subproject"
  "/home/pushkar/outdoor-autonomous-robot/firmware/robot_firmware/build/bootloader"
  "/home/pushkar/outdoor-autonomous-robot/firmware/robot_firmware/build/bootloader-prefix"
  "/home/pushkar/outdoor-autonomous-robot/firmware/robot_firmware/build/bootloader-prefix/tmp"
  "/home/pushkar/outdoor-autonomous-robot/firmware/robot_firmware/build/bootloader-prefix/src/bootloader-stamp"
  "/home/pushkar/outdoor-autonomous-robot/firmware/robot_firmware/build/bootloader-prefix/src"
  "/home/pushkar/outdoor-autonomous-robot/firmware/robot_firmware/build/bootloader-prefix/src/bootloader-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "/home/pushkar/outdoor-autonomous-robot/firmware/robot_firmware/build/bootloader-prefix/src/bootloader-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "/home/pushkar/outdoor-autonomous-robot/firmware/robot_firmware/build/bootloader-prefix/src/bootloader-stamp${cfgdir}") # cfgdir has leading slash
endif()
