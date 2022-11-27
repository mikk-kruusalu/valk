# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

file(MAKE_DIRECTORY
  "C:/Users/Mikk/esp/esp-idf/components/bootloader/subproject"
  "C:/Users/Mikk/Documents/ESP/valk/build/bootloader"
  "C:/Users/Mikk/Documents/ESP/valk/build/bootloader-prefix"
  "C:/Users/Mikk/Documents/ESP/valk/build/bootloader-prefix/tmp"
  "C:/Users/Mikk/Documents/ESP/valk/build/bootloader-prefix/src/bootloader-stamp"
  "C:/Users/Mikk/Documents/ESP/valk/build/bootloader-prefix/src"
  "C:/Users/Mikk/Documents/ESP/valk/build/bootloader-prefix/src/bootloader-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "C:/Users/Mikk/Documents/ESP/valk/build/bootloader-prefix/src/bootloader-stamp/${subDir}")
endforeach()
