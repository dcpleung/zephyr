# SPDX-License-Identifier: Apache-2.0

# Find out the version of binutils' ld.
# Old ld may not support some flags we want to pass.
#
# Same search pattern as in cmake/linker/ld/target.cmake:
# ld.bfd first then ld.
execute_process(COMMAND ${CMAKE_C_COMPILER} --print-prog-name=ld.bfd
                OUTPUT_VARIABLE GNU_BINUTILS_LD
                OUTPUT_STRIP_TRAILING_WHITESPACE)

if(NOT EXISTS "${GNU_BINUTILS_LD}")
  execute_process(COMMAND ${CMAKE_C_COMPILER} --print-prog-name=ld
                  OUTPUT_VARIABLE GNU_BINUTILS_LD
                  OUTPUT_STRIP_TRAILING_WHITESPACE)
endif()

if(NOT EXISTS "${GNU_BINUTILS_LD}")
  message(FATAL_ERROR "Clang cannot find ld.bfd or ld via --print-prog-name")
endif()

execute_process(COMMAND ${GNU_BINUTILS_LD} --version
                OUTPUT_VARIABLE GNU_BINUTILS_LD_VER)

# Extract from something like ") 2.38" (note the parenthesis):
# - "GNU ld (GNU Binutils for Ubuntu) 2.34"
# - "GNU ld (Zephyr SDK 0.15.2) 2.38"
string(REGEX MATCHALL "\\) [0-9]+[.][0-9]+[.]?[0-9]*" GNU_BINUTILS_LD_VER ${GNU_BINUTILS_LD_VER})

# Extract the actual version
string(REGEX REPLACE "[^0-9]*([0-9]+\.[0-9]+).*" "\\1" GNU_BINUTILS_LD_VER ${GNU_BINUTILS_LD_VER})
