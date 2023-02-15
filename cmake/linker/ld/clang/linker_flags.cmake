# The coverage linker flag is specific for clang.
if (NOT CONFIG_COVERAGE_GCOV)
  set_property(TARGET linker PROPERTY coverage --coverage)
endif()

# Clang has a tendency to mark data and functions with RWX permissions
# in object files, which results in linker marking segments containing
# them as RWX too. Same goes for stacks being marked executable.
# So ignore those warnings for now.
set_property(TARGET linker
             PROPERTY no_warn_rwx_segments "${LINKERFLAGPREFIX},--no-warn-rwx-segments")
set_property(TARGET linker
             PROPERTY no_warn_execstack "${LINKERFLAGPREFIX},--no-warn-execstack")
