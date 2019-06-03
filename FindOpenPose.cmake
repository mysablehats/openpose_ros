# Report other information
set(OpenPose_INCLUDE_DIRS "${_prefix}/include/openpose")
set(OpenPose_VERSION_MAJOR @OpenPose_VERSION_MAJOR@)
set(OpenPose_VERSION_MINOR @OpenPose_VERSION_MINOR@)
set(OpenPose_VERSION_PATCH @OpenPose_VERSION_PATCH@)
set(OpenPose_VERSION @OpenPose_VERSION@)

# Check that the user requested components 
# are actually targets that are part of this build
if (OpenPose_FIND_COMPONENTS)
  foreach (comp ${OpenPose_FIND_COMPONENTS})
    if (NOT TARGET ${comp})
      set (OpenPose_${comp}_FOUND 0)
      if (OpenPose_FIND_REQUIRED_${comp})
        message(FATAL_ERROR "OpenPose ${comp} not available.")
      endif (OpenPose_FIND_REQUIRED_${comp})
    else (NOT TARGET ${comp})
      set(OpenPose_${comp}_FOUND 1)
      set(OpenPose_LIBS "${comp};${OpenPose_LIBS}")
    endif (NOT TARGET ${comp})
  endforeach ()
else (OpenPose_FIND_COMPONENTS)
  set(OpenPose_LIBS "openpose")
endif (OpenPose_FIND_COMPONENTS)

if (OpenPose_INCLUDE_DIRS AND OpenPose_LIBS)
  set(OpenPose_FOUND 1)
endif (OpenPose_INCLUDE_DIRS AND OpenPose_LIBS)
