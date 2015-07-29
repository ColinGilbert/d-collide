# This module locates OGRE, OIS and CEGUI - the latter two are requirements of
# d-collide to use OGRE (OGRE does not strictly depend on them, but we consider
# it an OGRE dependency).
#
# This module defines the following variables:
#  OGRE_FOUND   - true if the required packages were found in the correct
#                 version, otherwise false (don't use OGRE then)
#  OGRE_INCLUDE_DIRS - a list of include directories (to be used in
#                      INCLUDE_DIRS()) that tells cmake where to find the
#                      headers of the required packages.
#  OGRE_LIBRARY_DIRS - a list of library directories (to be used in
#                      LINK_DIRECTORIES()) that tells cmake where to find the
#                      libraries of the required packages
#  OGRE_LIBRARIES    - a list of libraries (to be used in
#                      TARGET_LINK_LIBRARIES()) that includes the libraries of
#                      the required packages.
#  OGRE_CFLAGS       - the cflags required for OGRE
#  OGRE_CFLAGS_OTHERS - the cflags required for OGRE, without the -I flags
#  OGRE_PLUGIN_DIRECTORY - the directory where OGRE plugins should be loaded
#                          from

set(_requiredVersion "OGRE>=1.4.0" "OIS>=0.6.0" "CEGUI>=0.5.0")

find_package(PkgConfig)

set(_orig_pkgconfig_path $ENV{PKG_CONFIG_PATH})
if (WIN32)
    # AB: the OGRE SDK on Win32 sets the OGRE_HOME environment variable
    # AB: note that lib/pkgconfig is NOT delivered by OGRE SDK, but is our own
    #     d-collide extension, to make finding ogre easier
    set(ENV{PKG_CONFIG_PATH} "$ENV{PKG_CONFIG_PATH};$ENV{OGRE_HOME}\\lib\\pkgconfig")
endif (WIN32)
pkg_check_modules(_OGRE ${_requiredVersion})

# If OGRE==1.4.0 is installed only (i.e. < 1.4.1) we need GLX >= 1.3, 1.2 won't
# work.
# Warn about that.
if (_OGRE_FOUND AND _OGRE_OGRE_VERSION STREQUAL "1.4.0")
    message(STATUS "WARNING: OGRE claims to be at version 1.4.0. Make sure you
       have GLX >= 1.3 installed, OGRE 1.4.0 won't work with GLX 1.2. If you
       are sure that you have at least OGRE 1.4.1, you can ignore this warning
       (OGRE 1.4.1 claims to be 1.4.0, too)")
endif (_OGRE_FOUND AND _OGRE_OGRE_VERSION STREQUAL "1.4.0")

if (_OGRE_FOUND AND PKG_CONFIG_EXECUTABLE)
    # OGRE needs absolute paths to its plugins (apparently it can't figure out its
    # own installation prefix).
    # -> so we retrieve the "plugindir" here, so that we can use it later on
    execute_process(
        COMMAND ${PKG_CONFIG_EXECUTABLE} OGRE --variable=plugindir
        RESULT_VARIABLE _failed
        OUTPUT_VARIABLE _output
    )
    if (_failed)
        message(ERROR "Could not use pkg-config to retrieve the plugindir of OGRE. Should never happen, since pkg-config already found OGRE!")
        set(OGRE_FOUND FALSE)
        set(_OGRE_PLUGIN_DIRECTORY "")
    else (_failed)
        # remove the line ending from the output
        string(REGEX REPLACE "[\r\n]" "" _output "${_output}")

        set (_OGRE_PLUGIN_DIRECTORY ${_output})
        set (OGRE_FOUND TRUE)
    endif (_failed)

    if (OGRE_FOUND)
        if (WIN32)
            set (_ogre_gui_renderer OgreGUIRenderer)
        else (WIN32)
            set (_ogre_gui_renderer CEGUIOgreRenderer )
        endif (WIN32)

        set (OGRE_PLUGIN_DIRECTORY ${_OGRE_PLUGIN_DIRECTORY} CACHE STRING "Ogre Plugin directory")
        set(OGRE_LIBRARIES
            ${_ogre_gui_renderer} ${_OGRE_LIBRARIES}
            CACHE STRING "Ogre libraries to link against"
        )
        set(OGRE_INCLUDE_DIRS
            ${_OGRE_INCLUDE_DIRS}
            CACHE STRING "Ogre include directories"
        )
        set(OGRE_LIBRARY_DIRS
            ${_OGRE_LIBRARY_DIRS}
            CACHE STRING "Directories containing Ogre libraries"
        )
        set(OGRE_CFLAGS
            ${_OGRE_CFLAGS}
            CACHE STRING "Ogre CFLAGS"
        )
        set(OGRE_CFLAGS_OTHERS
            ${_OGRE_CFLAGS_OTHER}
            CACHE STRING "Ogre CFLAGS without -I flags"
        )
    endif (OGRE_FOUND)
else (_OGRE_FOUND AND PKG_CONFIG_EXECUTABLE)
    set(OGRE_FOUND FALSE)
endif (_OGRE_FOUND AND PKG_CONFIG_EXECUTABLE)


# DH: Mac OS X specific (find & setup OGRE framework)
if (APPLE)
    set(OIS_INCLUDE_DIR "/Developer/Ogre/Dependencies/include/OIS")
    set(OIS_LIBRARY_DIR "/Developer/Ogre/Dependencies/lib"
                        "/Developer/Ogre/Dependencies/lib/Release")
    
    cmake_find_frameworks(Ogre ${_requiredVersion})
    cmake_find_frameworks(OgreCEGUIRenderer ${_requiredVersion})
    cmake_find_frameworks(CEGUI ${_requiredVersion})
    cmake_find_frameworks(Carbon)
    
    if (Ogre_FRAMEWORKS)
        set(OGRE_FOUND TRUE)

        foreach (dir ${Ogre_FRAMEWORKS})
            set(Ogre_FRAMEWORK_INCLUDES ${Ogre_FRAMEWORK_INCLUDES} ${dir}/Headers)
        endforeach (dir)

        foreach (dir ${CEGUI_FRAMEWORKS})
            set(CEGUI_FRAMEWORK_INCLUDES ${CEGUI_FRAMEWORK_INCLUDES} ${dir}/Headers
                        ${dir}/Headers/elements ${dir}/Headers/falagard)
        endforeach (dir)

        set (OGRE_INCLUDE_DIRS
            ${OGRE_INCLUDE_DIRS}
            ${Ogre_FRAMEWORK_INCLUDES}
            ${CEGUI_FRAMEWORK_INCLUDES}
            ${OIS_INCLUDE_DIR}
        )
        set (OGRE_LIBRARY_DIRS
            ${OGRE_LIBRARY_DIRS}
            ${OIS_LIBRARY_DIR}
            ${CEGUI_LIBRARY_DIRS}
        )
        
        set(OGRE_LIBRARIES ${OGRE_LIBRARIES} ${Carbon_FRAMEWORKS}
                        ${Ogre_FRAMEWORKS} ${OgreCEGUIRenderer_FRAMEWORKS}
                        ${CEGUI_FRAMEWORKS} ois)
    endif (Ogre_FRAMEWORKS)
endif (APPLE)

# resetting PKG_CONFIG_PATH
set(ENV{PKG_CONFIG_PATH} ${_orig_pkgconfig_path})

mark_as_advanced(
    OGRE_INCLUDE_DIRS
    OGRE_LIBRARY_DIRS
    OGRE_LIBRARIES
    OGRE_CFLAGS
    OGRE_CFLAGS_OTHERS
    OGRE_PLUGIN_DIRECTORY
)

# vim: et sw=4 ts=4
