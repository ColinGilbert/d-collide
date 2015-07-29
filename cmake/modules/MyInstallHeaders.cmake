# This file defines the macros
# MY_INSTALL_HEADERS
# MY_INSTALL_HEADERS_RECURSIVE
#
# Usage: MY_INSTALL_HEADERS(foobar) to install the directory foobar
#        and all of its headers to include/foobar

macro(MY_INSTALL_HEADERS_RECURSIVE subdir)
    _MY_INSTALL_HEADERS(${subdir} ON)
endmacro(MY_INSTALL_HEADERS_RECURSIVE subdir)

macro(MY_INSTALL_HEADERS subdir)
    _MY_INSTALL_HEADERS(${subdir} OFF)
endmacro(MY_INSTALL_HEADERS subdir)


#internal macro
macro(_MY_INSTALL_HEADERS subdir recursive)
    file(GLOB _files "${subdir}/*.h")
    install(FILES ${_files} DESTINATION include/d-collide/${subdir})

    if (${recursive})
    endif (${recursive})
endmacro(_MY_INSTALL_HEADERS)

# vim: et sw=4 ts=4
