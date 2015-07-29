find_package(Lib3ds)

set(modelloader_SRCS
    rply/rply.c
    loaderply.cpp
    loader3ds.cpp
    loaderdummy.cpp
    textureinformation.cpp
)

if (LIB3DS_FOUND)
    set (modelloader_SRCS
        ${modelloader_SRCS}
        loader3dsinternal.cpp
    )
    set(HAVE_LIB3DS 1)
else (LIB3DS_FOUND)
    message(STATUS "lib3ds not found. Won't build .3ds modelloader.")
    set(LIB3DS_LIBRARY "")
    set(LIB3DS_INCLUDE_DIR "")
endif (LIB3DS_FOUND)


configure_file(${CMAKE_CURRENT_SOURCE_DIR}/dcollide-modelloader-config.h.cmake
    ${CMAKE_CURRENT_BINARY_DIR}/dcollide-modelloader-config.h)
include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_BINARY_DIR}
)

include_directories(${LIB3DS_INCLUDE_DIR})
add_library(modelloader ${modelloader_SRCS})
target_link_libraries(modelloader ${LIB3DS_LIBRARY})

if (LIB3DS_FOUND)
    # make sure the testapp can test whether the modelloader library provides
    # .3ds support
    set_target_properties(modelloader PROPERTIES HAVE_LIB3DS TRUE)
endif (LIB3DS_FOUND)
# vim: et sw=4 ts=4
