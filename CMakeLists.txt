project(d-collide)

# where to look first for cmake modules, before ${CMAKE_ROOT}/Modules/ is checked
set(CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake/modules")

# AB: INSTALL(DIRECTORY ...) was added in cmake 2.4.4, so we need at least that
#     version.
CMAKE_MINIMUM_REQUIRED(VERSION 2.4.4 FATAL_ERROR)

if (USE_SSE)
    set(DCOLLIDE_USE_SSE ${USE_SSE})
else (USE_SSE)
    set(DCOLLIDE_USE_SSE 0)
endif (USE_SSE)

if (CMAKE_COMPILER_IS_GNUCXX)
    # sse wanted?
    if (DCOLLIDE_USE_SSE EQUAL 2)
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -march=pentium4 -msse2 -mfpmath=sse")
        set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -march=pentium4 -msse2 -mfpmath=sse")
    endif (DCOLLIDE_USE_SSE EQUAL 2)

    # add a "Profiling" build type
    set(CMAKE_C_FLAGS_PROFILING "${CMAKE_C_FLAGS_DEBUG} -pg" CACHE STRING "Flags used by the C compiler during Profiling builds" FORCE)
    set(CMAKE_CXX_FLAGS_PROFILING "${CMAKE_CXX_FLAGS_DEBUG} -pg" CACHE STRING "Flags used by the C++ compiler during Profiling builds" FORCE)
    set(CMAKE_EXE_LINKER_FLAGS_PROFILING "${CMAKE_EXE_LINKER_FLAGS_DEBUG}" CACHE STRING "Flags used by the linker during Profiling builds." FORCE)
    set(CMAKE_SHARED_LINKER_FLAGS_PROFILING "${CMAKE_SHARED_LINKER_FLAGS_DEBUG}" CACHE STRING "Flags used by the linker during Profiling builds" FORCE)
    set(CMAKE_MODULE_LINKER_FLAGS_PROFILING "${CMAKE_MODULE_LINKER_FLAGS_DEBUG}" CACHE STRING "Flags used by the linker during Profiling builds." FORCE)
    mark_as_advanced(
        CMAKE_C_FLAGS_PROFILING
        CMAKE_CXX_FLAGS_PROFILING
        CMAKE_EXE_LINKER_FLAGS_PROFILING
        CMAKE_SHARED_LINKER_FLAGS_PROFILING
        CMAKE_MODULE_LINKER_FLAGS_PROFILING
    )

    # AB: for testing
    set(CMAKE_C_FLAGS_TESTING "${CMAKE_C_FLAGS_DEBUG} -O0" CACHE STRING "Flags used by the C compiler during Testing builds" FORCE)
    set(CMAKE_CXX_FLAGS_TESTING "${CMAKE_CXX_FLAGS_DEBUG} -O0" CACHE STRING "Flags used by the C++ compiler during Testing builds" FORCE)
    set(CMAKE_EXE_LINKER_FLAGS_TESTING "${CMAKE_EXE_LINKER_FLAGS_DEBUG}" CACHE STRING "Flags used by the linker during Testing builds." FORCE)
    set(CMAKE_SHARED_LINKER_FLAGS_TESTING "${CMAKE_SHARED_LINKER_FLAGS_DEBUG}" CACHE STRING "Flags used by the linker during Testing builds" FORCE)
    set(CMAKE_MODULE_LINKER_FLAGS_TESTING "${CMAKE_MODULE_LINKER_FLAGS_DEBUG}" CACHE STRING "Flags used by the linker during Testing builds." FORCE)
    mark_as_advanced(
        CMAKE_C_FLAGS_TESTING
        CMAKE_CXX_FLAGS_TESTING
        CMAKE_EXE_LINKER_FLAGS_TESTING
        CMAKE_SHARED_LINKER_FLAGS_TESTING
        CMAKE_MODULE_LINKER_FLAGS_TESTING
    )

    # AB: "FORCE" is required to update the GUI documentation string.
    #     therefore we set the variable to the current value only (so that the
    #     user settings won't change)
    set(CMAKE_BUILD_TYPE "${CMAKE_BUILD_TYPE}" CACHE STRING "Choose the type of build, options are: Release RelWithDebInfo MinSizeRel Profiling" FORCE)

    if (CMAKE_SYSTEM_NAME MATCHES Linux)
        set ( CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -ansi -Wundef -Wcast-align -Werror-implicit-function-declaration -Wchar-subscripts -Wall -W -Wpointer-arith -Wwrite-strings -Wformat-security -Wmissing-format-attribute")
        set ( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wnon-virtual-dtor -Wno-long-long -ansi -Wundef -Wcast-align -Wchar-subscripts -Wall -W -Wpointer-arith -Wwrite-strings -Wformat-security")

        # AB: -fomit-frame-pointer is enabled by -O, -O2 and some more.
        #     since we don't use these currently, it should be disabled.
        #     however since it makes debugging harder, we disable it explicitly
        #     just in case.
        set ( CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fno-omit-frame-pointer")
        set ( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fno-omit-frame-pointer")


        #set ( CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -pedantic")
        #set ( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pedantic")

        # AB: _GNU_SOURCE turns on some other definitions (see
        # /usr/include/features.h). in particular we need _ISOC99_SOURCE
        # (atm at least for isnan(), possibly for more, too)
        add_definitions(-D_GNU_SOURCE)
        set(CMAKE_REQUIRED_DEFINITIONS -D_GNU_SOURCE)
    endif (CMAKE_SYSTEM_NAME MATCHES Linux)

    if (CMAKE_SYSTEM_NAME MATCHES "Darwin")
        set ( CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -ansi -Wundef -Wcast-align -Werror-implicit-function-declaration -Wchar-subscripts -Wall -W -Wpointer-arith -Wwrite-strings -Wformat-security -Wmissing-format-attribute")
        set ( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wnon-virtual-dtor -Wno-long-long -ansi -Wundef -Wcast-align -Wchar-subscripts -Wall -W -Wpointer-arith -Wwrite-strings -Wformat-security")
        
        set ( CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fno-omit-frame-pointer")
        set ( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fno-omit-frame-pointer")

        add_definitions(-D_GNU_SOURCE)
        set(CMAKE_REQUIRED_DEFINITIONS -D_GNU_SOURCE)
        
        include (${CMAKE_ROOT}/Modules/CMakeFindFrameworks.cmake)
    endif (CMAKE_SYSTEM_NAME MATCHES "Darwin")
endif (CMAKE_COMPILER_IS_GNUCXX)


if (WIN32)
    set(CMAKE_LIBRARY_PATH "${CMAKE_LIBRARY_PATH};$ENV{LIB}")
    
    if (NOT CMAKE_SYSTEM MATCHES "CYGWIN*")
        if (MSVC)
            set ( CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} /Ox /Ot" )
            set ( CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} /Ox /Ot" )
        endif (MSVC)
        
        if (CMAKE_C_COMPILER MATCHES "icl")
            #set ( CMAKE_C_FLAGS "${CMAKE_C_FLAGS} /Wall /w2 /Wcheck" )
            #set ( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /Wall /w2 /Wcheck" )
        endif (CMAKE_C_COMPILER MATCHES "icl")
        
        if (NOT BORLAND)
            set ( CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} /fixed:no /NODEFAULTLIB:LIBCMT /NODEFAULTLIB:LIBC" )
        endif (NOT BORLAND)

        add_definitions(-D_USE_MATH_DEFINES)
        add_definitions(-DNOMINMAX)
        add_definitions(-DWIN32_LEAN_AND_MEAN)
        
        find_library(PTHREAD_LIB_DIR NAMES pthreadVC2 PATHS ENV INCLUDE)
        find_path(PTHREAD_INCLUDE_DIR pthread.h PATHS ENV INCLUDE)
        
        if (PTHREAD_LIB_DIR AND PTHREAD_INCLUDE_DIR)
            set(CMAKE_USE_PTHREADS_INIT 1)
            set(CMAKE_THREAD_LIBS_INIT ${PTHREAD_LIB_DIR})
            include_directories(${PTHREAD_INCLUDE_DIR})
        endif (PTHREAD_LIB_DIR AND PTHREAD_INCLUDE_DIR)
    endif (NOT CMAKE_SYSTEM MATCHES "CYGWIN*")
endif (WIN32)


# default build type
if (NOT CMAKE_CONFIGURATION_TYPES AND NOT CMAKE_BUILD_TYPE)
   set(CMAKE_BUILD_TYPE Debug)
endif (NOT CMAKE_CONFIGURATION_TYPES AND NOT CMAKE_BUILD_TYPE)

set ( CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} -DDCOLLIDE_BUILD_RELEASE_VERSION" )
set ( CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -DDCOLLIDE_BUILD_RELEASE_VERSION" )

option(DCOLLIDE_USE_DOUBLE
    "Build with double precision instead of normal single floating point precision"
    OFF
)
option(DCOLLIDE_BROADPHASE_DEBUG
    "Provide additional debug output for broadphase code"
    OFF
)
option(DCOLLIDE_DISABLE_THREADS
    "Build without multithreading support"
    OFF
)


include(CheckSymbolExists)
include(CheckIncludeFile)
set(_DC_CMAKE_REQUIRED_LIBRARIES "${CMAKE_REQUIRED_LIBRARIES}")
set(CMAKE_REQUIRED_LIBRARIES "-lm")
check_symbol_exists(isnan "math.h" HAVE_ISNAN)
set(CMAKE_REQUIRED_LIBRARIES "${_DC_CMAKE_REQUIRED_LIBRARIES}")
check_include_file(sys/time.h HAVE_SYS_TIME_H)
check_include_file(execinfo.h HAVE_EXECINFO_H)


if (NOT CMAKE_USE_PTHREADS_INIT)
    find_package(Threads)
endif (NOT CMAKE_USE_PTHREADS_INIT)

if (CMAKE_USE_PTHREADS_INIT)
    set(DCOLLIDE_USE_THREADS TRUE)
else (CMAKE_USE_PTHREADS_INIT)
    set(DCOLLIDE_USE_THREADS FALSE)
endif (CMAKE_USE_PTHREADS_INIT)

if (DCOLLIDE_DISABLE_THREADS)
    set(DCOLLIDE_USE_THREADS FALSE)
endif (DCOLLIDE_DISABLE_THREADS)


# make sure dcollide-config.h will always be found
include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_BINARY_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/d-collide
    ${CMAKE_CURRENT_BINARY_DIR}/d-collide
)

add_subdirectory(d-collide)
add_subdirectory(modelloader)
add_subdirectory(tests)
add_subdirectory(ode)
add_subdirectory(testapp)
add_subdirectory(mytests)
add_subdirectory(doc)

# vim: et sw=4 ts=4
