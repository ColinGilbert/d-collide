project(ode)

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)
include_directories(${CMAKE_CURRENT_BINARY_DIR}/include)
include_directories(${CMAKE_BINARY_DIR})

include(CheckIncludeFile)
include(CheckTypeSize)
check_type_size(char SIZEOF_CHAR)
check_type_size(int SIZEOF_INT)
check_type_size("long int" SIZEOF_LONG_INT)
check_type_size(short SIZEOF_SHORT)
check_type_size(void* SIZEOF_VOIDP)
check_include_file(float.h HAVE_FLOAT_H)
configure_file(${CMAKE_CURRENT_SOURCE_DIR}/config.h.cmake
    ${CMAKE_CURRENT_BINARY_DIR}/include/ode/config.h)

# AB: do NOT use custom buildtype flags.
#     instead CMAKE_C_FLAGS and CMAKE_CXX_FLAGS is used only.
#     --> ODE requires certain flags, we dont want the buildtype to overwrite
#         it.
set(_type ${CMAKE_BUILD_TYPE})
string(TOUPPER "${_type}" _type)
if (NOT WIN32)
    set(CMAKE_C_FLAGS_${_type} "")
    set(CMAKE_CXX_FLAGS_${_type} "")
else (NOT WIN32)
    # AB: this is not so easy as non-win32.
    #     MSVC links against different libraries, depending on the build type
    #     (i.e. win32 provides debug libraries).
    #     this is unfortunately implemented using a C/CXX flag, I believe:
    #     /MD vs /MDd
    #     using debug AND non-debug libraries in one project is an error, so we
    #     must provide this flag when building ODE in the same way as we use it
    #     for the rest of d-collide.
    #
    #     in visual studio the user can select the build type on the fly,
    #     without ever (!) setting CMAKE_BUILD_TYPE, so we must hardcode the
    #     list of available types:
    set(_types "DEBUG;MINSIZEREL;RELEASE;RELWITHDEBINFO") # cmake build types
    set(_types "${_types};PROFILING;TESTING") # additional d-collide types (atm not supported on windows)

    foreach (type ${_types})
        set(_args ${CMAKE_CXX_FLAGS_${type}})
        separate_arguments(_args)

        set(CMAKE_C_FLAGS_${type} "")
        set(CMAKE_CXX_FLAGS_${type} "")

        foreach (arg ${_args})
            # check if ${arg} is either /MD, /MDd, /MTD or /MTd
            if ("${arg}" MATCHES "/MDd?" OR "${arg}" MATCHES "/MTd?")
                set(CMAKE_C_FLAGS_${type} "${arg}")
                set(CMAKE_CXX_FLAGS_${type} "${arg}")
            endif ("${arg}" MATCHES "/MDd?" OR "${arg}" MATCHES "/MTd?")
        endforeach (arg)
    endforeach (type)
endif (NOT WIN32)

# AB: if (COMPILER_IS_GNUCXX) would be more correct, but the ODE Makefile.am
#     adds these flags unconditionally on all systems where autoconf is used, no
#     matter which compiler is used.
#     we may need to adjust these, if some non-gcc compiler does not support
#     these (Intel compiler?)
if (NOT WIN32)
    # AB: the C sources (LIBFAST_SOURCES) use -O1 only, not -O2 !
    set(CMAKE_C_FLAGS "-O1 -fPIC")
    set(CMAKE_CXX_FLAGS "-O2 -fPIC")

    if (NOT CMAKE_BUILD_TYPE_orig OR "${CMAKE_BUILD_TYPE_orig}" STREQUAL "debug")
        set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -g")
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g")
    endif(NOT CMAKE_BUILD_TYPE_orig OR "${CMAKE_BUILD_TYPE_orig}" STREQUAL "debug")
endif (NOT WIN32)

if (MSVC)
    # AB: see ode sources in build/vs2005/ode.vcproj
    # AB: Optimization="2" should be /O2
    # AB: OmitFramePointers="true" should be /Oy
    # AB: StringPooling="true" should be /GF
    # AB: EnableFunctionLevelLinking="true" should be /Gy
    set (CMAKE_C_FLAGS "/DWIN32 /D_CRT_SECURE_NO_DEPRECATE /DODE_LIB /O2 /Oy /GF /Gy")
    set (CMAKE_CXX_FLAGS "/DWIN32 /D_CRT_SECURE_NO_DEPRECATE /DODE_LIB /O2 /Oy /GF /Gy")
endif (MSVC)

set(LIBFAST_SOURCES
    ode/src/fastldlt.c
    ode/src/fastltsolve.c
    ode/src/fastdot.c
    ode/src/fastlsolve.c
)

set(LIBODE_SOURCES
    ode/src/obstack.cpp
    ode/src/collision_util.cpp
    ode/src/array.cpp
    ode/src/ode.cpp
    ode/src/error.cpp
    ode/src/odemath.cpp
    ode/src/collision_kernel.cpp
    ode/src/export-dif.cpp
    ode/src/quickstep.cpp
    ode/src/collision_quadtreespace.cpp
    ode/src/rotation.cpp
    ode/src/collision_space.cpp
    ode/src/collision_cylinder_box.cpp
    ode/src/collision_cylinder_sphere.cpp
    ode/src/collision_cylinder_plane.cpp
    ode/src/sphere.cpp
    ode/src/box.cpp
    ode/src/capsule.cpp
    ode/src/plane.cpp
    ode/src/ray.cpp
    ode/src/cylinder.cpp
    ode/src/convex.cpp
    ode/src/joint.cpp
    ode/src/step.cpp
    ode/src/collision_transform.cpp
    ode/src/lcp.cpp
    ode/src/stepfast.cpp
    ode/src/mass.cpp
    ode/src/testing.cpp
    ode/src/mat.cpp
    ode/src/timer.cpp
    ode/src/matrix.cpp
    ode/src/util.cpp
    ode/src/memory.cpp
    ode/src/misc.cpp
    ode/src/heightfield.cpp
    ode/src/odewrapper.cpp
    ode/src/odewrapper_orig.cpp
)

add_library(myode STATIC ${LIBFAST_SOURCES} ${LIBODE_SOURCES})

# vim: et sw=4 ts=4
