find_package(OGRE)

# check whether the modelloader library has been built with lib3ds support
get_target_property(LIB3DS_SUPPORT modelloader HAVE_LIB3DS)

set (BUILD_TESTAPP false)
if (OGRE_FOUND AND LIB3DS_SUPPORT)
    set (BUILD_TESTAPP true)
else (OGRE_FOUND AND LIB3DS_SUPPORT)
    if (NOT OGRE_FOUND)
        message(STATUS "Ogre not found. Cannot compile test application.")
    endif (NOT OGRE_FOUND)
    if (NOT LIB3DS_SUPPORT)
        message(STATUS "modelloader lacks lib3ds support. Cannot compile test application.")
    endif (NOT LIB3DS_SUPPORT)
endif (OGRE_FOUND AND LIB3DS_SUPPORT)


if (BUILD_TESTAPP)
    include_directories(${OGRE_INCLUDE_DIRS})
    link_directories(${OGRE_LIBRARY_DIRS})
    link_directories(${CMAKE_BINARY_DIR}/ode)
    add_definitions(${OGRE_CFLAGS_OTHERS})

#if (FALSE)
    # AB: make sure our local ODE is the _first_ include dir
    #     -> otherwise a path which contains an already installed ODE might be
    #        catched before and cause some harm
    include_directories(BEFORE ${CMAKE_SOURCE_DIR}/ode/include)
    include_directories(BEFORE ${CMAKE_BINARY_DIR}/ode/include)
    set(ODE_LIBRARY myode)
#else (FALSE)
#find_package(ODE)
#include_directories(${ODE_INCLUDE_DIRS})
#endif (FALSE)

    set(testapp_SRCS

        testapp.cpp
        inputhandler.cpp
        inputlistener.cpp
        ogresceneobjectfactory.cpp
        ogrematerials.cpp
        myobjectnode.cpp
        mydeformableobjectnode.cpp
        uiaction.cpp
        gui.cpp
        axisobject.cpp
        framehistory.cpp
        commandlineparser.cpp
        testappcommandline.cpp

        odewrapper.cpp

        scenes/scenebase.cpp
        scenes/physicsscenebase.cpp
        scenes/scenemanager.cpp
        
        scenes/deformable/deforming.cpp
        scenes/deformable/deformablescene.cpp
        scenes/deformable/deformablescenecollisions.cpp
        scenes/deformable/deformablescenespheres.cpp
        scenes/deformable/spherecloth.cpp

        scenes/general/devtest.cpp
        scenes/general/collisionstopscene.cpp
        scenes/general/dcollidescene.cpp
        scenes/general/penetrationdepthscene.cpp
        scenes/general/specificationscene.cpp

        scenes/physics/boxstackscene.cpp
        scenes/physics/clothbox.cpp
        scenes/physics/rotatingcloth.cpp
        scenes/physics/collisionresponse.cpp
        scenes/physics/rampscene.cpp
        scenes/physics/snookerscene.cpp
        scenes/physics/wallscene.cpp
        scenes/physics/shakescene.cpp
        scenes/physics/dominoday.cpp
        scenes/physics/marblerun.cpp
        scenes/physics/hangingclothscene.cpp
        
        scenes/rigid/manymovingboxes.cpp
        scenes/rigid/movingbunny.cpp
        scenes/rigid/tworigidhelicopters.cpp
        scenes/rigid/tworigidhelicoptersingrid.cpp

        scenes/benchmark/mixed.cpp
        scenes/benchmark/rigid.cpp
        scenes/benchmark/deformable.cpp

        modelloader/modelloader3ds.cpp
        modelloader/modelloaderply.cpp
    )

    if (APPLE)
        set (maxosxFlags "MACOSX_BUNDLE")

        set (MACOSX_BUNDLE_BUNDLE_NAME "TestApp")
        set (MACOSX_BUNDLE_INFO_STRING "D-Collide Test Application")
        set (MACOSX_BUNDLE_ICON_FILE "TestApp.icns")
        set (MACOSX_BUNDLE_GUI_IDENTIFIER "d-collide.TestApp")

        # TODO: set bundle properties
        #set (MACOSX_BUNDLE_LONG_VERSION_STRING "")
        #set (MACOSX_BUNDLE_COPYRIGHT "")

        # copy resources into the bundle
        set (BUNDLE_RESOURCES_PATH "${CMAKE_BINARY_DIR}/testapp/TestApp.app/Contents/Resources")
        set (_RESOURCE_SOURCE "${CMAKE_SOURCE_DIR}/testapp/resources")

        # for Mac OS X we need to copy ALL resources into the bundle
        # as we won't install anything to /usr/share (or the like)

        file(MAKE_DIRECTORY "${BUNDLE_RESOURCES_PATH}")

        foreach(_EXT "imageset" "xsd" "scheme" "tga" "JPG" "ply" "3ds" "mesh" "material")
            file(GLOB_RECURSE _RESOURCES RELATIVE "${_RESOURCE_SOURCE}"
                 "${_RESOURCE_SOURCE}/*.${_EXT}"
            )
            set (BUNDLE_RESOURCES "${_RESOURCES}" "${BUNDLE_RESOURCES}")
        endforeach(_EXT)

        foreach (_RESOURCE ${BUNDLE_RESOURCES})
            configure_file("${_RESOURCE_SOURCE}/${_RESOURCE}"
                           "${BUNDLE_RESOURCES_PATH}/${_RESOURCE}" COPYONLY)
        endforeach(_RESOURCE)

        configure_file("${CMAKE_SOURCE_DIR}/testapp/resources/d-collide.icns"
                       "${BUNDLE_RESOURCES_PATH}/TestApp.icns" COPYONLY)
    endif (APPLE)

    if (WIN32)
        set (windowsResourceFile "resources/icon.rc")
    endif (WIN32)

    add_executable(testapp ${maxosxFlags} main.cpp ${testapp_SRCS} ${windowsResourceFile})

    target_link_libraries(testapp
        modelloader
        ${ODE_LIBRARY}
        dcollide-static
        ${OGRE_LIBRARIES})

    install(TARGETS testapp DESTINATION bin)
    install(DIRECTORY resources/cegui-skin resources/models resources/textures
            DESTINATION share/d-collide/resources
            PATTERN ".svn" EXCLUDE)

endif (BUILD_TESTAPP)

set(DCOLLIDE_TESTAPP_INSTALL_PREFIX ${CMAKE_INSTALL_PREFIX})
set(DCOLLIDE_TESTAPP_SOURCE_PREFIX ${CMAKE_CURRENT_SOURCE_DIR})

# AB: note: we create the config file even if ogre was NOT found.
#           this way we can always assume that it exists.

configure_file(${CMAKE_CURRENT_SOURCE_DIR}/dcollide-config_testapp.h.cmake
    ${CMAKE_CURRENT_BINARY_DIR}/dcollide-config_testapp.h)

# make sure dcollide-config_testapp.h will always be found
include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_BINARY_DIR}
)


# vim: et sw=4 ts=4
