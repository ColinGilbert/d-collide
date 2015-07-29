if (NOT "${CMAKE_PROJECT_NAME}" STREQUAL "d-collide")
    message(FATAL_ERROR "CMake must be started using the toplevel directory, not the d-collide subdir.")
endif (NOT "${CMAKE_PROJECT_NAME}" STREQUAL "d-collide")

# generate and write a config.h file
configure_file(${CMAKE_CURRENT_SOURCE_DIR}/dcollide-config.h.cmake
    ${CMAKE_CURRENT_BINARY_DIR}/dcollide-config.h)

set(dcollide_SRCS
    debug.cpp

    broadphase/broadphase.cpp
    broadphase/broadphasefactory.cpp
    broadphase/broadphaseproxydata.cpp
    broadphase/broadphasejob.cpp
    broadphase/broadphasecollisions.cpp

    # Broadphase Octree Files
    broadphase/broadphasehierarchicalgrid.cpp
    broadphase/broadphasehierarchicalgridproxydata.cpp
    broadphase/broadphasehierarchicalgridjob.cpp
    broadphase/broadphasehierarchicalgridjob2.cpp
    broadphase/broadphasehierarchicalgridjobcollection.cpp
    broadphase/hierarchicalgrid.cpp

    # BroadPhase Brute Force Files
    broadphase/broadphasebruteforce.cpp
    broadphase/broadphasebruteforcejob.cpp

    math/matrix.cpp
    math/quaternion.cpp
    math/vector.cpp
    math/plane.cpp
    world.cpp
    pipeline.cpp
    worldparameters.cpp
    worldcollisions.cpp
    proxy.cpp
    proxyfactory.cpp
    thread/thread.cpp
    thread/jobpool.cpp
    thread/jobpoolcollection.cpp
    thread/threadpool.cpp
    thread/threadjobcollection.cpp
    debuglog.cpp
    debugstream.cpp
    debugstreamconfiguration_initialize.cpp

    detectordeform/detectordeformmanager.cpp
    detectordeform/detectordeformalgorithm.cpp
    detectordeform/detectordeformjob.cpp
    detectordeform/detectordeformproxydata.cpp
    detectordeform/spatialhash/intvector3.cpp
    detectordeform/spatialhash/simplehash.cpp
    detectordeform/spatialhash/spatialhash.cpp
    detectordeform/spatialhash/spatialgrid.cpp
    detectordeform/spatialhash/spatialhashalgorithm.cpp
    detectordeform/spatialhash/spatialhashjobcollection.cpp
    detectordeform/surfacehierarchy/surfacehierarchy.cpp
    detectordeform/surfacehierarchy/normalcone.cpp
    detectordeform/surfacehierarchy/deformablebvhnode.cpp
    detectordeform/surfacehierarchy/surfacehierarchyalgorithm.cpp
    detectordeform/surfacehierarchy/surfacehierarchyjob.cpp
    detectordeform/surfacehierarchy/surfacehierarchyselfjob.cpp
    detectordeform/surfacehierarchy/surfacehierarchyproxydata.cpp
    detectordeform/bvhierarchy/bvhierarchyalgorithm.cpp
    detectordeform/bvhierarchy/bvhierarchybvhnode.cpp
    detectordeform/bvhierarchy/bvhierarchyjob.cpp
    detectordeform/bvhierarchy/bvhierarchyproxydata.cpp

    bvhnode.cpp
    bvhnodedefault.cpp
    meshsplitter.cpp

    boundingvolumes/aabb.cpp
    boundingvolumes/obb.cpp
    boundingvolumes/boundingvolume.cpp
    boundingvolumes/boundingsphere.cpp
    boundingvolumes/kdop.cpp

    shapes/box.cpp
    shapes/cone.cpp
    shapes/cylinder.cpp
    shapes/mesh.cpp
    shapes/shape.cpp
    shapes/sphere.cpp
    shapes/wedge.cpp
    shapes/mesh/triangle.cpp
    shapes/mesh/vertex.cpp
    shapes/mesh/line.cpp
    shapes/mesh/meshfactory.cpp

    exceptions/exception.cpp


    detectorrigid/bvhtraverse.cpp
    detectorrigid/middlephaserigidjob.cpp
    detectorrigid/middlephaserigidjobcreator.cpp

    narrowphase/narrowphase.cpp
    narrowphase/triangleintersector.cpp
    narrowphase/boxsphereintersector.cpp
    narrowphase/boxboxintersector.cpp
    narrowphase/wedgeintersector.cpp
    narrowphase/intersectionhelpers.cpp
    narrowphase/cylinderintersector.cpp
    narrowphase/coneintersector.cpp
    narrowphase/spheresphereintersector.cpp
    narrowphase/meshmeshintersector.cpp
    narrowphase/narrowphasejob.cpp
    narrowphase/spheretriangleintersector.cpp
    
    collisioncache.cpp
    collisionpair.cpp
)

if (WIN32)
    set (windowsResourceFile "resources/icon.rc")
endif (WIN32)

add_library(dcollide-static STATIC ${dcollide_SRCS})
#add_library(dcollide-shared SHARED ${dcollide_SRCS} ${windowsResourceFile})

target_link_libraries(dcollide-static ${CMAKE_THREAD_LIBS_INIT})
#target_link_libraries(dcollide-shared ${CMAKE_THREAD_LIBS_INIT})

set_target_properties(dcollide-static PROPERTIES OUTPUT_NAME dcollide)
#set_target_properties(dcollide-shared PROPERTIES OUTPUT_NAME dcollide)

set_target_properties(dcollide-static PROPERTIES CLEAN_DIRECT_OUTPUT 1)
#set_target_properties(dcollide-shared PROPERTIES CLEAN_DIRECT_OUTPUT 1)

install(TARGETS dcollide-static
#                dcollide-shared
        RUNTIME DESTINATION bin
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib)


include(MyInstallHeaders)
# AB: note that if you add new headers, you need to at touch CMakeLists.txt,
#     otherwise the new headers won't be installed
# TODO: is there a way to do this recursively?
my_install_headers(.)
my_install_headers(boundingvolumes)
my_install_headers(broadphase)
my_install_headers(datatypes)
my_install_headers(exceptions)
my_install_headers(math)
my_install_headers(narrowphase)
my_install_headers(shapes)
my_install_headers(shapes/mesh)
my_install_headers(thread)
my_install_headers(detectordeform)
my_install_headers(detectordeform/bvhierarchy)
my_install_headers(detectordeform/spatialhash)
my_install_headers(detectordeform/surfacehierarchy)
my_install_headers(detectorrigid)

install(FILES ${CMAKE_CURRENT_BINARY_DIR}/dcollide-config.h DESTINATION include/d-collide/)

# vim: et sw=4 ts=4
