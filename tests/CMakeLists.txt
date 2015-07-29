project(tests)

#find_library(CPPUNIT_LIBRARY NAMES cppunit  PATHS /usr/lib /usr/local/lib)
find_library(CPPUNIT_LIBRARY NAMES cppunit)
find_path(CPPUNIT_INCLUDE_DIR cppunit/TestFixture.h ENV INCLUDE)

if (CPPUNIT_LIBRARY AND CPPUNIT_INCLUDE_DIR)

    # tell cmake where to find d-collide headers
    include_directories(../src ${CPPUNIT_INCLUDE_DIR})

    set(tests_SRCS
        main.cpp
        aabbtest.cpp
        kdoptest.cpp
        obbtest.cpp
        boundingvolumetest.cpp
        boundingvolumehierarchytest.cpp
        narrowphasetest.cpp
        normalstest.cpp
        middlephaserigidtest.cpp
        multimapelementtest.cpp
        proxytest.cpp
        deformable/deformablebvhnodetest.cpp
        deformable/surfacehierarchytest.cpp
        worldtest.cpp
        broadphasecollisionstest.cpp
        broadphasehierarchicalgridtest.cpp
        broadphasebruteforcetest.cpp
        listtest.cpp
        arraytest.cpp
        settest.cpp
        boundingspheretest.cpp
        undotest.cpp
    )

    add_executable(tests ${tests_SRCS})
    target_link_libraries(tests ${CPPUNIT_LIBRARY} ${CMAKE_DL_LIBS})
    target_link_libraries(tests modelloader dcollide-static)


    # AB: note that d-collide uses buildbot internally, thus we do NOT use the CTest
    #     framework (and consequently don't use "add_test()" here)
    add_custom_target(test
            COMMAND ${CMAKE_CURRENT_BINARY_DIR}/tests
            DEPENDS tests
    )
endif (CPPUNIT_LIBRARY AND CPPUNIT_INCLUDE_DIR)

# vim: et sw=4 ts=4
