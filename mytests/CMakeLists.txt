add_executable(broadphasetest broadphasetest.cpp)
target_link_libraries(broadphasetest dcollide-static)

add_executable(broadphaseperformancetest broadphaseperformancetest.cpp)
target_link_libraries(broadphaseperformancetest dcollide-static)

add_executable(mymiddlephaserigidtest mymiddlephaserigidtest.cpp)
target_link_libraries(mymiddlephaserigidtest dcollide-static)

add_executable(performancetest performancetest.cpp)
target_link_libraries(performancetest dcollide-static)

add_executable(threadtest threadtest.cpp)
target_link_libraries(threadtest dcollide-static)

add_executable(provoketest provoketest.cpp)
target_link_libraries(provoketest dcollide-static)

add_executable(surfacehierarchytest surfacehierarchytest.cpp)
target_link_libraries(surfacehierarchytest modelloader dcollide-static)

add_executable(spatialhashtest spatialhashtest.cpp)
target_link_libraries(spatialhashtest modelloader dcollide-static)

add_executable(simdtest simdtest.cpp)
target_link_libraries(simdtest modelloader dcollide-static)

add_executable(benchmark benchmark.cpp)
target_link_libraries(benchmark modelloader dcollide-static)

# vim: et sw=4 ts=4
