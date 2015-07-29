#include <datatypes/list.h>
#include <timing.h>
#include <world.h>
#include <worldcollisions.h>
#include <proxy.h>
#include <shapes/shapes.h>
#include <debug.h>
#include <debuglog.h>

#include <iostream>

//#define CALLGRIND 1
//#define MANY_IN_ONE 1
#define USE_NARROW_PHASE 1

using namespace dcollide;

void createManyRigidBoxes(World* world, float xOffset = 0.0f, float yOffset = 0.0f) {
#ifdef CALLGRIND
    const float minX = -400.0f;
    const float minY = -400.0f;
#else
#ifdef MANY_IN_ONE
    const float minX = -6000.0f;
    const float minY = -6000.0f;
#else // MANY_IN_ONE
    const float minX = -1000.0f;
    const float minY = -1000.0f;
#endif // MANY_IN_ONE
#endif // CALLGRIND
    const float maxX = -minX;
    const float maxY = -minY;
    for (float x = minX; x <= maxX; x += 30.0f) {
        for (float y = minY; y <= maxY; y += 50.0f) {
            Proxy* box = world->createProxy(new dcollide::Box(dcollide::Vector3(10.0, 10.0, 10.0)));
            dcollide::Vector3 translation = dcollide::Vector3(x + xOffset, y + yOffset, 0.0f);
            box->translate(translation);
            world->addProxy(box);
        }
    }
}



int main() {
    World world(Vector3(20000.0f, 20000.0f, 20000.0f));
    createManyRigidBoxes(&world, 0.0f, 0.0f);
    createManyRigidBoxes(&world, 5.0f, 0.0f);
    createManyRigidBoxes(&world, 8.0f, 0.0f);

    world.prepareSimulation();
    world.setUseCollisionCaching(false);

    std::cout << "have " << world.getTopLevelProxies().size() << " proxies" << std::endl;

#ifdef CALLGRIND
    const int calls = 100;
#else
#ifdef MANY_IN_ONE
    const int calls = 1;
#else
    const int calls = 200;
#endif // MANY_IN_ONE
#endif // CALLGRIND

    long int* broadPhaseTimes = new long int[calls];
    long int* rigidTimes = new long int[calls];
    long int* narrowTimes = new long int[calls];
    long int* totalTimes = new long int[calls];

    Timing timing;

#ifndef USE_NARROW_PHASE
    int phasesFlags = World::COLLISIONFLAG_SKIP_NARROW_PHASE;
#else
    int phasesFlags = 0;
#endif


    int flags = World::COLLISIONFLAG_NONE | phasesFlags;

//    flags |= World::COLLISIONFLAG_USE_THREADS;
    flags |= World::COLLISIONFLAG_USE_THREADS | World::COLLISIONFLAG_DISABLE_PIPELINING;
    long long int rigidTotal = 0;
    long long int broadTotal = 0;
    long long int narrowTotal = 0;
    for (int i = 0; i < calls; i++) {
        WorldCollisions c = world.calculateAllCollisions(flags);
        if (!world.getDebugLog()) {
            std::cerr << dc_funcinfo << "have no debuglog" << std::endl;
            return 1;
        }
        DebugLogEntry* entry = world.getDebugLog()->getMostRecentEntry();
        if (!entry) {
            std::cerr << dc_funcinfo << "have no debuglog entry" << std::endl;
            return 1;
        }
        broadPhaseTimes[i] = entry->getTiming("BroadPhase").elapsedTime();
        rigidTimes[i] = entry->getTiming("MiddlePhase").elapsedTime();
        narrowTimes[i] = entry->getTiming("NarrowPhase").elapsedTime();
        totalTimes[i] = entry->getTiming("calculateAllCollisions()").elapsedTime();

        broadTotal += broadPhaseTimes[i];
        rigidTotal += rigidTimes[i];
        narrowTotal += narrowTimes[i];
    }
    timing.stop();
    std::cout << calls << " calls took: " << timing.elapsedTime() << "us (" << timing.elapsedTime() / 1000 << "ms)" << std::endl;
    std::cout << "broad total: " << broadTotal << "us broad average: " << ((long double)broadTotal) / ((long double)calls) << "us" << std::endl;
    std::cout << "rigid total: " << rigidTotal << "us rigid average: " << ((long double)rigidTotal) / ((long double)calls) << "us" << std::endl;
    std::cout << "narrow total: " << narrowTotal << "us narrow average: " << ((long double)narrowTotal) / ((long double)calls) << "us" << std::endl;
    std::cout << "flags used: " << flags << std::endl;

#if 0
    std::cout << "detailed times:" << std::endl;
    for (int i = 0; i < calls; i++) {
        std::cout << i << ": broad=" << broadPhaseTimes[i] << " rigid=" << rigidTimes[i] << " total=" << totalTimes[i] << std::endl;
    }
#endif
    delete[] broadPhaseTimes;
    delete[] rigidTimes;
    delete[] narrowTimes;
    delete[] totalTimes;
}

/*
 * vim:et sw=4 ts=4
 */
