#include "shapes/box.h"
#include "world.h"
#include "math/vector.h"
#include "collisioninfo.h"
#include "detectorrigid/bvhtraverse.h"
#include "debug.h"
#include "bvhnode.h"
#include "collisionpair.h"
#include "narrowphase/narrowphase.h"
#include "proxy.h"

#include <iostream>

dcollide::Proxy* createProxyHierarchy() {
    return 0;
}

dcollide::Proxy* createSimpleProxyHierarchy(dcollide::World* world) {
    const int boxCount = 10;
    std::list<dcollide::Box*> boxes;
    for (int i = 0; i < boxCount; i++) {
        dcollide::Box* box = new dcollide::Box(dcollide::Vector3(10.0f, 20.0f, 30.0f));
        boxes.push_back(box);
    }
    dcollide::Proxy* proxy = world->createProxy();
    float x = 0.0f;
    float y = 0.0f;
    while (boxes.size() > boxCount / 2) {
        dcollide::Shape* shape = boxes.front();
        boxes.pop_front();
        dcollide::Proxy* p = world->createProxy(shape);
        p->translate(dcollide::Vector3(x, y, 0.0f));
        proxy->addChild(p);

        x += 10.0f;
    }

    x = 0.0f;
    y = 20.0f;
    while (boxes.size() > 0) {
        dcollide::Shape* shape = boxes.front();
        boxes.pop_front();
        dcollide::Proxy* p = world->createProxy(shape);
        p->translate(dcollide::Vector3(x, y, 0.0f));
        proxy->addChild(p);
    }
    return proxy;
}

int main() {
    dcollide::World* w1 = new dcollide::World(dcollide::Vector3(2000, 2000, 2000));

    dcollide::Proxy* p1 = createSimpleProxyHierarchy(w1);
    dcollide::Proxy* p2 = createSimpleProxyHierarchy(w1);
    p2->translate(dcollide::Vector3(20.0f, 0.0f, 0.0f));

    w1->addProxy(p1);
    w1->addProxy(p2);

    dcollide::CollisionPair input;
    input.bvol1 = p1->getBvHierarchyNode()->getBoundingVolume();
    input.bvol2 = p2->getBvHierarchyNode()->getBoundingVolume();
    std::cout << "input volume1: " << input.bvol1 << std::endl;
    std::cout << "input volume2: " << input.bvol2 << std::endl;


    dcollide::BvhTraverse detector;
    std::list<dcollide::BoundingVolumeCollision> list;

    std::cout << dc_funcinfo << "calling getBoundingVolumeCollisions()" << std::endl;
    list = detector.getBoundingVolumeCollisions(input);

    std::cout << dc_funcinfo << "retrieved collisions. result: " << list.size() << " collisions" << std::endl;
    for (std::list<dcollide::BoundingVolumeCollision>::iterator it = list.begin(); it != list.end(); ++it) {
        std::cout << "collision: " << (*it).node1 << " " << (*it).node2 << std::endl;
    }

    delete w1;

}

/*
 * vim: et sw=4 ts=4
 */
