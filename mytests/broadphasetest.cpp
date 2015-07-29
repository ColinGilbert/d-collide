/* Testcases for the whole broadphase
 *
 * one can use ./go.sh to automatically build this either with gcc or intel
 * and execute it with various tools from valgrind. Can even call kcachegrind to
 * measure the cpu-cycles and cpu-time
 * ==================================
 */

// Including necessary headers:
#include "shapes/shapes.h"
#include "world.h"
#include "worldcollisions.h"
#include "collisioninfo.h"
#include "math/vector.h"
#include "proxy.h"

#include "real.h"
#include "timing.h"
#include "broadphase/broadphasehierarchicalgrid.h"
#include "broadphase/broadphasebruteforce.h"
#include "broadphase/broadphasecollisions.h"
#include <iostream>

// This method performs all necessary to retrieve collisions from the world:
void getColl(dcollide::World* w1,dcollide::Proxy* proxy = 0) {

    std::cout << "Getting Collisions ... " << std::endl;
    dcollide::Timing time;

    const dcollide::BroadPhaseCollisions* bvc;

    if (proxy) {
        // Get collisions with proxy
        // and extract broadphase collisions:
        bvc = w1->calculateAllCollisionsWith(proxy).getBroadPhaseCollisions();
    } else {
        // Get all collisions
        // and extract broadphase collisions:
        bvc = w1->calculateAllCollisions().getBroadPhaseCollisions();
    }

    time.stop();
    std::cout << "\tDurance getCollisions: "<<
    time.elapsedTime() << " micros = " << (time.elapsedTime()/1000) << " millis"
    << std::endl;

    std::cout << "\tSet: Number of found collision pairs: " << bvc->getResults().size()
    << std::endl;

    // Don't do this, causes segfault (FIXME) atm in the set:
    //delete bvc;
}


int main() {
    // Flags wether we use Hierarchical Grid or BruteForce method
    bool isO = false;
    bool isBF = false;

    // To measure the durance of actions:
    dcollide::Timing time_global;

    // Creating a new world of dimension 10000,10000,10500:
    dcollide::World* w1 = new
        dcollide::World(dcollide::Vector3(10000.0f,10000.0f,10500.0f));

    // now we start the simulation by calling World::prepareSimulation, this
    // initializes all necessary parts of dcollide.
    w1->prepareSimulation();

    // get the pointer to the broadphase:
    dcollide::BroadPhase* bCheck = w1->getBroadPhase();

    if (!bCheck) {
        std::cerr << dc_funcinfo << "NULL BroadPhase pointer. this test assumes"
            << " World::getBroadPhase() returns eather an object of type "
            << "BroadPhaseHierarchicalGrid or of type "
            << "BroadPhaseBruteForce" << std::endl;
        return 1;
    }



    // if no hierarchical grid broadphase, check if it is a brute force
    // broadphase:
    if (bCheck->getBroadPhaseType() ==
            dcollide::BROADPHASE_TYPE_HIERARCHICALGRID) {
        isO = true;
        std::cout << "Using BroadPhaseHierarchicalGrid!" << std::endl;
    } else if (bCheck->getBroadPhaseType() == dcollide::BROADPHASE_TYPE_BRUTEFORCE) {
        isBF = true;
        std::cout << "Using BroadPhaseBruteForce!" << std::endl;
    } else {
        std::cerr << dc_funcinfo << "NULL BroadPhase pointer. this test assumes"
            << " World::getBroadPhase() returns eather an object of type "
            << "BroadPhaseHierarchicalGrid or of type "
            << "BroadPhaseBruteForce" << std::endl;
        return 1;
    }

    // Now casting the broadphase to the correct type, this is necessary because
    // we later use methods of HierarchicalGrid Broadphase which aren't present
    // in the bruteForce broadphase
    dcollide::BroadPhase* b1;
    if (isBF) {
        b1 = dynamic_cast<dcollide::BroadPhaseBruteForce*>(w1->getBroadPhase());
    }
    else if (isO) {
        b1 = dynamic_cast<dcollide::BroadPhaseHierarchicalGrid*>
                (w1->getBroadPhase());
    }
    std::cout << std::endl << "Worldsize: " <<
        w1->getWorldDimension() << "\n\tWorldMin: " << w1->getWorldMin() <<
         "\n\tWorldMax: " << w1->getWorldMax()
        << std::endl;

    std::cout << "Adding Proxies before simulation ... " << std::endl;
    // If it is BroadPhaseHierarchicalGrid, we set the UpperProbabilityBound for
    // the merging part:
    if (isO) {
        dynamic_cast<dcollide::BroadPhaseHierarchicalGrid*>
                (b1)->setUpperProbabilityBound(0.75f);
    }

    // Now we add our first proxies:
    for (int i = 1; i <= 5; ++i) {
        // Creating Proxy with the shape of a box:
        dcollide::Proxy* proxy2add = w1->createProxy(new dcollide::Box(
            dcollide::Vector3((dcollide::real)10*i,(dcollide::real)10*i,(dcollide::real)10*i)));
        // Now we will create a matrix and translate it:
        dcollide::Matrix m1;
        m1.translate(dcollide::Vector3((dcollide::real)(-10)*i,(dcollide::real)(-10)*i,(dcollide::real)(-10)*i));
        // This matrix now is the state of this proxy:
        proxy2add->setTransformation(m1);

        // Now we can add the proxy to the world:
        w1->addProxy(proxy2add);
    }

    // Adding Proxy with shape Sphere:
    dcollide::Proxy* proxySphereAdd = w1->createProxy(new dcollide::Sphere(
           (dcollide::real)100,(dcollide::real)10));
    dcollide::Matrix mSphere;
    mSphere.translate(dcollide::Vector3((dcollide::real)(-120),(dcollide::real)(-110),(dcollide::real)(-10)));
    proxySphereAdd->setTransformation(mSphere);
    w1->addProxy(proxySphereAdd);
    
    const dcollide::Mesh* sm = proxySphereAdd->getShape()->getMesh();


    // same procedure ...
    for (int i = 1; i <= 5; ++i) {
        dcollide::Proxy* proxy2add = w1->createProxy(new dcollide::Box(
            dcollide::Vector3((dcollide::real)100*i,(dcollide::real)(100)*i,(dcollide::real)100*i)));
        dcollide::Matrix m1;
        m1.translate(dcollide::Vector3((dcollide::real)(-20)*i,(dcollide::real)(-120)*i,(dcollide::real)(-30)*i));
        proxy2add->setTransformation(m1);
        w1->addProxy(proxy2add);
    }
    // same procedure ...
    for (int i = 1; i <= 5; ++i) {
        dcollide::Proxy* proxy2add = w1->createProxy(new dcollide::Box(
            dcollide::Vector3((dcollide::real)20*i,(dcollide::real)(20)*i,(dcollide::real)10*i)));
        dcollide::Matrix m1;
        m1.translate(dcollide::Vector3((dcollide::real)(-10)*i,(dcollide::real)(-120)*i,(dcollide::real)(30)*i));
        proxy2add->setTransformation(m1);
        w1->addProxy(proxy2add);
    }


    // Debug output of the internally used HierarchicalGrid of the broadphase:
    if (isO) {
        std::cout << "MaxGridDepth: " <<
                dynamic_cast<dcollide::BroadPhaseHierarchicalGrid*>
                (b1)->getMaxGridDepth() << std::endl;
        std::cout << "MaxGridMembers: " <<
                dynamic_cast<dcollide::BroadPhaseHierarchicalGrid*>
                (b1)->getMaxGridMembers() << std::endl;
    }

    // Now we want to get all Collisions that occured beetween the added
    // proxies (by calling our method from above):
    getColl(w1);
    std::cout << "\tNumber of Proxies: " << w1->getNumberOfTopLevelProxies()
    << std::endl;

    // again adding some more proxies, now while the simulation is already
    // running::
    std::cout << "Adding Proxies during simulation ... " << std::endl;
//    w1->setWorldDimension(dcollide::Vector3(10000.0f,8000.0f,10500.0f));

    for (int i = 1; i <= 10; ++i) {
        dcollide::Proxy* proxy2add = w1->createProxy(new dcollide::Box(
            dcollide::Vector3((dcollide::real)10*i,(dcollide::real)10*i,(dcollide::real)10*i)));
        dcollide::Matrix m1;
        m1.translate(dcollide::Vector3(40,50,100));
        proxy2add->setTransformation(m1);
        w1->addProxy(proxy2add);
    }

    if (isO) {
        // Debug output of the internally used HierarchicalGrid of the
        // broadphase:
        std::cout << "newMaxGridDepth: " <<
                dynamic_cast<dcollide::BroadPhaseHierarchicalGrid*>
                (b1)->calculateMaxGridDepth() << std::endl;
        std::cout << "newMaxGridMembers: " <<
                dynamic_cast<dcollide::BroadPhaseHierarchicalGrid*>
                (b1)->calculateMaxGridMembers() << std::endl;
        // Calling the BroadPhase::update() method to test it. This should not
        // really be done in a real simulation!
        dynamic_cast<dcollide::BroadPhaseHierarchicalGrid*>(b1)->update();
    }

    getColl(w1);
    std::cout << "\tNumber of Proxies: " << w1->getNumberOfTopLevelProxies()
    << std::endl;


    // same procedure ...
    std::cout << "Adding Proxies during simulation ... " << std::endl;
    dcollide::Proxy* proxy2translate = w1->createProxy(new dcollide::Box(
        dcollide::Vector3((dcollide::real)10,(dcollide::real)10,(dcollide::real)10)));
    dcollide::Matrix m1;
    m1.translate(dcollide::Vector3(40,50,100));
    proxy2translate->setTransformation(m1);
    w1->addProxy(proxy2translate);

    // here we translate a proxy that was already added the world to test if the
    // broadphase is correctly updated in the case:
    std::cout << "Translating Proxies with Shape "
        << proxy2translate->getShape() << " during simulation ... " << std::endl;
    proxy2translate->translate(dcollide::Vector3(50,50,50));
    if (isO) {
        dynamic_cast<dcollide::BroadPhaseHierarchicalGrid*>(b1)->update();
    }
    getColl(w1);
    std::cout << "\tNumber of Proxies: " << w1->getNumberOfTopLevelProxies()
    << std::endl;

    // removing a proxy:
    std::cout << "Removing Proxies during simulation ... " << std::endl;
    w1->removeProxy(proxy2translate);
    delete proxy2translate;
    if (isO) {
        dynamic_cast<dcollide::BroadPhaseHierarchicalGrid*>(b1)->update();
    }
    getColl(w1);
    std::cout << "\tNumber of Proxies: " << w1->getNumberOfTopLevelProxies()
    << std::endl;

    // same procedure ...
    std::cout << "Adding Proxies during simulation ... " << std::endl;
    for (int i = 1; i <= 5; ++i) {
        dcollide::Proxy* proxy2add = w1->createProxy(new dcollide::Box(
            dcollide::Vector3((dcollide::real)30*i,(dcollide::real)20*i,(dcollide::real)5*i)));
        dcollide::Matrix m1;
        m1.translate(dcollide::Vector3((dcollide::real)400*i,(dcollide::real)500*i,(dcollide::real)1000*i));
        proxy2add->setTransformation(m1);
        w1->addProxy(proxy2add);
    }

    dcollide::Proxy* proxy2add = w1->createProxy(new dcollide::Box(
        dcollide::Vector3((dcollide::real)30,(dcollide::real)20,(dcollide::real)5)));
    dcollide::Matrix m123;
    m123.translate(dcollide::Vector3((dcollide::real)400,(dcollide::real)500,(dcollide::real)1000));
    proxy2add->setTransformation(m123);
    w1->addProxy(proxy2add);

    // Now we test if collision detection for one proxy works:
    std::cout << "Checking for collisions for Proxy " << proxy2add << " only" << std::endl;
    getColl(w1,proxy2add);

    // again testing the update-function by changing the UpperProbabilityBound:
    if (isO) {
        dynamic_cast<dcollide::BroadPhaseHierarchicalGrid*>
                (b1)->setUpperProbabilityBound(0.25f);
        std::cout << "newMaxGridDepth: " <<
                dynamic_cast<dcollide::BroadPhaseHierarchicalGrid*>
                (b1)->calculateMaxGridDepth(0.25f) << std::endl;
        std::cout << "newMaxGridMembers: " <<
                dynamic_cast<dcollide::BroadPhaseHierarchicalGrid*>
                (b1)->calculateMaxGridMembers() << std::endl;
        dynamic_cast<dcollide::BroadPhaseHierarchicalGrid*>(b1)->update();

        dynamic_cast<dcollide::BroadPhaseHierarchicalGrid*>
                (b1)->setUpperProbabilityBound(0.75f);
    }
    getColl(w1);
    std::cout << "\tNumber of Proxies: " << w1->getNumberOfTopLevelProxies()
    << std::endl;

    // now we add a proxy which lies outside the world-dimension. This should
    // cause the world to increase, which causes the HierarchicalGrid to be
    // incerased.
    std::cout << "Adding Proxies (10000,120000,30000) during simulation (outside!)... " << std::endl;

    dcollide::Timing time2;
    // outside of the actual grid:
    for (int i = 1; i <= 1; ++i) {
        dcollide::Proxy* proxy2add = w1->createProxy(new dcollide::Box(
            dcollide::Vector3((dcollide::real)20*i,(dcollide::real)(20)*i,(dcollide::real)10*i)));
        dcollide::Matrix m1;
        m1.translate(dcollide::Vector3((dcollide::real)(10000)*i,(dcollide::real)(-120000)*i,(dcollide::real)(30000)*i));
        proxy2add->setTransformation(m1);
        w1->addProxy(proxy2add);
    }
    time2.stop();
    std::cout << "\tDurance adding Proxy: "<<
    time2.elapsedTime() << " micros = " << (time2.elapsedTime()/1000) << " millis"
    << std::endl;

    // output of the new worldsize to check if increasing really happened:
    std::cout << std::endl << "Worldsize: " <<
        w1->getWorldDimension() << "\n\tWorldMin: " << w1->getWorldMin() <<
         "\n\tWorldMax: " << w1->getWorldMax()
        << std::endl;

    if (isO) {
        std::cout << "newMaxGridDepth: " <<
                dynamic_cast<dcollide::BroadPhaseHierarchicalGrid*>
                (b1)->calculateMaxGridDepth(0.75f) <<
std::endl;
        std::cout << "newMaxGridMembers: " <<
                dynamic_cast<dcollide::BroadPhaseHierarchicalGrid*>
                (b1)->calculateMaxGridMembers() << std::endl;
        dynamic_cast<dcollide::BroadPhaseHierarchicalGrid*>(b1)->update();
    }

    getColl(w1);
    std::cout << "\tNumber of Proxies: " << w1->getNumberOfTopLevelProxies()
    << std::endl;

    delete w1;
    time_global.stop();

    std::cout << "\n----------------------------------------------------\n" <<
    "Overall durance: "<< time_global.elapsedTime()
    << " micros = " << (time_global.elapsedTime()/1000) << " millis" <<
    "\n----------------------------------------------------\n" << std::endl;
}

