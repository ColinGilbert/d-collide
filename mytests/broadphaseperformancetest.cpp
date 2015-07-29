#include <world.h>
#include <worldparameters.h>
#include <worldcollisions.h>
#include <proxy.h>
#include <debugstream.h>
#include <debuglog.h>
#include <shapes/shapes.h>

using namespace dcollide;

std::vector<Proxy*> createManyRigidBoxes(World* world, const Vector3& size, int count);

int main() {
    WorldParameters parameters;
    parameters.setWorldMinMax(Vector3(-1000, -1000, -1000), Vector3(50000, 50000, 50000));
    World world(parameters);

    std::vector<Proxy*> boxes;

    int countPerDimension = 10;
    int proxyCount = countPerDimension * countPerDimension * countPerDimension;
    Vector3 boxSize = Vector3(10.0, 10.0, 10.0);
    Vector3 boxOffset = Vector3(10.0, 10.0, 10.0); // distance between two boxes
    boxes = createManyRigidBoxes(&world, boxSize, proxyCount);

    int box = 0;
    Vector3 dist = boxSize + boxOffset;
    for (int x = 0; x < countPerDimension; x++) {
        for (int y = 0; y < countPerDimension; y++) {
            for (int z = 0; z < countPerDimension; z++) {
                Proxy* p = boxes.at(box);

                p->translate(dist.getX() * x,
                        dist.getY() * y,
                        dist.getZ() * z);

                box++;
            }
        }
    }

    world.prepareSimulation();


    std::vector<Vector3> translationPerStep;
    translationPerStep.reserve(proxyCount);

    /*
     * 0 = constant translation for all proxies
     * 1 = disable movement
     */
    int translationMode = 0;
    switch (translationMode) {
        case 0: // constant translation
        {
            Vector3 translation = Vector3(1.0, 1.0, 1.0);
            for (int i = 0; i < proxyCount; i++) {
                translationPerStep.push_back(translation);
            }
            break;
        }
        case 1:
         break;
        default:
            error() << "unknown translationMode " << translationMode;
            return 1;
    }

    /*
     * >= 0: do that many iterations
     *  < 0: do infinite iterations
     */
    int maxIterations = 20;
    bool useThreads = true;

    debug() << "starting collision detection with " << proxyCount << " proxies";
    int reverseMovementCounter = 0;
    for (int iteration = 0; maxIterations < 0 || iteration < maxIterations; iteration++) {
        // move all proxies by one step
        for (int i = 0; i < proxyCount; i++) {
            boxes[i]->translate(translationPerStep[i]);
        }
        reverseMovementCounter++;
        if (reverseMovementCounter > 100) {
            reverseMovementCounter = 0;
            for (std::vector<Vector3>::iterator it = translationPerStep.begin(); it != translationPerStep.end(); ++it) {
                (*it) = (*it) * -1;
            }
        }


        unsigned int flags = World::COLLISIONFLAG_SKIP_MIDDLE_PHASE;
        if (useThreads) {
            flags |= World::COLLISIONFLAG_USE_THREADS;
        }
        world.calculateAllCollisions(flags);

        if (world.getDebugLog()) {
//            world.getDebugLog()->printSummary();
        }
    }

    return 0;
}

std::vector<Proxy*> createManyRigidBoxes(World* world, const Vector3& size, int count) {
    std::vector<Proxy*> boxes;
    if (count <= 0) {
        return boxes;
    }
    boxes.reserve(count);
    for (int i = 0; i < count; i++) {
        Proxy* p = world->createProxy(new Box(size));
        world->addProxy(p);

        boxes.push_back(p);
    }

    return boxes;
}
/*
 * vim: et sw=4 ts=4
 */
