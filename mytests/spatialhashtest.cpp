#include <d-collide/world.h>
#include <d-collide/worldcollisions.h>
#include <d-collide/math/vector.h>
#include <d-collide/shapes/box.h>
#include <d-collide/shapes/sphere.h>
#include <d-collide/shapes/mesh.h>
#include <d-collide/collisioninfo.h>
#include <d-collide/debuglog.h>
#include <d-collide/debugstream.h>
#include <d-collide/timing.h>
#include <d-collide/proxy.h>
#include <d-collide/detectordeform/detectordeformmanager.h>
#include <d-collide/detectordeform/spatialhash/spatialhashalgorithm.h>

#include <modelloader/loaderply.h>
#include <modelloader/loaderdummy.h>

#include <iostream>

using namespace dcollide;

enum SpatialHashTestMode {
    MODE_BUNNY = 0,
    MODE_BOX = 1,
    MODE_SPHERE_CLOTH = 2
};

Mesh* createMeshifiedBox(real w, real h, real d) {
    Box box(w, h, d);
    const Mesh* m = box.getMesh();
    return m->cloneMesh();
}

int main() {
    WorldParameters parameters;
    parameters.setWorldDimension(Vector3(1000.0, 1000.0, 1000.0));
    parameters.addPrimaryDeformableAlgorithm(DEFORMABLE_TYPE_SPATIAL_HASH);
    World world(parameters);

    SpatialHashTestMode mode = MODE_SPHERE_CLOTH;

    Proxy* movableProxy = 0;
    Vector3 moveVector;
    int maxRuns = 1;

    switch (mode) {
        default:
            error() << dc_funcinfo << "invalid mode " << mode;
            return 1;
        case MODE_BUNNY:
        {
            const char* bunnyFile = "../testapp/resources/models/bunny.ply";
            //const char* bunnyFile = "../testapp/resources/models/bunny_lower_res2.ply";
            ModelLoader::LoaderPly loader;
            Mesh* bunnyMesh = loader.loadFromFileToOneMesh(bunnyFile);
            if (!bunnyMesh) {
                error() << dc_funcinfo << "ERROR: could not load bunny from " << bunnyFile;
                return 1;
            }
            Proxy* bunnyProxy = world.createProxy(bunnyMesh, (ProxyTypes) (PROXYTYPE_DEFORMABLE | PROXYTYPE_SELFCOLLIDABLE));
            world.addProxy(bunnyProxy);
            debug() << "loaded bunny with " << bunnyMesh->getTriangles().size() << " triangles and " << bunnyMesh->getVertices().size() << " vertices";
            break;
        }
        case MODE_BOX:
        {
            // AB: this is the spatial hash worst case: all triangles have more or less
            // the same edge length => all vertices are mapped to the same cell => a lot
            // of collisions
            Mesh* boxMesh = createMeshifiedBox(10.0, 10.0, 10.0);
            Proxy* boxProxy = world.createProxy(boxMesh, (ProxyTypes) (PROXYTYPE_DEFORMABLE | PROXYTYPE_SELFCOLLIDABLE));
            world.addProxy(boxProxy);
            break;
        }
        case MODE_SPHERE_CLOTH:
        {
            // IDEA:
            // 1. create a cloth (-> "Tuch"), i.e. a 2d surface made of
            //    triangles
            // 2. create a sphere
            //    -> for convenience we use a Mesh, not a Sphere.
            // 3. leth the cloth "fall down" onto the sphere

            float radius = 100.0f;
            Sphere sphere(radius);
            Mesh* sphereMesh = sphere.getMesh()->cloneMesh();

            ModelLoader::LoaderDummy loader;
            float clothSize = 3.0f * radius;
            Mesh* clothMesh = loader.createRectangleSurface(clothSize, clothSize, 2.0, 2.0);
            if (!clothMesh) {
                error() << dc_funcinfo << "could not load mesh";
                return 1;
            }
            Proxy* sphereProxy = world.createProxy(sphereMesh, PROXYTYPE_RIGID);
            Proxy* clothProxy = world.createProxy(clothMesh, PROXYTYPE_DEFORMABLE);

            clothProxy->translate(-clothSize/2.0f, -clothSize/2.0f, -(radius + 1.0f));
            world.addProxy(sphereProxy);
            world.addProxy(clothProxy);

            movableProxy = clothProxy;
            moveVector = Vector3(0.0, 0.0, 1.0);
            maxRuns = 20;

            debug() << "sphere has " << sphereMesh->getVertices().size() << " vertices and " << sphereMesh->getTriangles().size() << " triangles";
            debug() << "cloth has " << clothMesh->getVertices().size() << " vertices and " << clothMesh->getTriangles().size() << " triangles";
            break;
        }
    }

    world.prepareSimulation();

    DetectorDeformManager* manager = world.getDetectorDeformManager();
    if (!manager) {
        error() << dc_funcinfo << "NULL detectordeform manager";
        return 1;
    }
    const std::vector<DetectorDeformAlgorithm*>& algorithms = manager->getAlgorithms();
    SpatialHashAlgorithm* spatialHash = 0;
    for (std::vector<DetectorDeformAlgorithm*>::const_iterator it = algorithms.begin(); it != algorithms.end(); ++it) {
        spatialHash = dynamic_cast<SpatialHashAlgorithm*>(*it);
        if (spatialHash) {
            break;
        }
    }
    if (!spatialHash) {
        error() << dc_funcinfo << "NULL spatialHash";
        return 1;
    }

    debug() << "starting collision detection";
    int run = 1;
    do {
        Timing t;
        try {
            world.calculateAllCollisions();
        } catch (Exception e) {
            e.printBacktrace();
        }
        t.stop();
        debug() << "collision detection (run " << run << ") took " << (unsigned long int)(t.elapsedTime() / 1000) << "ms";

#if 0
        if (spatialHash->getLog()) {
            try {
                spatialHash->getLog()->printMostRecentEntry();
            } catch (Exception e) {
                e.printBacktrace();
            }
        } else {
            debug() << "spatialHash has no log. not printing additional information.";
        }
#endif

        if (movableProxy) {
            movableProxy->translate(moveVector);
        }

        run++;
    } while (movableProxy && run - 1 < maxRuns);


#if 0
    if (spatialHash->getLog()) {
        try {
            spatialHash->getLog()->printSummary();
        } catch (Exception e) {
            e.printBacktrace();
        }
    }
#endif

    return 0;
}

/*
 * vim: et sw=4 ts=4
 */
