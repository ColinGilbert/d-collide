#include <d-collide/world.h>
#include <d-collide/worldparameters.h>
#include <d-collide/proxy.h>
#include <d-collide/shapes/mesh.h>
#include <d-collide/debugstream.h>

#include <modelloader/loaderdummy.h>
#include <modelloader/loaderply.h>

using namespace dcollide;

int main() {
    WorldParameters parameters;
    parameters.addPrimaryDeformableAlgorithm(DEFORMABLE_TYPE_SURFACE_HIERARCHY);
    parameters.setWorldMinMax(Vector3(-100.0, -100.0, -100.0), Vector3(100.0, 100.0, 100.0));
    World world(parameters);

    ModelLoader::LoaderDummy loaderDummy;
    real width = 50.0;
    real height = 50.0;
    Mesh* clothMesh = loaderDummy.createRectangleSurface(width, height, 1.0, 1.0);
    debug() << "created mesh with " << clothMesh->getTriangles().size() << " triangles";
    Proxy* cloth = world.createProxy(clothMesh, PROXYTYPE_DEFORMABLE);
    cloth->translate((dcollide::real)(-width / 2.0), (dcollide::real)(-height / 2.0), 0.0);
    world.addProxy(cloth);

    ModelLoader::LoaderPly loaderPly;
    Mesh* plyMesh = loaderPly.loadFromFileToOneMesh("dragon_vrip.ply");
    if (plyMesh) {
        debug() << "loaded ply model";
        dcollide::Proxy* ply = world.createProxy(plyMesh, PROXYTYPE_DEFORMABLE);
        world.addProxy(ply);
    } else {
        debug() << "ply model could not be loaded";
    }

    debug() << "preparing simulation...";
    try {
        world.prepareSimulation();
    } catch (Exception e) {
        debug() << e.getErrorMessage();
        e.printBacktrace();
    }
    debug() << "simulation prepared.";


    // final cleanup to make valgrind shut up
    DebugStreamConfiguration::destroyStatic();
}

/*
 * vim: et sw=4 ts=4
 */
