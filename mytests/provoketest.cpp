#include <datatypes/list.h>
#include <timing.h>
#include <world.h>
#include <worldcollisions.h>
#include <proxy.h>
#include <shapes/shapes.h>
#include <debug.h>

#include <iostream>

//#define CALLGRIND 1

using namespace dcollide;

/*******************************************************************************
*                                 World Tests                                  *
*******************************************************************************/

void worldSizeZero(void) {
    std::cout   << "-----------------------------------\n"
                << "worldSizeZero()" << std::endl;

    dcollide::World testWorld(Vector3(0.0, 0.0, 0.0));
    testWorld.prepareSimulation();
    std::cout   << "World: "<< testWorld.getWorldMin()
                <<" to "    << testWorld.getWorldMax()
                << std::endl;
    dcollide::Proxy* biggerBox =
            testWorld.createProxy(new dcollide::Box(
                                        dcollide::Vector3(10, 10, 10))
                               );
    biggerBox->translate(-5, -5, -5);
    testWorld.addProxy(biggerBox);
    std::cout << "adding a Box (-5, -5, -5) to (5, 5, 5)" << std::endl;
    std::cout   << "new World dimensions: "<< testWorld.getWorldMin()
                <<" to "    << testWorld.getWorldMax()
                << std::endl;
}

void worldSizeNegative(void) {
    std::cout   << "-----------------------------------\n"
                << "worldSizeNegative()" << std::endl;
    dcollide::World testWorld(Vector3(-10.0, -10.0, -10.0));
    testWorld.prepareSimulation();
    std::cout   << "World: "<< testWorld.getWorldMin()
                <<" to "    << testWorld.getWorldMax()
                << std::endl;
}

void worldMinGreaterThanMax(void) {
    std::cout   << "-----------------------------------\n"
                << "worldMinGreaterThanMax()" << std::endl;
    dcollide::World testWorld(Vector3(10.0, 10.0, 10.0),
                              Vector3(0.0,   0.0,  0.0));
    testWorld.prepareSimulation();
    std::cout   << "World: "<< testWorld.getWorldMin()
                <<" to "    << testWorld.getWorldMax()
                << std::endl;

    dcollide::Proxy* box =
            testWorld.createProxy(new dcollide::Box(
                                dcollide::Vector3(1, 1, 1))
                               );
    box->translate(5, 5, 5);
    testWorld.addProxy(box);
    std::cout << "adding a Box (5, 5, 5) to (6, 6, 6)" << std::endl;
    std::cout   << "new World dimensions: "<< testWorld.getWorldMin()
                << " to "    << testWorld.getWorldMax()
                << std::endl;
}

void worldMinEqualToMax(void) {
    std::cout   << "-----------------------------------\n"
                << "worldMinEqualToMax()" << std::endl;
    dcollide::World testWorld(Vector3(10.0, 10.0, 10.0),
                              Vector3(10.0, 10.0, 10.0));
    testWorld.prepareSimulation();
    std::cout   << "World: "<< testWorld.getWorldMin()
                <<" to "    << testWorld.getWorldMax()
                << std::endl;
    dcollide::Proxy* box =
            testWorld.createProxy(new dcollide::Box(
                                dcollide::Vector3(20, 20, 20))
                               );
    testWorld.addProxy(box);
    std::cout << "adding a Box (0, 0, 0) to (20, 20, 20)" << std::endl;
    std::cout   << "new World dimensions: "<< testWorld.getWorldMin()
                << " to "    << testWorld.getWorldMax()
                << std::endl;
}

void worldMinMaxNegative(void) {
    std::cout   << "-----------------------------------\n"
                << "worldMinMaxNegative()" << std::endl;
    dcollide::World testWorld(Vector3(-16.0, -16.0, -16.0),
                              Vector3(-2.0, -2.0, -2.0));
    testWorld.prepareSimulation();
    std::cout   << "World: "<< testWorld.getWorldMin()
                <<" to "    << testWorld.getWorldMax()
                << std::endl;
    dcollide::Proxy* outsideBox =
            testWorld.createProxy(new dcollide::Box(
                                    dcollide::Vector3(1, 1, 1))
                               );
    testWorld.addProxy(outsideBox);
    std::cout << "adding a Box (0, 0, 0) to (1, 1, 1)" << std::endl;
    std::cout   << "new World dimensions: "<< testWorld.getWorldMin()
                <<" to "    << testWorld.getWorldMax()
                << std::endl;
}


void worldAddNullProxy(void) {
    dcollide::World testWorld(Vector3(10.0, 10.0, 10.0));
    try {
        testWorld.addProxy(0);
    } catch (NullPointerException e) {
        std::cout   << "caught expected exception in worldAddNullProxy(), "
                    << "proceeding with the test" << std::endl;
    }
}

void worldRemoveProxyNotInTheWorld(void) {
    dcollide::World testWorld(Vector3(10.0, 10.0, 10.0));

    Proxy* testProxy = testWorld.createProxy();
    testWorld.removeProxy(testProxy);
    delete testProxy;
}

void worldSetDimAfterwards(void) {
    dcollide::World testWorld(Vector3(10.0, 10.0, 10.0));

    testWorld.setWorldDimension(Vector3(0.0, 0.0, 0.0));
    testWorld.setWorldDimension(Vector3(-5.0, -5.0, -5.0));
    testWorld.setWorldDimension(Vector3(-1.0, 10.0, 10.0));
}

void RunWorldTests(void) {
    worldSizeZero();
    worldSizeNegative();
    //worldMinGreaterThanMax(); Infinite growing of world size
    worldMinEqualToMax();
    worldMinMaxNegative();
    worldAddNullProxy();
    worldRemoveProxyNotInTheWorld();
    worldSetDimAfterwards();
}

/*******************************************************************************
*                                 Shape Tests                                  *
*                              ---- General ----                               *
*******************************************************************************/
void ShapeSetNullProxy(void) {
    dcollide::Box testBox(Vector3(10.0, 10.0, 10.0));
    testBox.setProxy(0);
}

void RunShapeTests(void) {
    ShapeSetNullProxy();
}


/*******************************************************************************
*                                 Shape Tests                                  *
*                               ---- Boxes ----                                *
*******************************************************************************/

void BoxWithNullMesh(void) {
    dcollide::Box testBox(0);
}

void BoxZeroDimension(void) {
    dcollide::Box firstTestBox(Vector3(0.0, 0.0, 0.0));
    dcollide::Box secondTestBox(0.0, 0.0, 0.0);
}

void BoxNegativeDimension(void) {
    dcollide::Box firstTestBox(Vector3(-5.0, -5.0, -5.0));
    dcollide::Box secondTestBox(Vector3(-1.0, 100.0, 100.0));
    dcollide::Box thirdTestBox(-5.0, -5.0, -5.0);
    dcollide::Box fourthTestBox(-1.0, 100.0, 100.0);

}

void RunBoxTests(void) {
    BoxWithNullMesh();
    BoxZeroDimension();
    BoxNegativeDimension();
}

/*******************************************************************************
*                                 Shape Tests                                  *
*                              ---- Spheres ----                               *
*******************************************************************************/

void SphereZeroDimension(void) {
    dcollide::Sphere firstTestSphere(0.0);
    dcollide::Sphere secondTestSphere(0.0);
}

void SphereNegativeDimension(void) {
    dcollide::Sphere firstTestSphere(-10.0);
    dcollide::Sphere secondTestSphere(-10.0);
}

void SphereNullMeshPointer(void) {
    dcollide::Sphere testSphere(10.0);
    testSphere.setMesh(0);
}

void SphereIllegalMeshPrecision(void) {
    dcollide::Sphere testSphere(10.0);

    testSphere.setAverageEdgeLength(0.0);
    testSphere.setAverageEdgeLength(-10.0);
    testSphere.setAverageEdgeLength(10000000.0);
}

void RunSphereTests(void) {
    SphereZeroDimension();
    SphereNegativeDimension();
    SphereNullMeshPointer();
    SphereIllegalMeshPrecision();
}

/*******************************************************************************
*                                 Shape Tests                                  *
*                               ---- Mesh ----                                 *
*******************************************************************************/

Mesh* getStandardMesh(void) {
    std::vector<Vertex*> vertices;
    Vertex* FirstTestVertex = new Vertex(10.0, 10.0, 10.0);
    Vertex* SecondTestVertex = new Vertex(20.0, 10.0, 10.0);
    Vertex* ThirdTestVertex =  new Vertex(15.0, 20.0, 10.0);

    vertices.push_back(FirstTestVertex);
    vertices.push_back(SecondTestVertex);
    vertices.push_back(ThirdTestVertex);

    std::vector<Triangle*> triangles;
    triangles.push_back(new Triangle(FirstTestVertex, SecondTestVertex, ThirdTestVertex));

    return new Mesh(vertices, triangles);
}

void MeshNullConstructor(void) {
    std::vector<Vertex*> emptyVertexList;
    for (int i = 0; i < 10; i++) {
        emptyVertexList.push_back(0);
    }

    std::vector<Triangle*> emptyTriangleList;
    for (int i = 0; i < 10; i++) {
        emptyTriangleList.push_back(0);
    }

    std::vector<Vector3*> emptyNormalsList;
    for (int i = 0; i < 10; i++) {
        emptyNormalsList.push_back(0);
    }

    std::vector<int> indices;
    for (int i = 0; i < 10; i++) {
        indices.push_back(i);
    }
    try {
        dcollide::Mesh FirstTestMesh(emptyVertexList, emptyTriangleList);
    } catch (dcollide::NullPointerException e) {
        std::cout   << "caught expected exception in MeshNullConstructor(), "
                    << "proceeding with the test" << std::endl;
    }

    try {
        dcollide::Mesh SecondTestMesh(emptyVertexList, indices);
    } catch (dcollide::NullPointerException e) {
        std::cout   << "caught expected exception in MeshNullConstructor(), "
                    << "proceeding with the test" << std::endl;
    }

    try {
        dcollide::Mesh ThirdTestMesh(emptyVertexList, emptyNormalsList, indices);
    } catch (dcollide::MeshTopologyException e) {
        std::cout   << "caught expected exception in MeshNullConstructor(), "
                    << "proceeding with the test" << std::endl;
    }
}

void MeshWrongDeformVectorsCount() {
    std::vector<Vector3> deformVector;
    for (int i = 0; i <= 9; ++i) {
        deformVector.push_back(Vector3(0.0, 0.0, 0.0));
    }
    Mesh* myMesh = getStandardMesh();
    try {
        myMesh->deform(deformVector);
    } catch (dcollide::MeshDeformException e) {
        std::cout << "caught expected exception in MeshDeformVectorsCount(), "
                  << "proceeding with the test" << std::endl;
    }
}

void MeshDeformNullVectors() {
    Mesh* myMesh = getStandardMesh();
    std::vector<Vector3> deformVector;
    for (unsigned int i = 1; i <= myMesh->getVertexCount(); ++i) {
        deformVector.push_back(Vector3(0.0, 0.0, 0.0));
    }
    myMesh->deform(deformVector);
}

void RunMeshTests(void) {
    MeshNullConstructor();
    MeshWrongDeformVectorsCount();
    MeshDeformNullVectors();
}

/*******************************************************************************
*                                 Shape Tests                                  *
*                               ---- Cone ----                                 *
*******************************************************************************/
void ConeIllegalConstructor(void) {
    dcollide::Cone testConeOne(0.0, 10.0);
    dcollide::Cone testConeZwo(10.0, 0.0);
}

void ConeSetNoMesh(void) {
    dcollide::Cone testCone(10.0, 10.0);
    testCone.setMesh(0);
}/*******************************************************************************
*                                 Shape Tests                                  *
*                               ---- Mesh ----                                 *
 *******************************************************************************/

void RunConeTests(void) {
    ConeIllegalConstructor();
    ConeSetNoMesh();
}



int main() {
    std::cout << "Provoking Test: " << std::endl
              << "==============="  << std::endl
              << "This Test tries to crash our library by giving silly values "
              << "to our functions"
              << std::endl << std::endl
              << "Beginning now..."
              << std::endl;
    try {
    RunWorldTests();
    RunShapeTests();
    RunBoxTests();
    RunSphereTests();
    RunMeshTests();
    std::cout << "...ended" << std::endl
              << "If you can see this message without an error message above, "
              << "this means that no error is revealed."
              << std::endl;
    } catch (dcollide::Exception &e) {
        std::cout   << "Unhandled / unexpected Exception reached main()"
                    << "\n" << e.getErrorMessage() << std::endl;
        e.printBacktrace();
    }
}
/*
 * vim:et sw=4 ts=4
 */
