/*******************************************************************************
 *  Copyright (C) 2007 by the members of PG 510, University of Dortmund:       *
 *              d-collide-users@lists.sourceforge.net                          *
 *     Andreas Beckermann, Christian Bode, Marcel Ens, Sebastian Ens,          *
 *     Martin Fassbach, Maximilian Hegele, Daniel Haus, Oliver Horst,          *
 *     Gregor Jochmann, Timo Loist, Marcel Nienhaus and Marc Schulz            *
 *                                                                             *
 *  All rights reserved.                                                       *
 *                                                                             *
 *  Redistribution and use in source and binary forms, with or without         *
 *  modification, are permitted provided that the following conditions are met:*
 *   - Redistributions of source code must retain the above copyright          *
 *     notice, this list of conditions and the following disclaimer.           *
 *   - Redistributions in binary form must reproduce the above copyright       *
 *     notice, this list of conditions and the following disclaimer in the     *
 *     documentation and/or other materials provided with the distribution.    *
 *   - Neither the name of the PG510 nor the names of its contributors may be  *
 *     used to endorse or promote products derived from this software without  *
 *     specific prior written permission.                                      *
 *                                                                             *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS        *
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT          *
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR      *
 *  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER *
 *  OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,   *
 *  EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,        *
 *  PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR         *
 *  PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF     *
 *  LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING       *
 *  NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS         *
 *  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE                *
 *******************************************************************************/

#include "shapes/mesh/meshfactory.h"
#include "shapes/shapes.h"
#include "datatypes/multimapelement.h"

#include <iostream>
#include <algorithm>

namespace dcollide {

    MeshFactory::MeshFactory() {
    }

    MeshFactory::~MeshFactory() {
    }

    /*!
     * \brief construct a tetraeder asmesh 
     * 
     * Mesh-Tetraeder
     *
            3 \
           / \ \
          /  \  \
         /  --\---2
        0----  \ /
           -----1
     * \param v0 see ascii-art
     * \param v1 see ascii-art
     * \param v2 see ascii-art
     * \param v3 see ascii-art
     * \returns a new Mesh in the shape of a tetraeder
     *
     * OWNERSHIP NOTICE: The caller takes ownership of the returned object
     */
    Mesh* MeshFactory::createTetraeder(Vector3 p0, Vector3 p1, Vector3 p2,
            Vector3 p3) {
        using namespace dcollide;
        Vertex* v0 = new Vertex(p0);
        Vertex* v1 = new Vertex(p1);
        Vertex* v2 = new Vertex(p2);
        Vertex* v3 = new Vertex(p3);

        Triangle* t0 = new Triangle(v0, v2, v1);
        Triangle* t1 = new Triangle(v0, v1, v3);
        Triangle* t2 = new Triangle(v1, v2, v3);
        Triangle* t3 = new Triangle(v2, v0, v3);

        std::vector<Vertex*> vertices;
        vertices.push_back(v0);
        vertices.push_back(v1);
        vertices.push_back(v2);
        vertices.push_back(v3);

        std::vector<Triangle*> triangles;
        triangles.push_back(t0);
        triangles.push_back(t1);
        triangles.push_back(t2);
        triangles.push_back(t3);

        return new Mesh(vertices, triangles);
    }

    /*!
     * \brief Generates an Mesh representation/approximation for an sphere.
     *
     * To approximate the hull of an sphere an icosahedron is
     * created and subdivided acording to \p averageEdgeLength if \p
     * averageEdgeLength is given
     *
     * The figure below shows where the midpoint of the generated mesh lies:
     *   _____
     *  /     \
     * |   +   |
     *  \_____/
     *
     * \param radius the radius of the sphere
     * \param averageEdgeLength if given, the edges are subdivided until
     * \p averageEdgeLength is reached
     * \returns The mesh representation of a sphere with radius \p radius and
     * optionally an average edge length of \p averageEdgeLength
     * 
     * OWNERSHIP NOTICE: The caller takes ownership of the returned object
     */
    Mesh* MeshFactory::createSphere(real radius, real averageEdgeLength) {
        std::vector<Vertex*> vertices;
        std::vector<Triangle*> triangles;

        std::set<Triangle*> redundantTriangles;
        std::set<MultiMapElement<Vertex, Vertex*> > combinations;


        /* calculate golden rectangle side lenghts of the icosahedron
         * 
         * 3.804226065 = sqrt(10 + 2 * sqrt(5))
         * 1.618033989 = (1 + sqrt(5)) / 2
         */
        real a = (real)((4 * radius) / 3.804226065);
        real b = (real)(a * 1.618033989);

        /* generate vertices acording to the golden rectangles of the
         * icosahedron
         */
        vertices.push_back(new Vertex(-a/2,  b/2,    0));
        vertices.push_back(new Vertex( a/2,  b/2,    0));
        vertices.push_back(new Vertex(-a/2, -b/2,    0));
        vertices.push_back(new Vertex( a/2, -b/2,    0));

        vertices.push_back(new Vertex(-b/2,    0,  a/2));
        vertices.push_back(new Vertex(-b/2,    0, -a/2));
        vertices.push_back(new Vertex( b/2,    0,  a/2));
        vertices.push_back(new Vertex( b/2,    0, -a/2));

        vertices.push_back(new Vertex(   0,  a/2, -b/2));
        vertices.push_back(new Vertex(   0, -a/2, -b/2));
        vertices.push_back(new Vertex(   0,  a/2,  b/2));
        vertices.push_back(new Vertex(   0, -a/2,  b/2));


        /* generate first rough sphere approximation through an icosahedron
         * (connect the vertices to triangles and 'generate' normals)
         */
        triangles.push_back(
            new Triangle(
                vertices.at(1), // new Vector3(vertices.at(1)->getPosition()),
                vertices.at(0),  //new Vector3(vertices.at(0)->getPosition()),
                vertices.at(10)//, new Vector3(vertices.at(10)->getPosition())
            )
        );

        triangles.push_back(
            new Triangle(
                vertices.at(8),  //new Vector3(vertices.at(8)->getPosition()),
                vertices.at(0),  //new Vector3(vertices.at(0)->getPosition()),
                vertices.at(1)//,  new Vector3(vertices.at(1)->getPosition())
            )
        );

        triangles.push_back(
            new Triangle(
                vertices.at(2),  //new Vector3(vertices.at(2)->getPosition()),
                vertices.at(3),  //new Vector3(vertices.at(3)->getPosition()),
                vertices.at(11)//, new Vector3(vertices.at(11)->getPosition())
            )
        );

        triangles.push_back(
            new Triangle(
                vertices.at(2),  //new Vector3(vertices.at(2)->getPosition()),
                vertices.at(9),  //new Vector3(vertices.at(9)->getPosition()),
                vertices.at(3)//,  new Vector3(vertices.at(3)->getPosition())
            )
        );

        triangles.push_back(
            new Triangle(
                vertices.at(5),  //new Vector3(vertices.at(5)->getPosition()),
                vertices.at(4),  //new Vector3(vertices.at(4)->getPosition()),
                vertices.at(0)//,  new Vector3(vertices.at(0)->getPosition())
            )
        );

        triangles.push_back(
            new Triangle(
                vertices.at(2),  //new Vector3(vertices.at(2)->getPosition()),
                vertices.at(4),  //new Vector3(vertices.at(4)->getPosition()),
                vertices.at(5)//,  new Vector3(vertices.at(5)->getPosition())
            )
        );

        triangles.push_back(
            new Triangle(
                vertices.at(7),  //new Vector3(vertices.at(7)->getPosition()),
                vertices.at(1),  //new Vector3(vertices.at(1)->getPosition()),
                vertices.at(6)//,  new Vector3(vertices.at(6)->getPosition())
            )
        );

        triangles.push_back(
            new Triangle(
                vertices.at(3),  //new Vector3(vertices.at(3)->getPosition()),
                vertices.at(7),  //new Vector3(vertices.at(7)->getPosition()),
                vertices.at(6)//,  new Vector3(vertices.at(6)->getPosition())
            )
        );

        triangles.push_back(
            new Triangle(
                vertices.at(4),  //new Vector3(vertices.at(4)->getPosition()),
                vertices.at(11), //new Vector3(vertices.at(11)->getPosition()),
                vertices.at(10)//, new Vector3(vertices.at(10)->getPosition())
            )
        );

        triangles.push_back(
            new Triangle(
                vertices.at(11), //new Vector3(vertices.at(11)->getPosition()),
                vertices.at(6),  //new Vector3(vertices.at(6)->getPosition()),
                vertices.at(10)//, new Vector3(vertices.at(10)->getPosition())
            )
        );

        triangles.push_back(
            new Triangle(
                vertices.at(5),  //new Vector3(vertices.at(5)->getPosition()),
                vertices.at(8),  //new Vector3(vertices.at(8)->getPosition()),
                vertices.at(9)//,  new Vector3(vertices.at(9)->getPosition())
            )
        );

        triangles.push_back(
            new Triangle(
                vertices.at(9),  //new Vector3(vertices.at(9)->getPosition()),
                vertices.at(8),  //new Vector3(vertices.at(8)->getPosition()),
                vertices.at(7)//,  new Vector3(vertices.at(7)->getPosition())
            )
        );

        triangles.push_back(
            new Triangle(
                vertices.at(0),  //new Vector3(vertices.at(0)->getPosition()),
                vertices.at(4),  //new Vector3(vertices.at(4)->getPosition()),
                vertices.at(10)//, new Vector3(vertices.at(10)->getPosition())
            )
        );

        triangles.push_back(
            new Triangle(
                vertices.at(10), //new Vector3(vertices.at(10)->getPosition()),
                vertices.at(6),  //new Vector3(vertices.at(6)->getPosition()),
                vertices.at(1)//,  new Vector3(vertices.at(1)->getPosition())
            )
        );

        triangles.push_back(
            new Triangle(
                vertices.at(8),  //new Vector3(vertices.at(8)->getPosition()),
                vertices.at(1),  //new Vector3(vertices.at(1)->getPosition()),
                vertices.at(7)//,  new Vector3(vertices.at(7)->getPosition())
            )
        );

        triangles.push_back(
            new Triangle(
                vertices.at(5), //new Vector3(vertices.at(5)->getPosition()),
                vertices.at(0), //new Vector3(vertices.at(0)->getPosition()),
                vertices.at(8)//, new Vector3(vertices.at(8)->getPosition())
            )
        );

        triangles.push_back(
            new Triangle(
                vertices.at(2),  //new Vector3(vertices.at(2)->getPosition()),
                vertices.at(11), //new Vector3(vertices.at(11)->getPosition()),
                vertices.at(4)//,  new Vector3(vertices.at(4)->getPosition())
            )
        );

        triangles.push_back(
            new Triangle(
                vertices.at(3),  //new Vector3(vertices.at(3)->getPosition()),
                vertices.at(6),  //new Vector3(vertices.at(6)->getPosition()),
                vertices.at(11)//, new Vector3(vertices.at(11)->getPosition())
            )
        );

        triangles.push_back(
            new Triangle(
                vertices.at(9),  //new Vector3(vertices.at(9)->getPosition()),
                vertices.at(7),  //new Vector3(vertices.at(7)->getPosition()),
                vertices.at(3)//,  new Vector3(vertices.at(3)->getPosition())
            )
        );

        triangles.push_back(
            new Triangle(
                vertices.at(2),  //new Vector3(vertices.at(2)->getPosition()),
                vertices.at(5),  //new Vector3(vertices.at(5)->getPosition()),
                vertices.at(9)//,  new Vector3(vertices.at(9)->getPosition())
            )
        );


        /* subdivide each of the triangles in the icosahedron */
        for (int i = 0; i < 20; i++) {

            /* subdivide triangle */
            subdivide(
                vertices,
                triangles,
                redundantTriangles,
                combinations,
                triangles.at(i),
                radius,
                averageEdgeLength
            );

            /* mark subdivided triangle as obsolete */
            redundantTriangles.insert(triangles.at(i));
        }


        /* remove all triangles that are marked as redundant */
        std::sort(triangles.begin(), triangles.end());

        std::set<Triangle*>::iterator
            redundant_it = redundantTriangles.begin();
        std::vector<Triangle*>::iterator
            triangles_it = triangles.begin();

        while(   (triangles_it != triangles.end())
              && (redundant_it != redundantTriangles.end()) ) {

            if ((*triangles_it) == (*redundant_it)) {
                delete *triangles_it;

                  triangles_it = triangles.erase(triangles_it);
                ++redundant_it;
            } else {
                ++triangles_it;
            }
        }

        /* clear the Set, it isn't needed anymore */
        redundantTriangles.clear();

        /* create and return the resulting mesh */
        return new Mesh(vertices, triangles);
    }

    /*!
     * \brief create a mesh represantation of the given \p sphere
     * \param sphere
     *
     * OWNERSHIP NOTICE: The caller takes ownership of the returned object
     */
    Mesh* MeshFactory::createSphere(const Sphere& sphere) {
        return createSphere(sphere.getRadius(),sphere.getAverageEdgeLength());
    }

    /*!
     * \brief Generates a simple box mesh out of twelve triangles according to
     *        \p dimension.
     * 
     * The mesh for this box is created according to the dimension given in
     * \p dimension . Each box mesh is composed out of twelve triangles, each
     * side of the four sides of the box is dived into two trinagles.
     * 
     * The figure below shows the assignment of box vertices to indices of the
     * \p vertices vector:
     *   0------1
     *  /|     /|
     * 4------5 |
     * | |    | |
     * | 2------3
     * |/     |/
     * 6------7
     *
     * The vertices in the triangles are specified in anti-clockwise order.
     * \param dimension the dimension of the box
     *
     * OWNERSHIP NOTICE: The caller takes ownership of the returned object
     */
    Mesh* MeshFactory::createBox(Vector3 dimension) {

        std::vector<Vertex*> vertices = std::vector<Vertex*>(8);
        std::vector<Triangle*> triangles = std::vector<Triangle*>(12);


        vertices[0] = new Vertex(Vector3(0, dimension.getY(), 0));

        vertices[1] = new Vertex(Vector3(dimension.getX(),
                                         dimension.getY(),
                                         0
                                        ));

        vertices[2] = new Vertex(Vector3(0, 0, 0));

        vertices[3] = new Vertex(Vector3(dimension.getX(), 0, 0));

        vertices[4] = new Vertex(Vector3(0,
                                         dimension.getY(),
                                         dimension.getZ()
                                        ));

        vertices[5] = new Vertex(Vector3(dimension.getX(),
                                         dimension.getY(),
                                         dimension.getZ()
                                        ));

        vertices[6] = new Vertex(Vector3(0, 0, dimension.getZ()));

        vertices[7] = new Vertex(Vector3(dimension.getX(),
                                         0,
                                         dimension.getZ()
                                        ));



        // TODO: we can precalculate normals here!
        triangles[0]  = new Triangle( vertices[6], vertices[5], vertices[4] );
        triangles[1]  = new Triangle( vertices[5], vertices[6], vertices[7] );

        triangles[2]  = new Triangle( vertices[3], vertices[1], vertices[5] );
        triangles[3]  = new Triangle( vertices[3], vertices[5], vertices[7] );

        triangles[4]  = new Triangle( vertices[2], vertices[0], vertices[1] );
        triangles[5]  = new Triangle( vertices[2], vertices[1], vertices[3] );

        triangles[6]  = new Triangle( vertices[0], vertices[2], vertices[4] );
        triangles[7]  = new Triangle( vertices[2], vertices[6], vertices[4] );

        triangles[8]  = new Triangle( vertices[0], vertices[4], vertices[1] );
        triangles[9]  = new Triangle( vertices[1], vertices[4], vertices[5] );

        triangles[10] = new Triangle( vertices[2], vertices[3], vertices[6] );
        triangles[11] = new Triangle( vertices[3], vertices[7], vertices[6] );


        return new Mesh(vertices, triangles);
    }

    /*!
     * \brief create a mesh represantation of the given \p box
     * \param box
     *
     * OWNERSHIP NOTICE: The caller takes ownership of the returned object
     */
    Mesh* MeshFactory::createBox(const Box& box) {
        return createBox(box.getDimension());
    }


    /*!
     * \brief Generates a simple wedge mesh out of eight triangles according to
     *        \p dimensions.
     *
     * The figure below shows the assignment of wedge vertices to indices of the
     * \p vertices vector:
     *
     *         5------4
     *  z     /|     /|
     *  ^    / | *  / | z
     *  |   /  |*  /  |
     *  |  /   3--/---2
     *  | /  *   /  *
     *  |/ *    / *  y
     *  o------1---------------->
     *     x
     *
     * The vertices in the triangles are specified in anti-clockwise order.
     * 
     * \param dimension the dimension of the wedge
     *
     * OWNERSHIP NOTICE: The caller takes ownership of the returned object
     */
    Mesh* MeshFactory::createWedge(const real* dimension) {

        std::vector<Vertex*> vertices = std::vector<Vertex*>(6);
        std::vector<Triangle*> triangles = std::vector<Triangle*>(8);

        vertices[0] = new Vertex(Vector3(0, 0, 0));

        vertices[1] = new Vertex(Vector3(dimension[0], 0, 0));

        vertices[2] = new Vertex(Vector3(dimension[0], dimension[1], 0));

        vertices[3] = new Vertex(Vector3(0, dimension[1], 0));

        vertices[4] = new Vertex(Vector3(dimension[0], dimension[1], dimension[2]));

        vertices[5] = new Vertex(Vector3(0, dimension[1], dimension[2]));

        // TODO: we can precalculate normals here!
        //base rectangle
        triangles[0]  = new Triangle( vertices[2], vertices[1], vertices[0] );
        triangles[1]  = new Triangle( vertices[3], vertices[2], vertices[0] );

        //"ramp"
        triangles[2]  = new Triangle( vertices[0], vertices[1], vertices[4] );
        triangles[3]  = new Triangle( vertices[0], vertices[4], vertices[5] );

        //sides
        triangles[4]  = new Triangle( vertices[1], vertices[2], vertices[4] );
        triangles[5]  = new Triangle( vertices[5], vertices[3], vertices[0] );

        //back rectangle
        triangles[6]  = new Triangle( vertices[4], vertices[2], vertices[3] );
        triangles[7]  = new Triangle( vertices[5], vertices[4], vertices[3] );

        return new Mesh(vertices, triangles);
    }

    /*!
     * \brief create a mesh represantation of the given \p wedge
     * \param wedge
     *
     * OWNERSHIP NOTICE: The caller takes ownership of the returned object
     */
    Mesh* MeshFactory::createWedge(const Wedge& wedge) {
        return createWedge(wedge.getDimensions());
    }

    /*!
     * \brief generates a mesh representation of a cylinder
     *
     * The mesh for an cone is generated in the way, that first the base point
     * and the mid point of the top circle is generated.
     * Then the top circle is generated and each point on the circle is
     * connected with the mid point of the circle and with the base point of
     * the cone.   
     *
     * \param radius The radius of the cylinder
     * \param height the height of the cylinder
     * \param averageEdgeLength The averageEdgeLength of the approximated
     * circles on top and bottom
     *
     * OWNERSHIP NOTICE: The caller takes ownership of the returned object
     */
    Mesh* MeshFactory::createCone(real radius,real height,real
            averageEdgeLength) {

        const int MIN_SECTIONS = 8;

        int circleSegments = 0;
        real sectionAngle, localAngle, x, y, z;

        std::vector<Vertex*> vertices;
        std::vector<Triangle*> triangles;


        if (averageEdgeLength) {
            /* calculate the number of sections out of the choosen precision 
             */
            real sections;

            sections  = (real) (2 * M_PI * radius);
            sections /= averageEdgeLength;

            circleSegments = (int) ceil(sections);

            /* when lower bound is violated or lessest precision is choosen,
             * use the minimum number of sections.
             */
            if (circleSegments < MIN_SECTIONS) {
                circleSegments = MIN_SECTIONS;
            }

        } else {
            circleSegments = MIN_SECTIONS;
        }

        sectionAngle = (real)(2 * M_PI / circleSegments);


        Vertex* current;

        Vertex* basePoint = new Vertex(0, 0, 0); //the base point
        Vertex* tipPoint = new Vertex(0, 0, height); //the tip

        Vertex* firstPoint = new Vertex(radius, 0, 0);
        Vertex* lastPoint = firstPoint;

        vertices.push_back(basePoint);
        vertices.push_back(tipPoint);
        vertices.push_back(firstPoint);

        for (int segment = 1; segment < circleSegments; segment++) {

            localAngle = (real)(segment * sectionAngle);

            x = cos(localAngle) * radius;
            y = sin(localAngle) * radius;
            z = 0;

            current = new Vertex(x, y, z);

            vertices.push_back(current);

            triangles.push_back(new Triangle(lastPoint, basePoint, current));

            triangles.push_back(new Triangle(tipPoint, lastPoint, current));

            lastPoint = current;
        } //end for 


        triangles.push_back(new Triangle(firstPoint, lastPoint, basePoint));
        triangles.push_back(new Triangle(firstPoint, tipPoint, lastPoint));

        return new Mesh(vertices, triangles);
    }

    /*!
     * \brief create a mesh represantation of the given \p cone
     * \param cone
     *
     * OWNERSHIP NOTICE: The caller takes ownership of the returned object
     */
    Mesh* MeshFactory::createCone(const Cone& cone) {
        return createCone(cone.getRadius(),cone.getHeight(),cone.getAverageEdgeLength());
    }

    /*!
     * \brief Generates an Mesh representation/approximation for an cylinder.
     *
     * This method generates a mesh representation/approximation for this
     * cylinder shape. The orientation of the created mesh is shown in the
     * figure below, with a "x" marking the origin:
     * 
     * The cylinder rotationalsymmetric axis of the cylinder is its Z-axis
     *  
     *             ^ z
     *             |
     *          r  |
     *       |<--->|
     *       |     |
     *        _..-----.._
     *  ---- (_         _)
     *   ^   | ''-----'' |
     *   |   |           |
     * h |   |           |
     *   |   |           |
     *   V   |_..-----.._|
     *  ---- (_    x    _) ----------> y
     *         ''-----''
     *         /
     *        /
     *       /|
     *      x
     *
     * The mesh is created out of two circle approximations at the top and the
     * bottom which are simply connected through triangles.
     *
     * \param radius The radius of the cylinder
     * \param height the height of the cylinder
     * \param averageEdgeLength The averageEdgeLength of the approximated
     * circles on top and bottom
     *
     * OWNERSHIP NOTICE: The caller takes ownership of the returned object
     */
    Mesh* MeshFactory::createCylinder(real radius, real height,real averageEdgeLength) {
        static const int MIN_SECTIONS = 8;

        int circleSegments;
        real sections;

        std::vector<Vertex*> vertices;
        std::vector<Triangle*> triangles;


        if (averageEdgeLength > 0) {
            /* calculate the number of sections out of the choosen precision
             */
            sections  = (real) (2.0 * M_PI * radius);
            sections /= averageEdgeLength;

            circleSegments = (int) ceil(sections);

            /* when lower bound is violated or lessest precision is choosen,
             * use the minimum number of sections.
             */
            if (circleSegments < MIN_SECTIONS) {
                circleSegments = MIN_SECTIONS;
            }

        } else {
            circleSegments = MIN_SECTIONS;
            // FIXME: sections is not initialized
        }

        const real angle = (real) (2.0 * M_PI / circleSegments);

        // center vertices (top and bottom)
        Vertex* baseVertex = new Vertex(0, 0, 0);
        Vertex* topVertex = new Vertex(0, 0, height);
        vertices.push_back(baseVertex);
        vertices.push_back(topVertex);

        // first (and last) vertices on circumference
        Vertex* initialVertexBottom = new Vertex(radius, 0, 0);
        Vertex* initialVertexTop = new Vertex(radius, 0, height);
        vertices.push_back(initialVertexBottom);
        vertices.push_back(initialVertexTop);

        //pointers to the vertices created in the prior step
        Vertex* previousVertexBottom = initialVertexBottom;
        Vertex* previousVertexTop = initialVertexTop;
        for (int i=1; i < circleSegments; i++) {
            real x1 = cos(i*angle) * radius; // TODO: precalculate
            real y1 = sin(i*angle) * radius; //       sin/cos-tables

            Vertex* nextVertexBottom = new Vertex(x1, y1, 0);
            Vertex* nextVertexTop = new Vertex(x1, y1, height);
            vertices.push_back(nextVertexBottom);
            vertices.push_back(nextVertexTop);

            // draw sections
            triangles.push_back(new Triangle(baseVertex, nextVertexBottom, previousVertexBottom));
            triangles.push_back(new Triangle(topVertex, previousVertexTop, nextVertexTop));

            // and fill the sides
            triangles.push_back(
                new Triangle(
                    nextVertexTop, new Vector3( nextVertexTop->getPosition() - topVertex->getPosition() ),
                    previousVertexTop, new Vector3( previousVertexTop->getPosition() - topVertex->getPosition() ),
                    previousVertexBottom, new Vector3( previousVertexBottom->getPosition() - baseVertex->getPosition() )
                )
            );
            triangles.push_back(
                new Triangle(
                    nextVertexBottom, new Vector3( nextVertexBottom->getPosition() - baseVertex->getPosition() ),
                    nextVertexTop, new Vector3( nextVertexTop->getPosition() - topVertex->getPosition() ),
                    previousVertexBottom, new Vector3( previousVertexBottom->getPosition() - baseVertex->getPosition() )
                )
            );

            previousVertexBottom = nextVertexBottom;
            previousVertexTop = nextVertexTop;
        }

        // finally close the circles
        triangles.push_back(new Triangle(baseVertex, initialVertexBottom, previousVertexBottom));
        triangles.push_back(new Triangle(topVertex, previousVertexTop, initialVertexTop));

        // and close the sides
        triangles.push_back(
            new Triangle(
                initialVertexTop, new Vector3( initialVertexTop->getPosition() - topVertex->getPosition() ),
                previousVertexTop, new Vector3( previousVertexTop->getPosition() - topVertex->getPosition() ),
                previousVertexBottom, new Vector3( previousVertexBottom->getPosition() - baseVertex->getPosition() )
            )
        );
        triangles.push_back(
            new Triangle(
                initialVertexTop, new Vector3( initialVertexTop->getPosition() - topVertex->getPosition() ),
                previousVertexBottom, new Vector3( previousVertexBottom->getPosition() - baseVertex->getPosition() ),
                initialVertexBottom, new Vector3( initialVertexBottom->getPosition() - baseVertex->getPosition() )
            )
        );

        return new Mesh(vertices, triangles);
    }

    /*!
     * \brief create a mesh represantation of the given \p cylinder
     * \param cylinder
     *
     * OWNERSHIP NOTICE: The caller takes ownership of the returned object
     */
    Mesh* MeshFactory::createCylinder(const Cylinder& cylinder) {
        return createCylinder(cylinder.getRadius(),cylinder.getHeight(),
                cylinder.getAverageEdgeLength());
    }

    /*!
     * \brief Subdivides an given triangle in the mesh approximation
     * 
     * The given \p triangle is subdivied into four equilateral triangles.
     * The figure below shows where the new three vertices lie.
     * 
     *        C
     *       / \
     *      b   a
     *     /     \ 
     *    A---c---B
     * 
     * A,B and C are the 'old' vertices of the triangle that should be
     * subdivided. a,b and c are the vertices of the four new triangles.
     * 
     * According to the new side length of the new triangles this method decides
     * if another subdivision is needed. Therefor the side length is compared
     * with the optional param \p averageEdgeLength if the side length
     * is greater than averageEdgeLength+0.1 the triangles are further
     * subdivied.
     * \param vertices All vertices
     * \param triangles All triangles
     * \param redundantTriangles 
     * \param combinations
     * \param triangle The triangle which should be divided
     * \param averageEdgeLength
     */
    void MeshFactory::subdivide(std::vector<Vertex*>& vertices,
                           std::vector<Triangle*>& triangles,
                           std::set<Triangle*>& redundantTriangles,
                           std::set<MultiMapElement<Vertex, Vertex*> >& combinations,
                           Triangle* triangle, real radius,
                           real averageEdgeLength) {

        //TODO Check if dcollide::Set<> should be used instead of std::Set<>

        Vertex* a = 0;
        Vertex* b = 0;
        Vertex* c = 0;

        std::set<MultiMapElement<Vertex, Vertex*> >::iterator it;


        /* get vertices of triangle that should be subdivded */
        Vertex* A = triangle->getVertices()[0];
        Vertex* B = triangle->getVertices()[1];
        Vertex* C = triangle->getVertices()[2];


        /* get or calculate new vertex 'a' */
        it = combinations.find(MultiMapElement<Vertex, Vertex*>(B,C));

        if (it != combinations.end()) {
            a = (*it).getData();

        } else {

            Vector3 BC = B->getPosition() - C->getPosition();
            Vector3 pointA = (BC / 2) + C->getPosition();
                    pointA = pointA.normalize() * radius;

            a = new Vertex(pointA);

            vertices.push_back(a);
            combinations.insert(MultiMapElement<Vertex, Vertex*>(B, C, a));
        }


        /* get or calculate new vertex 'b' */
        it = combinations.find(MultiMapElement<Vertex, Vertex*>(A,C));

        if (it != combinations.end()) {
            b = (*it).getData();

        } else {

            Vector3 AC = A->getPosition() - C->getPosition();
            Vector3 pointB = (AC / 2) + C->getPosition();
                    pointB = pointB.normalize() * radius;

            b = new Vertex(pointB);

            vertices.push_back(b);
            combinations.insert(MultiMapElement<Vertex, Vertex*>(A, C, b));
        }


        /* get or calculate new vertex 'c' */
        it = combinations.find(MultiMapElement<Vertex, Vertex*>(A,B));

        if (it != combinations.end()) {
            c = (*it).getData();

        } else {
            Vector3 AB = A->getPosition() - B->getPosition();
            Vector3 pointC = (AB / 2) + B->getPosition();
            pointC = pointC.normalize() * radius;

            c = new Vertex(pointC);

            vertices.push_back(c);
            combinations.insert(MultiMapElement<Vertex, Vertex*>(A, B, c));
        }


        /* generate the four new triangles */
        int firstTriangle = triangles.size();

        triangles.push_back(
            new Triangle(
                A, //new Vector3(A->getPosition()),
                c, //new Vector3(c->getPosition()),
                b//, new Vector3(b->getPosition())
            )
        );

        triangles.push_back(
            new Triangle(
                c, //new Vector3(c->getPosition()),
                a, //new Vector3(a->getPosition()),
                b//, new Vector3(b->getPosition())
            )
        );

        triangles.push_back(
            new Triangle(
                c, //new Vector3(c->getPosition()),
                B, //new Vector3(B->getPosition()),
                a//, new Vector3(a->getPosition())
            )
        );

        triangles.push_back(
            new Triangle(
                b, //new Vector3(b->getPosition()),
                a, //new Vector3(a->getPosition()),
                C//, new Vector3(C->getPosition())
            )
        );


        if (averageEdgeLength > 0) {
            /* subdivide even more if necessary */
            Vector3 side = A->getPosition() - a->getPosition();
            real sideLength = (side).length();

            if (   (averageEdgeLength <= sideLength)
                && (fabs(sideLength - averageEdgeLength) >= 0.1) ) {

                /* call subdivide for each of the newly created triangles */
                for (int i = 0; i < 4; i++) {

                    /* subdivide triangle */
                    subdivide(
                        vertices,
                        triangles,
                        redundantTriangles,
                        combinations,
                        triangles.at(firstTriangle+i),
                        radius,
                        averageEdgeLength
                    );

                    /* mark subdivided triangle as obsolete */
                    redundantTriangles.insert(triangles.at(firstTriangle+i));
                }
            }
        }
    }

}
