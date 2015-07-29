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
#include "boxsphereintersector.h"

#include "../math/plane.h"
#include "../math/vector.h"
#include "../math/matrix.h"
#include "../shapes/box.h"
#include "../shapes/sphere.h"
#include "../proxy.h"
#include "../collisioninfo.h"

namespace dcollide {

std::list<CollisionInfo> BoxSphereIntersector::getIntersection(
        const Matrix& currentStateBox,  const Box* box,
        const Matrix& currentStateSphere, const Sphere* sphere,
        bool sphereIsPenetrator) {


    //Used notations for vertices and edges of the box
    // vX: vertex x; eXY: edge from X to Y
    //
    //         ^ Z
    //         |
    //         |
    //        vE--------vH
    //       / |       / |
    //      /  |      /  |
    //     vF--|-----vG  |
    //     |  vA------|--vD-----> Y
    //     |  /    eCD|  /
    //     | /eAB     | /eCD
    //     |/         |/
    //    vB----------vC
    //    /   eBC
    //   X
    //
    // indices of the vertices: A=>0 B=>1 C=>2 D=>3(2^3=8)
    //                          E=>4(2^4=16) F=>(2^5=32) G=>6 H=>7(2^6=128)
    const int testedVertexA = 0x01;
    const int testedVertexB = 0x01 << 1;
    const int testedVertexC = 0x01 << 2;
    const int testedVertexD = 0x01 << 3;
    const int testedVertexE = 0x01 << 4;
    const int testedVertexF = 0x01 << 5;
    const int testedVertexG = 0x01 << 6;
    const int testedVertexH = 0x01 << 7;

    const int testedEdgeAB = 0x01 << 8;
    const int testedEdgeAD = 0x01 << 9;
    const int testedEdgeAE = 0x01 << 10;
    const int testedEdgeBC = 0x01 << 11;
    const int testedEdgeBF = 0x01 << 12;
    const int testedEdgeCD = 0x01 << 13;
    const int testedEdgeCG = 0x01 << 14;
    const int testedEdgeDH = 0x01 << 15;
    const int testedEdgeEF = 0x01 << 16;
    const int testedEdgeEH = 0x01 << 17;
    const int testedEdgeFG = 0x01 << 18;
    const int testedEdgeGH = 0x01 << 19;

    //The trick is to find the point p on the box with the minimal distance to
    //the center m of the sphere. The point could be a vertex, on an edge or on
    //one face of the box.
    Vector3 minimalDistancePoint;
    real minimalDistance = 10000; //huge value, calculated distances are smaller

    //First candidates: Faces
    //A calculate projection of m onto the plane defined by the face (using the face normal/edge direction)
    //if the projected point is within the face, we can use it.
    //B Else, it is invalid and we must instead test all vertices at that face

    //keep track of with vertices were checked before to prevent double
    //calculation
    //8 vertices, 12 edges
    //use a bitflag field for this to save memory
    unsigned int elementsTested = 0;

    //A calculate projections onto plane
    //Instead of calculating each plane, we only need 3, the others are identical with an offset in x/y or z-direction

    //center of sphere in world-coordinates:
    Vector3 sphereCenter = currentStateSphere.getPosition();

    //calculate world-coordinates of needed vertices
    Vector3 vertexA = currentStateBox.getPosition();//base point

    Vector3 vertexB; //follow x-axis
    currentStateBox.transform(&vertexB,Vector3(box->getDimension().getX(), 0.0, 0.0));

    Vector3 vertexD; //follow y-axis
    currentStateBox.transform(&vertexD,Vector3(0.0, box->getDimension().getY(), 0.0));

    Vector3 vertexE; //follow z-axis
    currentStateBox.transform(&vertexE,Vector3(0.0, 0.0, box->getDimension().getZ()));

    //create planes by normal and point-on-plane
    //the three normals, pointing outward (also the edges of the box)
    Vector3 edgeEA = vertexA - vertexE;
    Vector3 edgeDA = vertexA - vertexD;
    Vector3 edgeBA = vertexA - vertexB;

    Plane planeXY_DCBA(edgeEA, vertexA);
    Plane planeXZ_ABFE(edgeDA, vertexA);
    Plane planeYZ_AEHD(edgeBA, vertexA);
    //GJ: sorry for the use of underscores, but planeXYABCD looks horrible

    //Project sphere-center m along the normals, by the distance of m to the planes

    //----------- Projection onto face(s) in DCBA and EFGH -----------//
    real sphereFaceDistance = planeXY_DCBA.calculateDistance(sphereCenter);
    Vector3 projSphereCenterOnPlane = sphereCenter - planeXY_DCBA.getNormal() * sphereFaceDistance;
    //check if the point is on the face: X-and-Y
    //determine the projection as linear combination of the edges AD and AE
    // proj = vertexA + u * edgeAE + v * edgeAD
    // the point is on the face if u and v are both within the interval [0...1]
    real u=0, v=0;
    findPlaneLinearCombination(projSphereCenterOnPlane, vertexA, edgeBA, edgeDA, 2, u, v);
    //since we have used the inverted edges, we need to multiply u and v with -1
    u *= -1;
    v *= -1;

    //separate the checks to decide which edges/vertices can hold the point
    //that is closest to the sphere-center.
    //Warning: big ugly if-then-else monster coming up.
    //Get pen and paper to understand this, this is just impossible to explain
    //in words and ascii art only.

    //here is a version without implementation to help geting an idea of what
    // is going on in the following lines
/*
    if (u >= 0) {
        if (u <= 1) {
            // u is valid
            if (v >= 0) {
                if (v <= 1) {
                    // u is valid, v is valid => contact on face
                } else {
                    // u is valid, v is too big => contact on edgeCD or GH
                }//end if v<=1
            } else {
                //u is valid, v is too small => contact on edgeAB or edgeEF
            }//end if v>=0
        } else {
            //u is too big
            if (v >= 0) {
                if (v <= 1) {
                    // u is too big, v is valid => contact on edgeBC or edgeFG
                } else {
                    // u is too big, v is too big => contact on vertC or vertG
                }//end if v<=1
            } else {
                //u is too big, v is too small => contact on vB or vF
            }//end if v>=0
        }//end if u <=1
    } else {
        // u is too small
        if (v >= 0) {
            if (v <= 1) {
                    // u is too small, v is valid => contact on edgeAD or edgeGH
            } else {
                    // u is too small, v is too big => contact on vertD or vertH
            }//end if v<=1
        } else {
                //u is too small, v is too small => contact on vA or vE
        }//end if v>=0
    }//end if
*/
    if (u >= 0) {
        if (u <= 1) {
            // u is valid
            if (v >= 0) {
                if (v <= 1) {
                    // u is valid, v is valid => contact on face
                    real opposingFaceDistance = sphereFaceDistance + box->getDimension()[2];

                    real opposingFaceDistanceAbs = fabs(opposingFaceDistance);
                    real sphereFaceDistanceAbs = fabs(sphereFaceDistance);

                    if (sphereFaceDistanceAbs <= opposingFaceDistanceAbs) {
                        minimalDistancePoint = projSphereCenterOnPlane;
                        minimalDistance = sphereFaceDistanceAbs;
                    } else {
                        minimalDistancePoint = projSphereCenterOnPlane - edgeEA;
                        minimalDistance = opposingFaceDistanceAbs;
                    }
                } else {
                    // u is valid, v is too big => contact on edgeCD or GH
                    Vector3 edgeCDcandidate = vertexD - edgeBA * u;
                    testCandidate(edgeCDcandidate, sphereCenter, minimalDistancePoint, minimalDistance);
                    //now check the other point on edgeGH
                    testCandidate(edgeCDcandidate-edgeEA, sphereCenter, minimalDistancePoint, minimalDistance);
                    elementsTested |= testedEdgeCD | testedEdgeGH;
                }//end if v<=1
            } else {
                //u is valid, v is too small => contact on edgeAB or edgeEF
                Vector3 edgeABcandidate = vertexA - edgeBA * u;
                testCandidate(edgeABcandidate, sphereCenter, minimalDistancePoint, minimalDistance);
                    //now check the other point on edgeEF
                testCandidate(edgeABcandidate-edgeEA, sphereCenter, minimalDistancePoint, minimalDistance);
                elementsTested |= testedEdgeAB | testedEdgeEF;
            }//end if v>=0
        } else {
            //u is too big
            if (v >= 0) {
                if (v <= 1) {
                    // u is too big, v is valid => contact on edgeBC or edgeFG
                    Vector3 edgeBCcandidate = vertexB - edgeDA * v;
                    testCandidate(edgeBCcandidate, sphereCenter, minimalDistancePoint, minimalDistance);
                    //now check the other point on edgeFG
                    testCandidate(edgeBCcandidate-edgeEA, sphereCenter, minimalDistancePoint, minimalDistance);
                    elementsTested |= testedEdgeBC | testedEdgeFG;
                } else {
                    // u is too big, v is too big => contact on vertC or vertG
                    // C has not been calculated, C = A + edgeAB + edgeAD
                    Vector3 vertexC = vertexA - edgeBA - edgeDA;
                    testCandidate(vertexC, sphereCenter, minimalDistancePoint, minimalDistance);
                    testCandidate(vertexC-edgeEA, sphereCenter, minimalDistancePoint, minimalDistance);
                    elementsTested |= testedVertexC | testedVertexG;
                }//end if v<=1
            } else {
                //u is too big, v is too small => contact on vB or vF
                testCandidate(vertexB, sphereCenter, minimalDistancePoint, minimalDistance);
                testCandidate(vertexB-edgeEA, sphereCenter, minimalDistancePoint, minimalDistance);
                elementsTested |= testedVertexB | testedVertexF;
            }//end if v>=0
        }//end if u <=1
    } else {
        // u is too small
        if (v >= 0) {
            if (v <= 1) {
                // u is too small, v is valid => contact on edgeAD or edgeGH
                Vector3 edgeADcandidate = vertexA - edgeDA * v;
                testCandidate(edgeADcandidate, sphereCenter, minimalDistancePoint, minimalDistance);
                    //now check the other point on edgeFG
                testCandidate(edgeADcandidate-edgeEA, sphereCenter, minimalDistancePoint, minimalDistance);
                elementsTested |= testedEdgeAD | testedEdgeEF;

            } else {
                // u is too small, v is too big => contact on vertD or vertH
                testCandidate(vertexD, sphereCenter, minimalDistancePoint, minimalDistance);
                testCandidate(vertexD-edgeEA, sphereCenter, minimalDistancePoint, minimalDistance);
                elementsTested |= testedVertexD | testedVertexH;
            }//end if v<=1
        } else {
            //u is too small, v is too small => contact on vA or vE
            testCandidate(vertexA, sphereCenter, minimalDistancePoint, minimalDistance);
            testCandidate(vertexE, sphereCenter, minimalDistancePoint, minimalDistance);
            elementsTested |= testedVertexA | testedVertexE;
        }//end if v>=0
    }//end if

    //----------- Projection onto face(s) ABFE and GCDH -----------//
    sphereFaceDistance = planeXZ_ABFE.calculateDistance(sphereCenter);
    projSphereCenterOnPlane = sphereCenter - planeXZ_ABFE.getNormal() * sphereFaceDistance;
    //check if the point is on the face: X-and-Z
    //determine the projection as linear combination of the edges AB and AE
    // proj = vertexA + u * edgeAB + v * edgeAE
    // the point is on the face if u and v are both within the interval [0...1]
    findPlaneLinearCombination(projSphereCenterOnPlane, vertexA, edgeBA, edgeEA, 1, u, v);
    //since we have used the inverted edges, we need to multiply u and v with -1
    u *= -1;
    v *= -1;

    //separate the checks to decide which edges/vertices can hold the point
    //that is closest to the sphere-center.
    //Warning: big ugly if-then-else monster coming up.
    //Get pen and paper to understand this, this is just impossible to explain
    //in words and ascii art only.

    if (u >= 0) {
        if (u <= 1) {
            // u is valid
            if (v >= 0) {
                if (v <= 1) {
                    // u is valid, v is valid => contact on face
                    real opposingFaceDistance = sphereFaceDistance + box->getDimension()[1];

                    real opposingFaceDistanceAbs = fabs(opposingFaceDistance);
                    real sphereFaceDistanceAbs = fabs(sphereFaceDistance);

                    if (sphereFaceDistanceAbs <= opposingFaceDistanceAbs) {
                        if (sphereFaceDistanceAbs < minimalDistance) {
                            minimalDistancePoint = projSphereCenterOnPlane;
                            minimalDistance = sphereFaceDistanceAbs;
                        }
                    } else {
                        if (opposingFaceDistanceAbs < minimalDistance) {
                            minimalDistancePoint = projSphereCenterOnPlane - edgeDA;
                            minimalDistance = opposingFaceDistanceAbs;
                        }
                    }
                } else {
                    // u is valid, v is too big => contact on edgeEF or GH
                    Vector3 edgeEFcandidate = vertexE - edgeBA * u;
                    if ((elementsTested & testedEdgeEF) == 0) {
                        testCandidate(edgeEFcandidate, sphereCenter, minimalDistancePoint, minimalDistance);
                        elementsTested |= testedEdgeEF;
                    }
                    //now check the other point on edgeGH if not done before
                    if ((elementsTested & testedEdgeGH) == 0) {
                        testCandidate(edgeEFcandidate-edgeDA, sphereCenter, minimalDistancePoint, minimalDistance);
                        elementsTested |= testedEdgeGH;
                    }

                }//end if v<=1
            } else {
                //u is valid, v is too small => contact on edgeAB or edgeCD
                Vector3 edgeABcandidate = vertexA - edgeBA * u;
                if ((elementsTested & testedEdgeAB) == 0) {
                    testCandidate(edgeABcandidate, sphereCenter, minimalDistancePoint, minimalDistance);
                    elementsTested |= testedEdgeAB;
                }
                //now check the other point on edgeCD
                if ((elementsTested & testedEdgeCD) == 0) {
                    testCandidate(edgeABcandidate-edgeDA, sphereCenter, minimalDistancePoint, minimalDistance);
                    elementsTested |= testedEdgeCD;
                }
            }//end if v>=0
        } else {
            //u is too big
            if (v >= 0) {
                if (v <= 1) {
                    // u is too big, v is valid => contact on edgeBF or edgeCG
                    Vector3 edgeBFcandidate = vertexB - edgeEA * v;
                    if ((elementsTested & testedEdgeBF) == 0) {
                        testCandidate(edgeBFcandidate, sphereCenter, minimalDistancePoint, minimalDistance);
                        elementsTested |= testedEdgeBF;
                    }
                    //now check the other point on edgeCG
                    if ((elementsTested & testedEdgeCG) == 0) {
                        testCandidate(edgeBFcandidate-edgeDA, sphereCenter, minimalDistancePoint, minimalDistance);
                        elementsTested |= testedEdgeCG;
                    }
                } else {
                    // u is too big, v is too big => contact on vertF or vertG
                    //we do not need to test vertex F if an edge containing that point has been tested before
                    //F has not been calculated, F = B + edgeAE
                    Vector3 vertexF = vertexB - edgeEA;
                    if ((elementsTested & (testedVertexF | testedEdgeBF | testedEdgeEF | testedEdgeFG)) == 0) {
                        testCandidate(vertexF, sphereCenter, minimalDistancePoint, minimalDistance);
                        elementsTested |= testedVertexF;
                    }
                    //test vertexG
                    if ((elementsTested & (testedVertexG | testedEdgeCG | testedEdgeFG | testedEdgeGH)) == 0) {
                        testCandidate(vertexF - edgeDA, sphereCenter, minimalDistancePoint, minimalDistance);
                        elementsTested |= testedVertexG;
                    }

                }//end if v<=1
            } else {
                //u is too big, v is too small => contact on vB or vC
                if ((elementsTested & (testedVertexB | testedEdgeAB | testedEdgeBC | testedEdgeBF)) == 0) {
                    testCandidate(vertexB, sphereCenter, minimalDistancePoint, minimalDistance);
                    elementsTested |= testedVertexB;
                }
                //test vertexC
                if ((elementsTested & (testedVertexC | testedEdgeBC | testedEdgeCD | testedEdgeCG)) == 0) {
                    testCandidate(vertexB -edgeDA, sphereCenter, minimalDistancePoint, minimalDistance);
                    elementsTested |= testedVertexC;
                }
            }//end if v>=0
        }//end if u <=1
    } else {
        // u is too small
        if (v >= 0) {
            if (v <= 1) {
                // u is too small, v is valid => contact on edgeAE or edgeDH
                Vector3 edgeAEcandidate = vertexA - edgeEA * v;
                if ((elementsTested & testedEdgeAE) == 0) {
                    testCandidate(edgeAEcandidate, sphereCenter, minimalDistancePoint, minimalDistance);
                    elementsTested |= testedEdgeAE;
                }
                    //now check the other point on edgeDH
                if ((elementsTested & testedEdgeDH) == 0) {
                    testCandidate(edgeAEcandidate-edgeDA, sphereCenter, minimalDistancePoint, minimalDistance);
                    elementsTested |= testedEdgeDH;
                }

            } else {
                // u is too small, v is too big => contact on vertE or vertH
                if ((elementsTested & (testedVertexE | testedEdgeAE | testedEdgeEF | testedEdgeEH)) == 0) {
                    testCandidate(vertexE, sphereCenter, minimalDistancePoint, minimalDistance);
                    elementsTested |= testedVertexE;
                }
                //test vertexH
                if ((elementsTested & (testedVertexH | testedEdgeDH | testedEdgeEH | testedEdgeGH)) == 0) {
                    testCandidate(vertexE - edgeDA, sphereCenter, minimalDistancePoint, minimalDistance);
                    elementsTested |= testedVertexH;
                }
            }//end if v<=1
        } else {
            //u is too small, v is too small => contact on vertA or vertD
            if ((elementsTested & (testedVertexA | testedEdgeAB | testedEdgeAD | testedEdgeAE)) == 0) {
                testCandidate(vertexA, sphereCenter, minimalDistancePoint, minimalDistance);
                elementsTested |= testedVertexA;
            }
            //test vertexD
            if ((elementsTested & (testedVertexD | testedEdgeAD | testedEdgeCD | testedEdgeDH)) == 0) {
                testCandidate(vertexD, sphereCenter, minimalDistancePoint, minimalDistance);
                elementsTested |= testedVertexD;
            }
        }//end if v>=0
    }//end if

    //----------- Projection onto face(s) ADHE and BCGF -----------//
    sphereFaceDistance = planeYZ_AEHD.calculateDistance(sphereCenter);
    projSphereCenterOnPlane = sphereCenter - planeYZ_AEHD.getNormal() * sphereFaceDistance;
    //check if the point is on the face: Y-and-Z
    //determine the projection as linear combination of the edges AD and AE
    // proj = vertexA + u * edgeAD + v * edgeAE
    // the point is on the face if u and v are both within the interval [0...1]
    findPlaneLinearCombination(projSphereCenterOnPlane, vertexA, edgeDA, edgeEA, 0, u, v);
    //since we have used the inverted edges, we need to multiply u and v with -1
    u *= -1;
    v *= -1;

    //separate the checks to decide which edges/vertices can hold the point
    //that is closest to the sphere-center.
    //Warning: big ugly if-then-else monster coming up.
    //Get pen and paper to understand this, this is just impossible to explain
    //in words and ascii art only.

    if (u >= 0) {
        if (u <= 1) {
            // u is valid
            if (v >= 0) {
                if (v <= 1) {
                    // u is valid, v is valid => contact on face
                    real opposingFaceDistance = sphereFaceDistance + box->getDimension()[0];

                    real opposingFaceDistanceAbs = fabs(opposingFaceDistance);
                    real sphereFaceDistanceAbs = fabs(sphereFaceDistance);

                    if (sphereFaceDistanceAbs <= opposingFaceDistanceAbs) {
                        if (sphereFaceDistanceAbs < minimalDistance) {
                            minimalDistancePoint = projSphereCenterOnPlane;
                            minimalDistance = sphereFaceDistanceAbs;
                        }
                    } else {
                        if (opposingFaceDistanceAbs < minimalDistance) {
                            minimalDistancePoint = projSphereCenterOnPlane - edgeBA;
                            minimalDistance = opposingFaceDistanceAbs;
                        }
                    }
                } else {
                    // u is valid, v is too big => contact on edgeEH or FG
                    Vector3 edgeEHcandidate = vertexE - edgeDA * u;
                    if ((elementsTested & testedEdgeEH) != 0) {
                        testCandidate(edgeEHcandidate, sphereCenter, minimalDistancePoint, minimalDistance);
                        //since we won't use the flags after this step, we don't
                        //need to set them anymore
                        //elementsTested |= testedEdgeEH;
                    }
                    //now check the other point on edgeFG if not done before
                    if ((elementsTested & testedEdgeFG) != 0) {
                        testCandidate(edgeEHcandidate-edgeBA, sphereCenter, minimalDistancePoint, minimalDistance);
                        //elementsTested |= testedEdgeFG;
                    }

                }//end if v<=1
            } else {
                //u is valid, v is too small => contact on edgeAD or edgeBC
                Vector3 edgeADcandidate = vertexA - edgeDA * u;
                if ((elementsTested & testedEdgeAD) != 0) {
                    testCandidate(edgeADcandidate, sphereCenter, minimalDistancePoint, minimalDistance);
                    //elementsTested |= testedEdgeAD;
                }
                    //now check the other point on edgeBC if not done before
                if ((elementsTested & testedEdgeBC) != 0) {
                    testCandidate(edgeADcandidate-edgeBA, sphereCenter, minimalDistancePoint, minimalDistance);
                    //elementsTested |= testedEdgeBC;
                }
            }//end if v>=0
        } else {
            //u is too big
            if (v >= 0) {
                if (v <= 1) {
                    // u is too big, v is valid => contact on edgeDH or edgeCG
                    Vector3 edgeDHcandidate = vertexD - edgeEA * v;
                    if ((elementsTested & testedEdgeDH) != 0) {
                        testCandidate(edgeDHcandidate, sphereCenter, minimalDistancePoint, minimalDistance);
                        //elementsTested |= testedEdgeDH;
                    }
                    //now check the other point on edgeCG
                    if ((elementsTested & testedEdgeCG) != 0) {
                        testCandidate(edgeDHcandidate-edgeBA, sphereCenter, minimalDistancePoint, minimalDistance);
                        //elementsTested |= testedEdgeCG;
                    }
                } else {
                    // u is too big, v is too big => contact on vertH or vertG
                    //H has not been calculated, H = E + edgeAD
                    Vector3 vertexH = vertexE - edgeDA;
                    if ((elementsTested & (testedVertexH | testedEdgeDH | testedEdgeEH | testedEdgeGH)) != 0) {
                        testCandidate(vertexH, sphereCenter, minimalDistancePoint, minimalDistance);
                        //elementsTested |= testedVertexH;
                    }
                    //test vertexG
                    if ((elementsTested & (testedVertexG | testedEdgeCG | testedEdgeFG | testedEdgeGH)) != 0) {
                        testCandidate(vertexH - edgeBA, sphereCenter, minimalDistancePoint, minimalDistance);
                        //elementsTested |= testedVertexG;
                    }

                }//end if v<=1
            } else {
                //u is too big, v is too small => contact on vD or vC
                if ((elementsTested & (testedVertexD | testedEdgeAD | testedEdgeCD | testedEdgeDH)) != 0) {
                    testCandidate(vertexD, sphereCenter, minimalDistancePoint, minimalDistance);
                    //elementsTested |= testedVertexB;
                }
                //test vertexC
                if ((elementsTested & (testedVertexC | testedEdgeBC | testedEdgeCD | testedEdgeCG)) != 0) {
                    testCandidate(vertexD-edgeBA, sphereCenter, minimalDistancePoint, minimalDistance);
                    //elementsTested |= testedVertexC;
                }
            }//end if v>=0
        }//end if u <=1
    } else {
        // u is too small
        if (v >= 0) {
            if (v <= 1) {
                // u is too small, v is valid => contact on edgeAE or edgeBF
                Vector3 edgeAEcandidate = vertexA - edgeEA * v;
                if ((elementsTested & testedEdgeAE) != 0) {
                    testCandidate(edgeAEcandidate, sphereCenter, minimalDistancePoint, minimalDistance);
                    //elementsTested |= testedEdgeAE;
                }
                //now check the other point on edgeBF
                if ((elementsTested & testedEdgeBF) != 0) {
                    testCandidate(edgeAEcandidate-edgeBA, sphereCenter, minimalDistancePoint, minimalDistance);
                    //elementsTested |= testedEdgeBF;
                }

            } else {
                // u is too small, v is too big => contact on vertE or vertF
                if ((elementsTested & (testedVertexE | testedEdgeAE | testedEdgeEF | testedEdgeEH)) != 0) {
                    testCandidate(vertexE, sphereCenter, minimalDistancePoint, minimalDistance);
                    //elementsTested |= testedVertexE;
                }
                //test vertexF
                if ((elementsTested & (testedVertexF | testedEdgeBF | testedEdgeEF | testedEdgeFG)) != 0) {
                    testCandidate(vertexE - edgeBA, sphereCenter, minimalDistancePoint, minimalDistance);
                    //elementsTested |= testedVertexF;
                }
            }//end if v<=1
        } else {
            //u is too small, v is too small => contact on vertA or vertB
            if ((elementsTested & (testedVertexA | testedEdgeAB | testedEdgeAD | testedEdgeAE)) != 0) {
                testCandidate(vertexA, sphereCenter, minimalDistancePoint, minimalDistance);
                //elementsTested |= testedVertexA;
            }
            //test vertexB
            if ((elementsTested & (testedVertexB | testedEdgeAB | testedEdgeBC | testedEdgeBF)) != 0) {
                testCandidate(vertexB, sphereCenter, minimalDistancePoint, minimalDistance);
                //elementsTested |= testedVertexB;
            }
        }//end if v>=0
    }//end if



    //now we know the point on the box that is closest to the sphere-center
    //if there is a collision, the minimal distance is smaller than the radius
    //of the sphere. Else, there cannot be a collision
    if (minimalDistance > sphere->getRadius()) {
        //return an empty list
        return std::list<CollisionInfo>();
    }

    //there is a collision
    CollisionInfo coll;
    //Calculate penetration depth, collision point and normal
    coll.penetrationDepth = sphere->getRadius() - minimalDistance;
    coll.collisionPoint = minimalDistancePoint;
    //for the calculation of the normal, it is important to know which of the
    //shapes is the intruder
    if (sphereIsPenetrator) {
        coll.penetratingProxy = sphere->getProxy();
        coll.penetratedProxy = box->getProxy();
        coll.normal = sphereCenter - minimalDistancePoint;
        //rare special case: if the sphere is exactly at the center of the box,
        //the normal would be (0, 0,0) which is invalid
        //in this case, return an arbitrary collision normal vector of lenght 1
        if (coll.normal.isNull()){
            coll.normal = Vector3(1, 0, 0);
            coll.penetrationDepth = box->getDimension().getX() / 2;
        } else {
            coll.normal.normalize();
        }

    } else {
        //the box penetrated the sphere -> the collisionpoint is on the sphere
        //calculating that point would need another vector-normalizing operation,
        //so i guess its ok to return the same collisionpoint as before and just
        //inverting the normal
        coll.penetratedProxy = sphere->getProxy();
        coll.penetratingProxy = box->getProxy();
        coll.normal = minimalDistancePoint - sphereCenter;
        //rare special case: if the sphere is exactly at the center of the box,
        //the normal would be (0, 0,0) which is invalid
        //in this case, return an arbitrary collision normal vector of lenght 1
        if (coll.normal.isNull()){
            coll.normal = Vector3(1, 0, 0);
            coll.penetrationDepth = sphere->getRadius();
        } else {
            coll.normal.normalize();
        }

    }

    std::list<CollisionInfo> result;
    result.push_back(coll);
    return result;
}

/*!
 * \brief solves p = planeBase + u * directionU + v * directionV
 *
 * Any point on a plane can be written as a linear combination of the plane-base
 * and two directional vectors in the plane
 * \return true if a linear combination was found. In that case,
 * \p u and \p v are set to the appropriate values
 */
bool BoxSphereIntersector::findPlaneLinearCombination(    const Vector3& point,
                                                const Vector3& planeBase,
                                                const Vector3& directionU,
                                                const Vector3& directionV,
                                                int redundantDirection,
real& u, real& v) const {
    //GJ: for now, we can be sure that this is called with \p point beeing a
    //point on the plane, so we don't need to check that
    // solving
    // pX = baseX + u*dirUX + v*dirVX <=> pX - baseX = u*dirUX + v*dirVX
    // pY = baseY + u*dirUY + v*dirVY <=> pY - baseY = u*dirUY + v*dirVY
    // pZ = baseZ + u*dirUZ + v*dirVZ <=> pZ - baseZ = u*dirUZ + v*dirVZ
    // 3 equitations, 2 unknown variables.
    // In our case (2D-Projection), one of these lines reads 0 = u * 0 + v * 0
    // we can ignore that line and solve for u and v with the two remaining ones

    //std::cout << "findPlaneLinearCombination(): solve p"<<point<<" - base"
    //          << planeBase <<" = "<< point-planeBase
    //          << " = u*dirU"<< directionU << " + v*dirV"<< directionV
    //          << ", redundant: " << redundantDirection << std::endl;

    //the redundant equitation is given by the user

    switch (redundantDirection) {
        case 2: {
            // solving in X and Y gives us (thank you, wxMaxima!)
            // u = -(dirVX*(baseY-pY)+dirVY*pX-baseX*dirVY)/(dirUY*dirVX-dirUX*dirVY)
            // v =  (dirUX*(baseY-pY)+dirUY*pX-baseX*dirUY)/(dirUY*dirVX-dirUX*dirVY)

            //third equitation is redundant, solve with X-and-Y

            //we can precalculate some parts of the equitations to reuse them
            // for calculating v: (baseY-pY) and the denominator are the same

            //save 1 subtraction
            real baseYminusPointY = planeBase.getY() - point.getY();
            //save 2 multiplications and 1 subtraction
            real denominator =      directionU.getY() * directionV.getX()
                                 -  directionU.getX() * directionV.getY();

            u = - ( directionV.getX() * (baseYminusPointY)
                +directionV.getY() * point.getX() - planeBase.getX() * directionV.getY())
                / (denominator);
            v =  (  directionU.getX() * (baseYminusPointY)
                +directionU.getY() * point.getX() - planeBase.getX() * directionU.getY())
                / (denominator);
            return true;
        }
        case 1: {
            //second equitation is redundant, solve with X-and-Z
            //save 1 subtraction
            real baseZminusPointZ = planeBase.getZ() - point.getZ();
            //save 2 multiplications and 1 subtraction
            real denominator =      directionU.getZ() * directionV.getX()
                                -   directionU.getX() * directionV.getZ();

            u = - ( directionV.getX() * (baseZminusPointZ)
                    +directionV.getZ() * point.getX() - planeBase.getX() * directionV.getZ())
                / (denominator);
            v =  (  directionU.getX() * (baseZminusPointZ)
                    +directionU.getZ() * point.getX() - planeBase.getX() * directionU.getZ())
                /(denominator);
            return true;
        }
        case 0: {
            //first equitation is redundant, solve with Y-and-Z
            //save 1 subtraction
            real baseZminusPointZ = planeBase.getZ() - point.getZ();
            //save 2 multiplications and 1 subtraction
            real denominator =      directionU.getZ() * directionV.getY()
                                -   directionU.getY() * directionV.getZ();
            u = - ( directionV.getY() * (baseZminusPointZ)
                    +directionV.getZ() * point.getY() - planeBase.getY() * directionV.getZ())
                / (denominator);
            v =  (  directionU.getY() * (baseZminusPointZ)
                    +directionU.getZ() * point.getY() - planeBase.getY() * directionU.getZ())
                /(denominator);
            return true;
        }

    }

    return false;
}

/*!
 * \brief checks if the new point is closer to the sphere as the current min
 *
 * \return true if the new point is closer (minimalDistancePoint is changed
 * in that case.
 */
bool BoxSphereIntersector::testCandidate (const Vector3& candidate,
                                          const Vector3& sphereCenter,
                        Vector3& minimalDistancePoint, real& minimalDistance) {
    //only test the point if we have not tested its edge
    real candidateDistance = (candidate - sphereCenter).lengthApproxBabylonian(2);
    if (candidateDistance < minimalDistance) {
        minimalDistancePoint = candidate;
        minimalDistance  = candidateDistance;
        return true;
    }
    return false;
}
}
/*
 * vim: et sw=4 ts=4
 */
