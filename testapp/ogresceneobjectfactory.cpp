/*******************************************************************************
 *  Copyright (C) 2007 by the members of PG 510, University of Dortmund:       *
 *              d-collide-devel@lists.sourceforge.net                          *
 *
 *     Andreas Beckermann, Christian Bode, Marcel Ens, Sebastian Ens,          *
 *     Martin Fassbach, Maximilian Hegele, Daniel Haus, Oliver Horst,         *
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

#include "ogresceneobjectfactory.h"
#include "ogrematerials.h"
#include "myobjectnode.h"

#include <d-collide/proxy.h>
#include <d-collide/shapes/shapes.h>
#include <d-collide/math/plane.h>
#include <d-collide/boundingvolumes/aabb.h>
#include <d-collide/boundingvolumes/obb.h>
#include <d-collide/boundingvolumes/kdop.h>
#include <d-collide/boundingvolumes/boundingsphere.h>
#include <d-collide/broadphase/hierarchicalgrid.h>
#include <d-collide/bvhnode.h>
#include <d-collide/debugstream.h>
#include <d-collide/exceptions/exception.h>

#include <OgreManualObject.h>
#include <OgreSceneManager.h>
#include <OgreSceneNode.h>
#include <OgreString.h>
#include <OgreMaterial.h>
#include <OgreMaterialManager.h>

int OgreSceneObjectFactory::mObjectNameNumber = 0;


struct Line {
    Line(const dcollide::Vector3& p, const dcollide::Vector3& v)
        : point(p), vector(v) {
    }
    dcollide::Vector3 point;
    dcollide::Vector3 vector;
};



OgreSceneObjectFactory::OgreSceneObjectFactory(Ogre::SceneManager* manager) {
    mOgreSceneManager = manager;
    if (!mOgreSceneManager) {
        // TODO: exception
        std::cerr << "a NULL scene manager is not allowed" << std::endl;
    }
}

OgreSceneObjectFactory::~OgreSceneObjectFactory() {
}

// note: a NULL shape is perfectly valid here!
Ogre::String OgreSceneObjectFactory::createUniqueObjectName(const dcollide::Shape*) {
    mObjectNameNumber++;

    std::stringstream ss;
    ss << "OgreSceneObjectFactory_Object_";
    ss << mObjectNameNumber;

    // FIXME: can we always assume that Ogre::String is a std::String?
    Ogre::String name;
    ss >> name;

    return name;
}

Ogre::String OgreSceneObjectFactory::createUniqueObjectName(const dcollide::BoundingVolume*) {
    mObjectNameNumber++;

    std::stringstream ss;
    ss << "OgreSceneObjectFactory_Object_";
    ss << mObjectNameNumber;

    // FIXME: can we always assume that Ogre::String is a std::String?
    Ogre::String name;
    ss >> name;

    return name;
}

/*!
 * Creates an OGRE based hierarchy that reflects \p objectHierarchy
 *
 * The toplevel \ref Ogre::SceneNode represents the position and orientation of
 * the dcollide::Proxy, whereas the Ogre object attached to it (see \ref
 * Ogre::SceneNode::attachObject) represents the \ref dcollide::Shape.
 *
 * Each child proxy of the \p objectHierarchy is represented by yet another \ref
 * Ogre::SceneNode.
 *
 * \param derivedScale The scale of all ancestors of \p objectHierarchy, NOT
 * including the scale of \p objectHierarchy itself. This parameter is required
 * because Ogre does not apply scaling values to translations (only to the
 * vertices), but we apply scaling values to translations, too.
 *
 * \return A \ref Ogre::SceneNode object containing an Ogre representation of \p
 * objectHierarchy. The node is NOT added to anything, it must be added to (e.g.) the \ref
 * SceneManager::getRootSceneNode before it is of any use.
 */
Ogre::SceneNode* OgreSceneObjectFactory::createSceneHierarchy(MyObjectNode* objectHierarchy) {
    if (!objectHierarchy) {
        throw dcollide::NullPointerException("objectHierarchy");
    }
    if (!objectHierarchy->getProxy()) {
        throw dcollide::NullPointerException("objectHierarchy->getProxy()");
    }
    if (objectHierarchy->getOgreSceneNode()) {
        throw dcollide::Exception("ogre hierarchy already created");
    }

    Ogre::SceneNode* node = mOgreSceneManager->createSceneNode();

    dcollide::Vector3 translation = objectHierarchy->getTranslation();

    // AB: WARNING: we assume that the 3x3 matrix in the Proxy matrix is
    //     actually a rotation matrix.
    //     I have _NO_ idea what happens if it also contains things like
    //     scalings or even worse stuff.
    const dcollide::Matrix& matrix = objectHierarchy->getRotation();
    Ogre::Matrix3 rot(matrix.getElement(0, 0), matrix.getElement(0, 1), matrix.getElement(0, 2),
                matrix.getElement(1, 0), matrix.getElement(1, 1), matrix.getElement(1, 2),
                matrix.getElement(2, 0), matrix.getElement(2, 1), matrix.getElement(2, 2));
    Ogre::Quaternion quat(rot);

    node->setPosition(translation[0], translation[1], translation[2]);
    node->setOrientation(quat);

    if (objectHierarchy->getProxy()->getShape()) {
        Ogre::ManualObject* shapeObject
            = createManualObject(
                  objectHierarchy->getProxy()->getShape(),
                  objectHierarchy->getTextureInformation(),
                  objectHierarchy->getUseCulling(),
                  objectHierarchy->getUseTextures()
              );
        node->attachObject(shapeObject);
    }

    const std::list<MyObjectNode*>& children = objectHierarchy->getChildren();
    for (std::list<MyObjectNode*>::const_iterator it = children.begin(); it != children.end(); ++it) {
        Ogre::SceneNode* child = createSceneHierarchy(*it);
        (*it)->setOgreSceneNode(child);
        node->addChild(child);
    }

    return node;
}

Ogre::SceneNode* OgreSceneObjectFactory::createBoundingVolumeSceneHierarchy(const dcollide::BvhNode* boundingVolumeNode) {
    if (!boundingVolumeNode) {
        throw dcollide::NullPointerException("boundingVolumeNode");
    }
    if (!boundingVolumeNode->getBoundingVolume()) {
        throw dcollide::NullPointerException("boundingVolumeNode->getBoundingVolume()");
    }

    if (boundingVolumeNode->mUser) {
        std::cerr << "ERROR: boundingVolumeNode->mUser is already used" << std::endl;
        return 0;
    }
    Ogre::SceneNode* sceneNode = mOgreSceneManager->createSceneNode();

    //std::cout << "create hierarchy volumeType_bvhnode" << boundingVolumeNode->getBoundingVolume()->getVolumeType() << std::endl;
    Ogre::ManualObject* boundingVolumeObject = createManualObject(boundingVolumeNode->getBoundingVolume());
    //int volumeTypeTemp = Ogre::any_cast<int>(boundingVolumeObject->getUserAny());
    //std::cout << "create volumeType manualobject" << volumeTypeTemp << std::endl;

    if (!boundingVolumeObject) {
        std::cerr << "could not create ogre object from bounding volume" << std::endl;
        return sceneNode;
    }
    sceneNode->attachObject(boundingVolumeObject);
#ifdef __GNUC__
#warning FIXME
#endif
    // AB: ugly. FIXME: use non-const parameter? or make mUser mutable?
    const_cast<dcollide::BvhNode*>(boundingVolumeNode)->mUser = boundingVolumeObject;

    const std::list<dcollide::BvhNode*>& children = boundingVolumeNode->getChildren();
    for (std::list<dcollide::BvhNode*>::const_iterator it = children.begin(); it != children.end(); ++it) {
        Ogre::SceneNode* child = createBoundingVolumeSceneHierarchy(*it);
        sceneNode->addChild(child);
    }

    return sceneNode;
}

Ogre::SceneNode* OgreSceneObjectFactory::createHierarchicalGrid(
        const dcollide::HierarchicalGrid* grid) {
    if (!grid) {
        return 0;
    }
    Ogre::SceneNode* sceneNode = mOgreSceneManager->createSceneNode();
    Ogre::ManualObject* object = mOgreSceneManager->
            createManualObject("OgreSceneObjectFactory_HierarchicalGrid");
    object->setDynamic(true);
    object->begin(OgreMaterials::getHierarchicalGridMaterial(),
            Ogre::RenderOperation::OT_LINE_LIST);
    specifyHierarchicalGrid(object, grid);
    object->end();

    sceneNode->attachObject(object);
    return sceneNode;
}

/*!
 * \return A \ref Ogre::ManualObject representing the \p shape. The object will
 * NOT be attached to any scenenode and thus won't be deleted automatically
 * before you use it in a \ref SceneNode::attachObject call.
 */
Ogre::ManualObject* OgreSceneObjectFactory::createManualObject(const dcollide::Shape* shape, const ModelLoader::TextureInformation& textureInformation, bool useCulling, bool useTextures) {
    if (!shape) {
        return 0;
    }

    Ogre::String name = createUniqueObjectName(shape);
    Ogre::ManualObject* object = mOgreSceneManager->createManualObject(name);
    Ogre::String materialName;
    {
        std::stringstream ss;
        ss << name << "_Material";
        ss >> materialName;
    }

    OgreMaterials::createShapeMaterial(materialName, textureInformation, useCulling, useTextures);

    const dcollide::Mesh* mesh = const_cast<dcollide::Shape*>(shape)->getMesh();
    const std::vector<dcollide::Triangle*>& triangles = mesh->getTriangles();

    object->estimateVertexCount(triangles.size() * 3);
    object->begin(materialName);
        specifyTrianglesForShape(object, shape, textureInformation, useTextures);
    object->end();

    return object;
}

Ogre::ManualObject* OgreSceneObjectFactory::createManualObject(const dcollide::BoundingVolume* boundingVolume) {
    if (!boundingVolume) {
        return 0;
    }

    Ogre::String name = createUniqueObjectName(boundingVolume);
    Ogre::ManualObject* object = mOgreSceneManager->createManualObject(name);

    initOgreObjectForBoundingVolume(object, boundingVolume);
    return object;
}

void OgreSceneObjectFactory::initOgreObjectForBoundingVolume(Ogre::ManualObject* object, const dcollide::BoundingVolume* boundingVolume) {
    if (!boundingVolume) {
        object->setUserAny(Ogre::Any((int)0));
        return;
    }

    object->setUserAny(Ogre::Any((int)boundingVolume->getVolumeType()));
    object->setDynamic(true);

    switch (boundingVolume->getVolumeType()) {
        case dcollide::BV_TYPE_AABB:
            initOgreObjectForAabb(object, static_cast<const dcollide::Aabb*>(boundingVolume));
            break;
        case dcollide::BV_TYPE_KDOP:
            initOgreObjectForKdop(object, static_cast<const dcollide::Kdop*>(boundingVolume));
            break;
        case dcollide::BV_TYPE_SPHERE:
            initOgreObjectForSphere(object, static_cast<const dcollide::BoundingSphere*>(boundingVolume));
            break;
        case dcollide::BV_TYPE_OBB:
            initOgreObjectForObb(object, static_cast<const dcollide::Obb*>(boundingVolume));
            break;

        // AB: do NOT add a default! -> this way we get compiler warnings if we
        //     forget a type
    }
}

void OgreSceneObjectFactory::initOgreObjectForAabb(Ogre::ManualObject* object, const dcollide::Aabb* aabb) {
    if (!object) {
        // TODO: exception
        return;
    }
    if (!aabb) {
        // TODO: exception
        return;
    }
    Ogre::String material = OgreMaterials::getBoundingVolumeMaterial();
    object->estimateVertexCount(8);
    object->estimateIndexCount(24);

    dcollide::Vector3 min = aabb->getMin();
    dcollide::Vector3 max = aabb->getMax();
    object->begin(material, Ogre::RenderOperation::OT_LINE_LIST);
        specifyBoundingVolumeLinesForAabb(object, aabb);
    object->end();
}

void OgreSceneObjectFactory::initOgreObjectForObb(Ogre::ManualObject* object,
        const dcollide::Obb* obb) {
    if (!object) {
        // TODO: exception
        return;
    }
    if (!obb) {
        // TODO: exception
        return;
    }
    Ogre::String material = OgreMaterials::getBoundingVolumeMaterial();
    object->estimateVertexCount(8);
    object->estimateIndexCount(24);

    object->begin(material, Ogre::RenderOperation::OT_LINE_LIST);
        specifyBoundingVolumeLinesForObb(object, obb);
    object->end();
}

void OgreSceneObjectFactory::initOgreObjectForKdop(Ogre::ManualObject* object, const dcollide::Kdop* kdop) {
    if (!object) {
        // TODO: exception
        return;
    }
    if (!kdop) {
        // TODO: exception
        return;
    }
    Ogre::String material = OgreMaterials::getBoundingVolumeMaterial();

    object->begin(material, Ogre::RenderOperation::OT_LINE_LIST);
        specifyBoundingVolumeLinesForKdop(object, kdop);
    object->end();
}

void OgreSceneObjectFactory::initOgreObjectForSphere(Ogre::ManualObject* object, const dcollide::BoundingSphere* sphere) {
    if (!object) {
        // TODO: exception
        return;
    }
    if (!sphere) {
        // TODO: exception
        return;
    }
    Ogre::String material = OgreMaterials::getBoundingVolumeMaterial();

    object->begin(material, Ogre::RenderOperation::OT_TRIANGLE_LIST);
        specifyBoundingVolumeTrianglesForSphere(object, sphere);
    object->end();
}

void OgreSceneObjectFactory::specifyBoundingVolumeLines(Ogre::ManualObject* object, const dcollide::BoundingVolume* bv) {
    if (!bv) {
        dcollide::error() << dc_funcinfo << "NULL BoundingVolume";
        return;
    }
    switch (bv->getVolumeType()) {
        case dcollide::BV_TYPE_AABB:
            specifyBoundingVolumeLinesForAabb(object, static_cast<const dcollide::Aabb*>(bv));
            break;
        case dcollide::BV_TYPE_KDOP:
            specifyBoundingVolumeLinesForKdop(object, static_cast<const dcollide::Kdop*>(bv));
            break;
        case dcollide::BV_TYPE_SPHERE:
            // FIXME: we are in specifyBoundingVolume_Lines_, but spheres used
            // triangles
            specifyBoundingVolumeTrianglesForSphere(object, static_cast<const dcollide::BoundingSphere*>(bv));
            break;
        case dcollide::BV_TYPE_OBB:
            specifyBoundingVolumeLinesForObb(object, static_cast<const
                    dcollide::Obb*>(bv));
            break;

        // AB: do NOT add a default! -> this way we get compiler warnings if we
        //     forget a type
    }

    dcollide::Vector3 bvMin = bv->getSurroundingAabbMin();
    dcollide::Vector3 bvMax = bv->getSurroundingAabbMax();
    Ogre::Vector3 min(bvMin[0], bvMin[1], bvMin[2]);
    Ogre::Vector3 max(bvMax[0], bvMax[1], bvMax[2]);
    Ogre::AxisAlignedBox aabb(min, max);
    object->setBoundingBox(aabb);
}

void OgreSceneObjectFactory::specifyBoundingVolumeLinesForAabb(Ogre::ManualObject* object, const dcollide::Aabb* aabb) {
    if (!aabb) {
        return;
    }
    dcollide::Vector3 min = aabb->getMin();
    dcollide::Vector3 max = aabb->getMax();

    object->position(min.getX(), min.getY(), min.getZ());
    object->position(min.getX(), min.getY(), max.getZ());
    object->position(min.getX(), max.getY(), min.getZ());
    object->position(min.getX(), max.getY(), max.getZ());
    object->position(max.getX(), min.getY(), min.getZ());
    object->position(max.getX(), min.getY(), max.getZ());
    object->position(max.getX(), max.getY(), min.getZ());
    object->position(max.getX(), max.getY(), max.getZ());

    object->index(2);
    object->index(0);

    object->index(1);
    object->index(0);

    object->index(1);
    object->index(3);

    object->index(2);
    object->index(3);

    object->index(4);
    object->index(5);

    object->index(6);
    object->index(4);

    object->index(5);
    object->index(7);

    object->index(6);
    object->index(7);

    object->index(4);
    object->index(0);

    object->index(1);
    object->index(5);

    object->index(2);
    object->index(6);

    object->index(3);
    object->index(7);
}

void OgreSceneObjectFactory::specifyBoundingVolumeLinesForObb(
        Ogre::ManualObject* object, const dcollide::Obb* obb) {
    if (!obb) {
        return;
    }

    dcollide::Vector3* verts = obb->getVertices();

    for (int i = 0; i<8;++i) {
        object->position(verts[i].getX(),verts[i].getY(),verts[i].getZ());
    }

    object->index(2);
    object->index(0);

    object->index(1);
    object->index(0);

    object->index(1);
    object->index(3);

    object->index(2);
    object->index(3);

    object->index(4);
    object->index(5);

    object->index(6);
    object->index(4);

    object->index(5);
    object->index(7);

    object->index(6);
    object->index(7);

    object->index(4);
    object->index(0);

    object->index(1);
    object->index(5);

    object->index(2);
    object->index(6);

    object->index(3);
    object->index(7);

    delete[] verts;
}

void OgreSceneObjectFactory::specifyBoundingVolumeLinesForKdop(Ogre::ManualObject* object, const dcollide::Kdop* kdop) {
    if (!kdop) {
        return;
    }

    /*
     * This code sucks terribly from an efficiency POV: it works in O(k^3).
     *
     * Unfortunately the other solutions I tried didn't work out:
     * 1. intersecting 3 neighboring planes to find a point and then connecting
     *    them properly did not work: sometimes the intersection of 3
     *    neighboring planes is outside the k-DOP. figuring out the correct
     *    planes in these cases is .. well, I did not manage to do that.
     * 2. Starting by the lines of a AABB and then intersecting the lines with
     *    each of the k-DOP planes should work, but whenever a line is
     *    completely behind a plane (i.e. not visible), we have to add two or
     *    more new lines. I could not find a "good" way to figure out which
     *    lines exactly to add (more precisely: which NOT to add. I could find
     *    all of the points, but simply connecting all of them with each other
     *    would result in too many lines)
     *
     * So we go with a brute force method here: intersect each plane with each
     * plane (O(k^2)) which gives us a set of lines. Each of these lines is then
     * intersected with each of the planes to get the correct line segments (or
     * discard the line entirely).
     *
     * It ain't nice, it ain't fast, but it works.
     */

    std::vector<dcollide::Plane> planes;
    planes.reserve(kdop->getK());
    for (int i = 0; i < kdop->getK(); i++) {
        planes.push_back(dcollide::Plane(kdop->getPlaneNormal(i), kdop->getDistanceOfPlaneToOrigin(i)));
    }

    std::list<Line> lines;
    dcollide::Vector3 intersectionPoint;
    dcollide::Vector3 intersectionVector;
    for (std::vector<dcollide::Plane>::iterator planeIt = planes.begin(); planeIt != planes.end(); ++planeIt) {
        const dcollide::Plane& plane1 = (*planeIt);
        std::vector<dcollide::Plane>::iterator planeIt2 = planeIt;
        ++planeIt2;
        for (; planeIt2 != planes.end(); ++planeIt2) {
            const dcollide::Plane& plane2 = (*planeIt2);
            if (!plane1.intersectPlane(plane2, &intersectionPoint, &intersectionVector)) {
                continue;
            }
            lines.push_back(Line(intersectionPoint, intersectionVector));
        }
    }

    for (std::list<Line>::iterator it = lines.begin(); it != lines.end(); ++it) {
        const Line& line = (*it);
        std::list<dcollide::Vector3> points;
        for (std::vector<dcollide::Plane>::iterator planeIt = planes.begin(); planeIt != planes.end(); ++planeIt) {
            const dcollide::Plane& plane = (*planeIt);
            if (plane.intersectLine(line.point, line.vector, &intersectionPoint)) {
                points.push_back(intersectionPoint);
            }
        }

        std::vector<dcollide::Vector3> lineSegmentPoints;
        for (std::list<dcollide::Vector3>::iterator it = points.begin(); it != points.end(); ++it) {
            bool behind = false;
            for (std::vector<dcollide::Plane>::iterator planeIt = planes.begin(); planeIt != planes.end(); ++planeIt) {
                const dcollide::Plane& plane = (*planeIt);
                if (plane.isOnPlane(*it)) {
                    continue;
                }
                bool a = plane.isInFrontOfPlane(*it);
                bool b = plane.isBehindPlane(*it);
                if (a && b) {
                    continue;
                }
                if (a) {
                    behind = true;
                    break;
                }
            }
            if (!behind) {
                lineSegmentPoints.push_back(*it);
            }
        }

        if (lineSegmentPoints.size() >= 2) {
            for (unsigned int i = 0; i < lineSegmentPoints.size(); i++) {
                for (unsigned int j = i + 1; j < lineSegmentPoints.size(); j++) {
                    object->position(lineSegmentPoints[i].getX(), lineSegmentPoints[i].getY(), lineSegmentPoints[i].getZ());
                    object->position(lineSegmentPoints[j].getX(), lineSegmentPoints[j].getY(), lineSegmentPoints[j].getZ());
                }
            }
        }
    }
}

void OgreSceneObjectFactory::specifyBoundingVolumeTrianglesForSphere(Ogre::ManualObject* object, const dcollide::BoundingSphere* boundingSphere) {
    if (!boundingSphere) {
        throw dcollide::NullPointerException("const dcollide::BoundingSphere* boundingSphere");
    }

    // AB: we use the defaul precision currently.
    //     therefore the bounding sphere does not look a lot like a bounding
    //     sphere, but it takes much less memory and processing power.
    //     feel free to adjust, if you want a nicer sphere
    //     (i.e. add a precision parameter here)
    // OH: A precision parameter around 0.4 is very well even for big spheres,
    //     but unfortunately the rendering gets very slow acording to the huge
    //     amount of triangles.
    const static dcollide::Mesh* mesh = 0;
    if (mesh == 0) {
        dcollide::Sphere sphere(1);

#ifdef __GNUC__
#warning FIXME: delete on destruction
#endif
        mesh = sphere.getMesh()->cloneMesh();
    }

    const static std::vector<dcollide::Triangle*>& triangles = mesh->getTriangles();

    for (std::vector<dcollide::Triangle*>::const_iterator it = triangles.begin(); it != triangles.end(); ++it) {
        const dcollide::Vertex* v;
        const dcollide::Vector3 normal = (*it)->getNormalVector();
        for (int i = 0; i < 3; i++) {
            v = (*it)->getVertices()[i];
            dcollide::Vector3 pos = boundingSphere->getCenterVector() +
                                    (v->getPosition() *
                                     boundingSphere->getRadius());

            object->position(pos.getX(), pos.getY(), pos.getZ());
            object->normal(normal.getX(), normal.getY(), normal.getZ());
        }
    }

    dcollide::Vector3 bvMin = boundingSphere->getSurroundingAabbMin();
    dcollide::Vector3 bvMax = boundingSphere->getSurroundingAabbMax();
    Ogre::Vector3 min(bvMin[0], bvMin[1], bvMin[2]);
    Ogre::Vector3 max(bvMax[0], bvMax[1], bvMax[2]);
    Ogre::AxisAlignedBox aabb(min, max);
    object->setBoundingBox(aabb);
}

void OgreSceneObjectFactory::specifyHierarchicalGrid(Ogre::ManualObject* object,
        const dcollide::HierarchicalGrid* grid) {
    specifyHierarchicalGridPoints(object, grid);
}

void OgreSceneObjectFactory::specifyHierarchicalGridPoints(Ogre::ManualObject*
        object, const dcollide::HierarchicalGrid* grid) {
    const dcollide::Vector3& min = grid->getGridMin();
    const dcollide::Vector3& max = grid->getGridMax();
    object->position(min.getX(), min.getY(), min.getZ());
    object->position(min.getX(), min.getY(), max.getZ());
    object->position(min.getX(), min.getY(), min.getZ());
    object->position(min.getX(), max.getY(), min.getZ());
    object->position(min.getX(), min.getY(), min.getZ());
    object->position(max.getX(), min.getY(), min.getZ());

    object->position(min.getX(), min.getY(), max.getZ());
    object->position(min.getX(), max.getY(), max.getZ());
    object->position(min.getX(), min.getY(), max.getZ());
    object->position(max.getX(), min.getY(), max.getZ());

    object->position(min.getX(), max.getY(), min.getZ());
    object->position(min.getX(), max.getY(), max.getZ());
    object->position(min.getX(), max.getY(), min.getZ());
    object->position(max.getX(), max.getY(), min.getZ());

    object->position(min.getX(), max.getY(), max.getZ());
    object->position(max.getX(), max.getY(), max.getZ());

    object->position(max.getX(), min.getY(), min.getZ());
    object->position(max.getX(), min.getY(), max.getZ());
    object->position(max.getX(), min.getY(), min.getZ());
    object->position(max.getX(), max.getY(), min.getZ());

    object->position(max.getX(), min.getY(), max.getZ());
    object->position(max.getX(), max.getY(), max.getZ());

    object->position(max.getX(), max.getY(), min.getZ());
    object->position(max.getX(), max.getY(), max.getZ());

    for (std::vector<dcollide::HierarchicalGrid*>::const_iterator it = grid->getChildrenConst().begin(); it != grid->getChildrenConst().end(); ++it) {
        specifyHierarchicalGridPoints(object, *it);
    }
}

void OgreSceneObjectFactory::specifyTrianglesForShape(Ogre::ManualObject* object,
                                                      const dcollide::Shape* shape,
                                                      const ModelLoader::TextureInformation& textureInformation,
                                                      bool useTextures) {
    
    const dcollide::Mesh* mesh = const_cast<dcollide::Shape*>(shape)->getMesh();
    const std::vector<dcollide::Triangle*>& triangles = mesh->getTriangles();
    const std::vector<dcollide::Vertex*>& vertices = mesh->getVertices();

    std::map<const dcollide::Vertex*, int> vertex2Index;
    int index = 0;
    for (std::vector<dcollide::Vertex*>::const_iterator it = vertices.begin(); it != vertices.end(); ++it) {
        vertex2Index.insert(std::make_pair(*it, index));
        index++;
    }

    const std::vector<dcollide::Vector3>& texels = textureInformation.getTexels();

    // actual "rendering" code
    {
        int index = 0;
        int triangle = 0;
        for (std::vector<dcollide::Triangle*>::const_iterator it = triangles.begin(); it != triangles.end(); ++it) {
            const dcollide::Vertex* v;
            const dcollide::Vector3* pos;
            const dcollide::Vector3* normal;

            for (int i = 0; i < 3; i++) {

                v = (*it)->getVertices()[i];

                pos = &(v->getPosition());
                normal = (*it)->getNormals()[i];

                object->position(pos->getX(), pos->getY(), pos->getZ());
                object->normal(normal->getX(), normal->getY(), normal->getZ());

                if (useTextures && textureInformation.isTextured()) {
                    if (textureInformation.getTexelSharing()) {
                        std::map<const dcollide::Vertex*, int>::const_iterator vertexIndexIt = vertex2Index.find(v);
                        if (vertexIndexIt == vertex2Index.end()) {
                            throw dcollide::Exception("Triangle vertex not in the getVertices() of the mesh");
                        }
                        int vertexIndex = (*vertexIndexIt).second;
                        if (textureInformation.isTextured()) {
                            dcollide::Vector3 v = texels[vertexIndex];
                            object->textureCoord(v[0], v[1], v[2]);
                        }
                    } else {
                        dcollide::Vector3 v = texels[index];
                        object->textureCoord(v[0], v[1], v[2]);
                    }
                }

                index++;
            }

            triangle++;
        }
    }
}

/*
 * vim: et sw=4 ts=4
 */
