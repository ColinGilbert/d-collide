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

#include "boundingvolume.h"
#include "aabb.h"
#include "obb.h"
#include "kdop.h"
#include "boundingsphere.h"
#include "dcollide-defines.h"
#include "world.h"
#include "proxy.h"
#include "shapes/shapes.h"
#include "debug.h"

#include <cstdlib>

namespace dcollide {
    BoundingVolume::~BoundingVolume() {
    }

    /*!
     * \brief Create a new BoundingVolume object.
     *
     * This method creates and returns a new BoundingVolume object, according to
     * the preferred BoundingVolume-type settings.
     *
     * \param parent The BvhNode that the BoundingVolume should be in. See \ref
     * setHierarchyNode.
     *
     * \return A new BoundingVolume object. The caller is responsible for
     * deleting it.
     */
    BoundingVolume* BoundingVolume::createBoundingVolume(World* world, BvhNode* parent) {
        if (!world) {
            throw NullPointerException("Parameter world");
        }
        BoundingVolume* bv = 0;
        switch (getCreateRigidBoundingVolumeType(world)) {
            case BV_TYPE_AABB:
                bv = new Aabb();
                break;
            case BV_TYPE_KDOP:
                bv = new Kdop();
                break;
            case BV_TYPE_SPHERE:
                bv = new BoundingSphere();
                break;
            case BV_TYPE_OBB:
                bv = new Obb();
                break;
            default:
                // TODO: exception!!
                std::cerr << dc_funcinfo << "FATAL ERROR: bounding volume type " << getCreateRigidBoundingVolumeType(world) << " not supported" << std::endl;
                exit(1);
                return 0;
        }
        bv->setHierarchyNode(parent);
        return bv;
    }

    /*!
     * \overload
     *
     * This method creates a new BoundingVolume as a copy of \p copy.
     */
    BoundingVolume* BoundingVolume::createBoundingVolume(World* world, const BoundingVolume* copy, BvhNode* parent) {
        if (!copy) {
            return createBoundingVolume(world, parent);
        }

        if (copy->getVolumeType() != getCreateRigidBoundingVolumeType(world)) {
            std::stringstream errormessage;
            errormessage << "ERROR: requested a copy of BV type "
                    << copy->getVolumeType()
                    << ", but expected type " << getCreateRigidBoundingVolumeType(world)
                    << std::endl;
            throw TypeMismatchException(errormessage.str());
        }

        BoundingVolume* bv = 0;
        switch (getCreateRigidBoundingVolumeType(world)) {
            case BV_TYPE_AABB:
                bv = new Aabb(*static_cast<const Aabb*>(copy));
                break;
            case BV_TYPE_KDOP:
                bv = new Kdop(*static_cast<const Kdop*>(copy));
                break;
            case BV_TYPE_SPHERE:
                bv = new BoundingSphere(*static_cast<const BoundingSphere*>(copy));
                break;
            case BV_TYPE_OBB:
                bv = new Obb(*static_cast<const Obb*>(copy));
                break;
            default:
                // TODO: exception!!
                std::cerr << dc_funcinfo << "FATAL ERROR: bounding volume type " << getCreateRigidBoundingVolumeType(world) << " not supported" << std::endl;
                exit(1);
                return 0;
        }
        bv->setHierarchyNode(parent);
        return bv;
    }

    /*!
     * Convenience method for \ref WorldParameters::getRigidBoundingVolumeType
     */
    BoundingVolumeType BoundingVolume::getCreateRigidBoundingVolumeType(World* world) {
        return world->getWorldParameters().getRigidBoundingVolumeType();
    }

    /*!
     * \brief adjusts this bounding volume to an given shape
     *        (top-down processing)
     */
    void BoundingVolume::adjustToShape(const Shape* shape) {
        if (!shape) {
            throw NullPointerException("Parameter \"const Shape* shape\"");
        }
        if (!shape->getProxy()) {
            throw NullPointerException("shape->getProxy()");
        }
        Matrix worldState(shape->getProxy()->getWorldTransformation());
        switch(shape->getShapeType()) {
            case Shape::SHAPE_TYPE_BOX:
                    adjustToBox(&worldState,
                                 static_cast<const Box*>(shape));
                    break;

            case Shape::SHAPE_TYPE_CONE:
                    adjustToCone(&worldState,
                                  static_cast<const Cone*>(shape));
                    break;

            case Shape::SHAPE_TYPE_CYLINDER:
                    adjustToCylinder(&worldState,
                                      static_cast<const Cylinder*>(shape));
                    break;

            case Shape::SHAPE_TYPE_MESH:
                    adjustToMesh(static_cast<const Mesh*>(shape));
                    break;

            case Shape::SHAPE_TYPE_MESH_PART:
                    adjustToMeshPart(static_cast<const MeshPart*>(shape));
                    break;

            case Shape::SHAPE_TYPE_SPHERE:
                    adjustToSphere(&worldState,
                                    static_cast<const Sphere*>(shape));
                    break;

            case Shape::SHAPE_TYPE_WEDGE:
                    adjustToWedge(&worldState,
                                   static_cast<const Wedge*>(shape));
                    break;

            default:
                    throw UnsupportedShapeException(
                        "Cannot adjust Aabb to unknown Shape");
        }
    }

    std::ostream& operator<<(std::ostream& os, const dcollide::BoundingVolume& v) {
        os << "[" << v.getVolumeType() << ": "
                  << v.getSurroundingAabbMin() << ", "
                  << v.getSurroundingAabbMax()
           << "]";
        return os;
    }

    /*!
     * Make this BoundingVolume contain exactly the \p mesh.
     *
     * The default implementation uses \ref reset
     */
    void BoundingVolume::adjustToMesh(const Mesh* mesh) {
        if (getVolumeType() == BV_TYPE_OBB) {
            adjustToMesh(mesh);
        } else {
            reset(mesh->getVertices());
        }
    }

    /*!
     * Make this BoundingVolume contain exactly the \p meshPart.
     *
     * The default implementation uses \ref reset
     */
    void BoundingVolume::adjustToMeshPart(const MeshPart* meshPart) {
        if (getVolumeType() == BV_TYPE_OBB) {
            adjustToMeshPart(meshPart);
        } else {
            reset(meshPart->getVertices());
        }
    }


    /*!
     * This method calculates an approximation of \p cone and calls \ref reset
     * on that approximation.
     *
     * This method can be used in the \ref adjustoToCone implementation in
     * derived classes, if not better solution is required or known.
     *
     * Note that this may be as bad as calculating an axis-aligned bounding box
     * around \p cone, even if the \ref BoundingVolume type used is something
     * totally diffent and potentially much smaller! Specialized adjust
     * functions should be prefered over this one if possible!
     */
    void BoundingVolume::resetToApproximatedAabbOfCone(const Matrix* worldState, const Cone* cone) {
        // AB: we don't actually need the _minimal_ AABB. we can live with an
        //     approximation:
        //     1. calculate AABB around the untransformed Cone
        //     2. transform all points of the AABB by worldState
        //     3. calculate an AABB of the tranformed points
        const real radius = cone->getRadius();
        const real height = cone->getHeight();
        Vector3 untransformedMin = Vector3(-radius, -radius, 0.0);
        Vector3 untransformedMax = Vector3(radius, radius, height);
        std::list<Vector3> untransformedPoints; // AABB of untransformed Cone
        untransformedPoints.push_back(untransformedMin);
        untransformedPoints.push_back(Vector3(untransformedMin.getX(), untransformedMin.getY(), untransformedMax.getZ()));
        untransformedPoints.push_back(Vector3(untransformedMin.getX(), untransformedMax.getY(), untransformedMin.getZ()));
        untransformedPoints.push_back(Vector3(untransformedMin.getX(), untransformedMax.getY(), untransformedMax.getZ()));
        untransformedPoints.push_back(Vector3(untransformedMax.getX(), untransformedMin.getY(), untransformedMin.getZ()));
        untransformedPoints.push_back(Vector3(untransformedMax.getX(), untransformedMin.getY(), untransformedMax.getZ()));
        untransformedPoints.push_back(Vector3(untransformedMax.getX(), untransformedMax.getY(), untransformedMin.getZ()));
        untransformedPoints.push_back(untransformedMax);

        std::list<Vector3> points;
        for (std::list<Vector3>::const_iterator it = untransformedPoints.begin(); it != untransformedPoints.end(); ++it) {
            Vector3 p;
            worldState->transform(&p, *it);
            points.push_back(p);
        }

        reset(points);
    }

    /*!
     * This method is similar to \ref resetToApproximatedAabbOfCone, however an
     * approximation of a \ref Cylinder is calculated instead of a \ref Cone.
     */
    void BoundingVolume::resetToApproximatedAabbOfCylinder(const Matrix* worldState,
                                const Cylinder* cylinder) {
        // AB: we don't actually need the _minimal_ AABB. we can live with an
        //     approximation:
        //     1. calculate AABB around the untransformed Cylinder
        //     2. transform all points of the AABB by worldState
        //     3. calculate an AABB of the tranformed points
        const real radius = cylinder->getRadius();
        const real height = cylinder->getHeight();
        Vector3 untransformedMin = Vector3(-radius, -radius, 0.0);
        Vector3 untransformedMax = Vector3(radius, radius, height);
        std::list<Vector3> untransformedPoints; // AABB of untransformed Cylinder
        untransformedPoints.push_back(untransformedMin);
        untransformedPoints.push_back(Vector3(untransformedMin.getX(), untransformedMin.getY(), untransformedMax.getZ()));
        untransformedPoints.push_back(Vector3(untransformedMin.getX(), untransformedMax.getY(), untransformedMin.getZ()));
        untransformedPoints.push_back(Vector3(untransformedMin.getX(), untransformedMax.getY(), untransformedMax.getZ()));
        untransformedPoints.push_back(Vector3(untransformedMax.getX(), untransformedMin.getY(), untransformedMin.getZ()));
        untransformedPoints.push_back(Vector3(untransformedMax.getX(), untransformedMin.getY(), untransformedMax.getZ()));
        untransformedPoints.push_back(Vector3(untransformedMax.getX(), untransformedMax.getY(), untransformedMin.getZ()));
        untransformedPoints.push_back(untransformedMax);

        std::list<Vector3> points;
        for (std::list<Vector3>::const_iterator it = untransformedPoints.begin(); it != untransformedPoints.end(); ++it) {
            Vector3 p;
            worldState->transform(&p, *it);
            points.push_back(p);
        }

        reset(points);
    }

}

/*
 * vim: et sw=4 ts=4
 */
