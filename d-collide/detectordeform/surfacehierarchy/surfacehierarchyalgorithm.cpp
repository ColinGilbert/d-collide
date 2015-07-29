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

#include "surfacehierarchyalgorithm.h"

#include "proxy.h"
#include "world.h"
#include "shapes/shape.h"
#include "real.h"
#include "debugstream.h"
#include "exceptions/exception.h"
#include "deformablebvhnode.h"
#include "shapes/mesh/vertex.h"

#include "surfacehierarchyjob.h"
#include "surfacehierarchyselfjob.h"
#include "surfacehierarchyproxydata.h"

#include "thread/threadjobcollection.h"

#include <collisionpair.h>


#define SURFACEHIERARCHY_DEBUG 1

namespace dcollide {

    SurfaceHierarchyAlgorithm::SurfaceHierarchyAlgorithm(World* world, Pipeline* pipeline)
            : DetectorDeformAlgorithm(pipeline) {
        mWorld = world;
    }

    SurfaceHierarchyAlgorithm::~SurfaceHierarchyAlgorithm() {
    }

    bool SurfaceHierarchyAlgorithm::supportsSelfCollisions() const {
        return true;
    }

    bool SurfaceHierarchyAlgorithm::supportsPairCollisions() const {
        return true;
    }

    bool SurfaceHierarchyAlgorithm::getListenForProxyChanges() const {
        return false;
    }

    /*!
     * \brief Create a SurfaceHierarchy on all deformable proxies
     *
     * This method should be called once (and only once) as soon as the \ref
     * Proxy objects have been added to the \ref World. It will use \ref
     * SurfaceHierarchyCreator on all deformable proxies (see \ref
     * World::getDeformableProxies).
     */
    void SurfaceHierarchyAlgorithm::prepareSimulation() {
        //getting the list of all deformable Proxys from world-class
        const std::list<Proxy*>& allDeformableProxies = mWorld->getDeformableProxies();

        for(std::list<Proxy*>::const_iterator iter = allDeformableProxies.begin();
                iter != allDeformableProxies.end();
                ++iter) {
            if (!((*iter)->getProxyType() & PROXYTYPE_DEFORMABLE)) {
                throw Exception("a Proxy from World::getDeformableProxies() is not deformable");
            }

            // TODO:
            // child proxies!
            // * toplevel proxies may be a container proxy only (i.e.
            //   without a shape) and the actual shapes may be in the children
            // * even if a proxy has a Shape, there may still be child proxies
            //   with shapes too!
            if (!(*iter)->getShape()) {
                continue;
            }
            if (((*iter)->getShape()->getShapeType() == Shape::SHAPE_TYPE_MESH) ||
                    ((*iter)->getShape()->getShapeType() == Shape::SHAPE_TYPE_MESH_PART)) {
                SurfaceHierarchyCreator creator(this, mWorld, *iter);
                creator.generateSurfaceHierarchy();
            }
        }

    }

    void SurfaceHierarchyAlgorithm::createCollisionJobFor(const CollisionPair& pair) {


        Proxy* p1 = pair.bvol1->getHierarchyNode()->getProxy();
        Proxy* p2 = pair.bvol2->getHierarchyNode()->getProxy();

        // check if this is a selfcollidablejob!!
        if (pair.bvol1->getHierarchyNode()->getProxy() == pair.bvol2->getHierarchyNode()->getProxy()) {

            //debug(10) << "creating selfcollisionjob...";

            SurfaceHierarchySelfJob* sh_self_job = new SurfaceHierarchySelfJob(
                    static_cast<SurfaceHierarchyProxyData*>(getProxyDataFor(p1)));

            getJobCollection()->addJob(sh_self_job);


            return;

        }

        //debug(10) << "creating deformablecollisionjob...";


        SurfaceHierarchyJob* sh_job = new SurfaceHierarchyJob(p1, p2, getProxyDataIndex());

        getJobCollection()->addJob(sh_job);

    }


    void SurfaceHierarchyAlgorithm::notifyAllCollisionPairsAreAdded() {
        // AB: we probably don't need to do anything here.
        //     reimplemented only for the sake of completeness (to have a sample
        //     implementation where all required methods are reimplemented)
        DetectorDeformAlgorithm::notifyAllCollisionPairsAreAdded();
    }

    DetectorDeformProxyData* SurfaceHierarchyAlgorithm::createProxyData(Proxy* proxy) {
        return new SurfaceHierarchyProxyData(proxy);
    }
}

/*
 * vim: et sw=4 ts=4
 */
