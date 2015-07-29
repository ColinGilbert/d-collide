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

#include "bvhierarchyalgorithm.h"

#include "debugstream.h"
#include "exceptions/exception.h"
#include "world.h"
#include "proxy.h"
#include "shapes/shape.h"
#include "shapes/mesh.h"
#include "meshsplitter.h"
#include "bvhierarchybvhnode.h"
#include "bvhierarchyjob.h"
#include "bvhierarchyproxydata.h"
#include "collisionpair.h"
#include "math/matrix.h"

namespace dcollide {

    class BvHierarchySplitter : public DefaultMeshSplitter {
        public:
            BvHierarchySplitter(World* world, Mesh* mesh, BvhNode* topBvhNode);

        protected:
            virtual BvhNode* createBvhNode(MeshPart* part) const;
    };

    BvHierarchySplitter::BvHierarchySplitter(World* world, Mesh* mesh, BvhNode* topBvhNode)
            : DefaultMeshSplitter(world, mesh, topBvhNode) {
    }

    BvhNode* BvHierarchySplitter::createBvhNode(MeshPart* part) const {
        BvhNode* node = new BvHierarchyBvhNode(getWorld(), part);
        node->initializeNode();
        return node;
    }



    BvHierarchyAlgorithm::BvHierarchyAlgorithm(World* world, Pipeline* pipeline)
            : DetectorDeformAlgorithm(pipeline) {
        mWorld = world;
    }

    BvHierarchyAlgorithm::~BvHierarchyAlgorithm() {
    }

    // TODO: make this the default implementation in DetectorDeformAlgorithm!
    // -> make addTopLevelProxy() pure virtual then!
    void BvHierarchyAlgorithm::prepareSimulation() {
        const std::list<Proxy*>& deformableProxies = mWorld->getDeformableProxies();
        for (std::list<Proxy*>::const_iterator it = deformableProxies.begin(); it != deformableProxies.end(); ++it) {
            addTopLevelProxy(*it);
        }
    }

    DetectorDeformProxyData* BvHierarchyAlgorithm::createProxyData(Proxy* proxy) {
        return new BvHierarchyProxyData(proxy);
    }

    void BvHierarchyAlgorithm::createCollisionJobFor(const CollisionPair& pair) {
        BvHierarchyJob* job = new BvHierarchyJob();
        job->setProxyDataIndex(getProxyDataIndex());
        job->addInput(pair);
        getJobCollection()->addJob(job);
    }

    void BvHierarchyAlgorithm::notifyAllCollisionPairsAreAdded() {
        // AB: we probably don't need to do anything here.
        DetectorDeformAlgorithm::notifyAllCollisionPairsAreAdded();
    }

    void BvHierarchyAlgorithm::addTopLevelProxy(Proxy* proxy) {
        std::list<Proxy*> queue;
        queue.push_back(proxy);
        while (!queue.empty()) {
            Proxy* p = queue.front();
            queue.pop_front();

            for (std::list<Proxy*>::const_iterator it = p->getChildProxies().begin(); it != p->getChildProxies().end(); ++it) {
                queue.push_back(*it);
            }

            if (!p->getShape()) {
                continue;
            }
            if (p->getShape()->getShapeType() != Shape::SHAPE_TYPE_MESH) {
                throw Exception("Deformable proxy has non-mesh shape - this is not allowed!");
            }

            initMesh(p, static_cast<Mesh*>(p->getShape()));
        }
    }

    void BvHierarchyAlgorithm::removeTopLevelProxy(Proxy* proxy) {
        DCOLLIDE_UNUSED(proxy);
        // nothing to do: the ProxyData is deleted automatically
    }

    void BvHierarchyAlgorithm::initMesh(Proxy* parent, Mesh* mesh) {
        if (!parent) {
            throw NullPointerException("parent");
        }
        if (!mesh) {
            throw NullPointerException("mesh");
        }

        BvHierarchyProxyData* data = static_cast<BvHierarchyProxyData*>(parent->getDetectorDeformProxyData(getProxyDataIndex()));
        if (!data) {
            throw NullPointerException("data");
        }
        BvHierarchyBvhNode* node = new BvHierarchyBvhNode(mWorld, mesh);
        node->initializeNode();
        BvHierarchySplitter splitter(mWorld, mesh, node);
        splitter.startSplitting();

        // AB: parent->getBvHierarchyNode() already adjusts to the Shape.
        //     -> the toplevel node therefore already works using "top-down"
        //        updates.
        //        consequently, we dont need to do any update in our own,
        //        custom, toplevel node.
        if (!node->getChildren().empty()) {
            node->setIgnoreShape(true);
        }

        if (data->mDeformableNode) {
            throw Exception("data->mDeformableNode is not NULL");
        }
        data->mDeformableNode = node;

        const int maxDepth = 3;
        setTopDown(node, 1, maxDepth);

        node->reinitializeChildren();
    }

    void BvHierarchyAlgorithm::setTopDown(BvHierarchyBvhNode* node, int depth, int maxDepth) {
        node->setIsTopDown(true);
        if (depth + 1 > maxDepth) {
            return;
        }
        for (std::list<BvHierarchyBvhNode*>::const_iterator it = node->getBvHerarchyBvhNodeChildren().begin(); it != node->getBvHerarchyBvhNodeChildren().end(); ++it) {
            setTopDown((*it), depth + 1, maxDepth);
        }
    }

    void BvHierarchyAlgorithm::translateProxy(Proxy* proxy, const Vector3& translateBy) {
        if (!proxy) {
            throw NullPointerException("proxy");
        }

        BvHierarchyProxyData* data = static_cast<BvHierarchyProxyData*>(proxy->getDetectorDeformProxyData(getProxyDataIndex()));
        if (!data) {
            throw NullPointerException("data");
        }
        data->mDeformableNode->translate(translateBy);
    }

    void BvHierarchyAlgorithm::rotateProxy(Proxy* proxy, const Matrix& rotation) {
        if (!proxy) {
            throw NullPointerException("proxy");
        }

        BvHierarchyProxyData* data = static_cast<BvHierarchyProxyData*>(proxy->getDetectorDeformProxyData(getProxyDataIndex()));
        if (!data) {
            throw NullPointerException("data");
        }

        DCOLLIDE_UNUSED(rotation);
        data->mDeformableNode->rotate(/*rotation*/);
    }

    void BvHierarchyAlgorithm::setProxyMatrix(Proxy* proxy, const Matrix& newMatrix) {
         if (!proxy) {
            throw NullPointerException("proxy");
        }

        BvHierarchyProxyData* data = static_cast<BvHierarchyProxyData*>(proxy->getDetectorDeformProxyData(getProxyDataIndex()));
        if (!data) {
            throw NullPointerException("data");
        }

        DCOLLIDE_UNUSED(newMatrix);
        data->mDeformableNode->changeMatrix(/*newMatrix*/);
   }

    void BvHierarchyAlgorithm::deformProxy(Proxy* proxy, const std::vector<Vector3>& vertexMoveArray) {
        if (!proxy) {
            throw NullPointerException("proxy");
        }

        BvHierarchyProxyData* data = static_cast<BvHierarchyProxyData*>(proxy->getDetectorDeformProxyData(getProxyDataIndex()));
        if (!data) {
            throw NullPointerException("data");
        }

        DCOLLIDE_UNUSED(vertexMoveArray);
        data->mDeformableNode->deform(/*vertexMoveArray*/);
    }

    void BvHierarchyAlgorithm::deformProxy(Proxy* proxy, unsigned int vertexIndex, const Vector3& vertexPosition) {
        if (!proxy) {
            throw NullPointerException("proxy");
        }

        BvHierarchyProxyData* data = static_cast<BvHierarchyProxyData*>(proxy->getDetectorDeformProxyData(getProxyDataIndex()));
        if (!data) {
            throw NullPointerException("data");
        }

        DCOLLIDE_UNUSED(vertexIndex);
        DCOLLIDE_UNUSED(vertexPosition);
        data->mDeformableNode->deform(/*vertexMoveArray*/);
    }
}

/*
 * vim: et sw=4 ts=4
 */
