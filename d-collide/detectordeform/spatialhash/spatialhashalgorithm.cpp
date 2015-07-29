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

#include "spatialhashalgorithm.h"

#include "spatialhashjobcollection.h"
#include "spatialhash.h"
#include "spatialgrid.h"
#include "proxy.h"
#include "world.h"
#include "debugstream.h"
#include "debug.h"
#include "debuglog.h"
#include "timing.h"
#include "boundingvolumes/boundingvolume.h"
#include "bvhnode.h"
#include "shapes/shape.h"
#include "shapes/mesh.h"

// TODO: move to defines.h
#define SPATIALHASH_TRIANGLES_PER_THREAD 4000

namespace dcollide {

    /*!
     * \param pipeline The multithreading framework this algorithm is running in
     */
    SpatialHashAlgorithm::SpatialHashAlgorithm(Pipeline* pipeline)
            : DetectorDeformAlgorithm(pipeline) {
       
        mSpatialGrid = new SpatialGrid(1, new SimpleHash(100001) );
        mJobCollection = new SpatialHashJobCollection(pipeline, this);
        mAverageSideLength = 0;

    }

    SpatialHashAlgorithm::~SpatialHashAlgorithm() {
        delete mSpatialGrid;
        delete mJobCollection;
    }

    /*!
     * Self collisions are supported by this algorithm
     */
    bool SpatialHashAlgorithm::supportsSelfCollisions() const {
        return true;
    }

    /*!
     * Pair collisions are supported by this algorithm
     */
    bool SpatialHashAlgorithm::supportsPairCollisions() const {
        return true;
    }

    /*!
     * \return FALSE - this algorithm does not want to receive updates about
     *         \ref Proxy movements.
     */
    bool SpatialHashAlgorithm::getListenForProxyChanges() const {
        return false;
    }

    /*!
     * The spatial hash makes no use of global initialization
     */
    void SpatialHashAlgorithm::prepareSimulation() {
    }

    /*!
     * Accumulate all broadphase results. Phase one will not be started, until we have
     * retrieved all results. This will be notified via 
     * \ref SpatialHashAlgorithm::notifyAllCollisionPairsAreAdded .
     */
    void SpatialHashAlgorithm::createCollisionJobFor(const CollisionPair& pair) {
        mProxies.insert(pair.bvol1->getHierarchyNode()->getProxy());
        mProxies.insert(pair.bvol2->getHierarchyNode()->getProxy());
    }

    /*!
     * \brief Invocation of this method will trigger the job creation of phase one
     * Spatial hashing needs job creation to be delayed after all results have been
     * retrieved from the broadphase. This is needed because the side lenght of the 
     * unit cubes will be aligned to the average side-length of all involved triangles.
     */
    void SpatialHashAlgorithm::notifyAllCollisionPairsAreAdded() {
        // TODO: when collision detection is completed, mProxies needs to be
        // cleared! (same for mMeshes)
        // -> use notifyPipelineCompleted() ?

        for(std::set<Proxy*>::iterator i = mProxies.begin(); i != mProxies.end(); ++i) {
            //Recursively retrieve all submeshes from proxy *i
            std::list<const Mesh*> meshes = getMeshesFromProxy(*i);
            mMeshes.splice(mMeshes.end(), meshes);
        }
   
        if(mAverageSideLength == 0) {
            mAverageSideLength = calculateAverageSideLength(mMeshes);
            mSpatialGrid->setUnitLength(mAverageSideLength);
        }

        SpatialHashJob* pOne = new SpatialHashJob(true, mSpatialGrid);
        pOne->setPhaseOneData(&mMeshes);
        mJobCollection->addJob(pOne);

        // AB: NOT yet setAllJobsAreAdded()
        // -> will be done once phase two has been added
        mJobCollection->start();
    }

    /*!
     * \brief Calculate the average side length of several meshes
     *
     * This method calculates the average side length of \p meshes by first
     * calculating the average side length of each mesh (using \ref
     * Mesh::getAverageSideLength) and then this method weights this value
     * according to the triangle count of the mesh.
     *
     * This method is important because spatial hashing works with maximum performance,
     * when the side length of the unit cubes equals the average side lengths of all polygons
     * involved. You may find a justification of this in "Optimized Spatial 
     * Hashing for Collision Detection of Deformable Objects".
     */
    real SpatialHashAlgorithm::calculateAverageSideLength(const std::list<const Mesh*>& meshes) const {
        real average = 0.0;
        unsigned int triangleCount = 0;
        for (std::list<const Mesh*>::const_iterator it = meshes.begin(); it != meshes.end(); ++it) {
            triangleCount += (*it)->getTriangleCount();
        }
        for (std::list<const Mesh*>::const_iterator it = meshes.begin(); it != meshes.end(); ++it) {
            const Mesh* m = *it;
            real weight = ((real)m->getTriangleCount()) / ((real)(triangleCount));
            average += weight * m->getAverageSideLength();
        }
        return average;
    }
    /*!
     * \brief Reset all data - needs to be called every frame
     */
    void SpatialHashAlgorithm::resetAlgorithm() {
        mProxies.clear();
        mMeshes.clear();
    }

    /*!
     * \brief The returned object represents the workload, that needs to be performed to do collision detection 
     */
    PipelineThreadJobCollection* SpatialHashAlgorithm::getJobCollection() {
        return mJobCollection;
    }

    /*!
     * \brief Generates a mesh for each shape of the proxy and returns them in a flat list
     * Travels the hierarchy of the given proxy and returns all its components
     * in a flat list. The mesh is retrieved via the getMesh() method of the
     * associated shape, so eventually for non-mesh types mesh representations
     * are created.
     */
    std::list<const Mesh*> SpatialHashAlgorithm::getMeshesFromProxy(const Proxy* p) {
        std::list<const Mesh*> meshList;

        //Safety check for valid shape and p references
        if (!p) {
            return meshList;
		}
		
		if (p->getShape()) {
            //Add current shape to list
        	meshList.push_back(p->getShape()->getMesh());
        }

        for(std::list<Proxy*>::const_iterator i = p->getChildProxies().begin(); i != p->getChildProxies().end(); i++ ) {
            std::list<const Mesh*> childmeshes = getMeshesFromProxy((*i));

            meshList.merge(childmeshes);
        }

        return meshList;
    }

    /*!
     * \brief Phase two is actually multithreaded. This method is responsible to split the workload evenly across \ref SpatialHashJobs .
     * The SPATIALHASH_TRIANGLES_PER_THREAD define will influence the number of jobs created in phase two.
     * If it is set to 4000, this method will assign 4000 triangles to each job. If the whole scene has less than
     * 4000 triangles, one job with less than 4000 triangles will be created.
     *
     * The method has to take into account that triangles are not given in a flat list, but given in a 
     * flat list per mesh. So this method specifies not only which triangles a job should work on, but 
     * which meshes also. This is what the class \ref TriangleBatch is for.
     *
     * Lets have a look at a sample scene, consisting of three meshes:\n
     * mesh 1: 500 triangles  \n
     * mesh 2: 4500 triangles \n
     * mesh 3: 1500 triangles \n
     * For this scenario the method will create 2 Jobs, because the total number is less than 8000 triangles.\n
     * The following triangle batches will be created:\n
     * batch 1:   ( mesh1, startIndex:1, endIndex:500 ) -> assigned to job 1 \n
     * batch 2:   ( mesh2, startIndex:1, endIndex:3500) -> assigned to job 1 \n
     * So we have a total of 4000 triangles for the first job. \n
     * batch 3:   ( mesh2, startIndex:3501, endIndex:4000 ) -> assigned to job 2 \n
     * batch 4:   ( mesh3, startIndex:1, endIndex: 1500 ) -> assigned to job 2 \n
     * So we have a total of 1999 Triangles for the second job, and have no more triangles left
     * to assign.
     */
    void SpatialHashAlgorithm::createJobsForPhaseTwo() {
        /*
         *Precondition of loop:
         *Create a new HashJob, which will take a number of triangles
         *specified in SPATIALHASH_TRIANGLES_PER_THREAD
         */
        int batchSize = SPATIALHASH_TRIANGLES_PER_THREAD;

        SpatialHashJob* newJob = new SpatialHashJob(false, mSpatialGrid);

        for(std::list<const Mesh*>::iterator i = mMeshes.begin(); i != mMeshes.end(); ++i) {
            int remainingTriangles = (*i)->getTriangleCount();
            int index = 0;
            while( remainingTriangles > batchSize ) {
                newJob->addTriangleBatch(TriangleBatch((*i),
                                         index, 
                                         index + batchSize -1
                                        ));
                mJobCollection->addJob(newJob);
                //Advance index
                index += batchSize;
                remainingTriangles -= batchSize;
                //Create a new job and set batchsize accordingly
                newJob = new SpatialHashJob(false, mSpatialGrid);
                batchSize = SPATIALHASH_TRIANGLES_PER_THREAD;
            }
            //RemainingTriangles < BatchSize
            //Write remaining Triangles, and fill the rest of the buffer
            //with triangles from the next mesh
            newJob->addTriangleBatch(TriangleBatch((*i),
                                         index, 
                                         index + remainingTriangles - 1
                                        ));

            //We allocted #remainingTriangles to this batch, so we have to
            //correct the number of free triangles
            batchSize -= remainingTriangles;
        }
        //Add last job
        mJobCollection->addJob(newJob);

        mJobCollection->setAllJobsAreAdded();
    }
}

/*
 * vim: et sw=4 ts=4
 */
