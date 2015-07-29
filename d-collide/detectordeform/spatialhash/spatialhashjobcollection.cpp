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

#include "spatialhashjobcollection.h"

#include "spatialhashalgorithm.h"
#include "spatialgrid.h"
#include "broadphase/broadphasecollisions.h"
#include "shapes/mesh.h"
#include "debugstream.h"

namespace dcollide {

    /*!
     * \brief Creates a new TriangleBatch
     * \param Mesh The mesh the TriangleBatch is part of
     * \param indexStart Start of the continuos region of triangles assigned to this mesh
     * \param indexEnd The end of the continuos region
     * There is no error checking on the parameters, if indexStart > indexEnd, or one of them is
     * out of bounds, the program will behave unpredictably.
     * TriangleBatch doesn't take ownership of the Mesh.
     */
    TriangleBatch::TriangleBatch( const Mesh* Mesh, int indexStart, int indexEnd) 
        : mMesh(Mesh),
          mIndexStart(indexStart),
          mIndexEnd(indexEnd) {
    }

    /*!
     * \brief Creates a new SpatialHashJob
     * \param phaseOne If true, this job will be of type "Phase One", otherwise it will be of type "Phase Two".
     * \param SpatialGrid All jobs work on the same instance of \ref SpatialGrid . This instance must be specified here.
     */
    SpatialHashJob::SpatialHashJob(bool phaseOne, SpatialGrid* spatialGrid) {
        mIsPhaseOne = phaseOne;
        mMeshList = 0;
        mSpatialGrid = spatialGrid;
    }

    /*!
     * Set the data that is required to perform phase one.
     *
     * \param meshList a pointer to a list with all meshes that collision
     * detection is to be performed with. WARNING: the list this pointer points
     * to must remain valid for the whole lifetime of this job!
     */
    void SpatialHashJob::setPhaseOneData(std::list<const Mesh*>* meshList) {
        if (!mIsPhaseOne) {
            throw Exception("Cannot set phase one data to a phase two job");
        }
        mMeshList = meshList;
    }

    /*!
     * \brief Assigns a triangle batch to this job
     * Works only if this job is a phase two job. The invocation of the run
     * method will process all TriangleBatches, which were added with this method.
     */
    void SpatialHashJob::addTriangleBatch(TriangleBatch batch) {
        if (mIsPhaseOne) {
            throw Exception("Cannot add triangle batch to a phase one job");
        }
        mBatchList.push_back(batch);
    }

    /*!
     * \brief Perform the work represented by this job
     * Invocation will be done by the pipeline framework (\ref Pipeline)
     *
     * \li If job is a phase one job, all vertices of all meshes will be inserted into the \ref SpatialGrid
     * \li If job is a phase two job, all TriangleBatches will be processed. After a phase two job has been run, its result may be passed to the pipeline via the \ref SpatialHashJob::processResults method.
     */
    void SpatialHashJob::run() {
        if (mIsPhaseOne) {
            phaseOne();
        } else {
            //Iterate the triangle batch list
            for (std::list<TriangleBatch>::iterator it = mBatchList.begin(); it != mBatchList.end(); ++it) {
                phaseTwo(*it);
            }
        }
    }

    /*!
     * \brief Invocation will insert all vertices, which where assigned via \ref SpatialHashJob::setPhaseOneData into the grid.
     * This is done in several steps: 
     * \li Delete all previously inserted vertices (vertices inserted in the previous frame)
     * \li Calculation of the average side length of \p meshes and adjustion of the unit lenght
     * of the SpatialGrid (see \ref setUnitLength)
     * \li Insertion of the vertices of \p meshes into the \ref SpatialGrid ("Phase 1").
     *
     * After this method is called, collision detection can be performed ("Phase
     * 2").
     */
    void SpatialHashJob::phaseOne() {
        if (!mMeshList) {
            return;
        }
        // * Reset the spatial grid
        // * Set the average side length

        mSpatialGrid->reset();

        
        //Phase 1: Add all vertices to the SpatialGrid
        unsigned int totalVertexCount = 0;
        for (std::list<const Mesh*>::const_iterator meshIt = mMeshList->begin(); meshIt != mMeshList->end(); ++meshIt) {
            const Mesh* m = *meshIt;
            const Proxy* p = m->getProxy();
            const std::vector<Vertex*>& vertices = m->getVertices();
            for (std::vector<Vertex*>::const_iterator vertexIt = vertices.begin(); vertexIt != vertices.end(); ++vertexIt) {
                mSpatialGrid->insert((*vertexIt), p);
                totalVertexCount++;
            }
        }
    }

    

    /*!
     * \brief Invocation of this method will process the triangles of TriangleBatch
     * The idea of phase two is to determine all vertices which are in proximity of the 
     * triangle, we want to check collision with. We calculate the Aabb of the triangle, and fetch
     * all vertices which are inside.
     *
     * Then we will determine the set of adjacent triangles to this vertices, and regard them as
     * potential colliding with the triangle, we check collision with.
     * See \ref SpatialGrid::checkTriangle for more detail.
     */ 
    void SpatialHashJob::phaseTwo(TriangleBatch& b) {
        //Phase 2: check if triangle aabb collide with the previously added vertices
        unsigned int totalTriangleCount = 0;
        bool selfCollisions = false;

        const Mesh* m = b.getMesh();
        const Proxy* p = m->getProxy();

        if( p->getProxyType() & PROXYTYPE_SELFCOLLIDABLE ) {
            selfCollisions = true;
        }

        const std::vector<Triangle*>& triangles = m->getTriangles();

        for(int i = b.getStartIndex(); i <= b.getEndIndex(); i++) {
            mSpatialGrid->checkTriangle(triangles[i], p, selfCollisions, mResults);
            totalTriangleCount++;
        }
    }

    /*!
     * \brief Call this method after this job has finished its work. The results of this job will be added to the pipeline results.
     */
    void SpatialHashJob::processResults(Pipeline* pipeline) {
        pipeline->processAndSpliceMiddlePhaseResults(mResults);
    }


    /*! 
     *\brief The HashJobCollection is a container for all the jobs of the spatial hash, both phase one and phase two jobs
     */
    SpatialHashJobCollection::SpatialHashJobCollection(Pipeline* p, SpatialHashAlgorithm* hash) :
            PipelineThreadJobCollection(p, Pipeline::PHASE_MIDDLEPHASE) {
        mSpatialHashAlgorithm = hash;
        mPipeline = p;
    }

    void SpatialHashJobCollection::allJobsCompleted() {
        // call base implementation
        PipelineThreadJobCollection::allJobsCompleted();

        resetCollection();
    }

    void SpatialHashJobCollection::setJobCompleted(ThreadJob* job_) {
        SpatialHashJob* job = static_cast<SpatialHashJob*>(job_);
        if (job->isPhaseOne()) {
            MutexLocker lock(&mMutex);
            mSpatialHashAlgorithm->createJobsForPhaseTwo();
            setAllJobsAreAdded();
        }

        PipelineThreadJobCollection::setJobCompleted(job);
    }
}
/*
 * vim: et sw=4 ts=4
 */
