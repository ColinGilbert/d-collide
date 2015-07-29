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


#ifndef DCOLLIDE_SPATIAL_HASHJOBCOLLECTION_H
#define DCOLLIDE_SPATIAL_HASHJOBCOLLECTION_H

#include "pipeline.h"
#include "thread/jobpool.h"
#include "broadphase/broadphase.h"
#include "broadphase/broadphasecollisions.h"
#include "detectordeform/trianglepair.h"


namespace dcollide {
    class SpatialHashImplementation;
    class SpatialHashAlgorithm;
    class SpatialGrid;
    class Mesh;

    /*!
     * \brief The TriangleBatch is used in multithreading to assign the workload of huge meshes to several jobs
     * A mesh may be divided into several triangle batches. These triangle batches may be assigned to different jobs. See \ref SpatialHashAlgorithm::createJobsForPhaseTwo , where the splitting is done.
     * A triangle batch does not actually store any triangles, instead it stores start and end markers on
     * the mesh data. A startIndex of 2 and an endIndex of 10 means, that the triangles with the numbers 2-10
     * of a mesh are assigned to this batch. The next triangle batch could hold the triangles 11-20. Both 
     * TriangleBatches could be assigned to different jobs. It is solely possible to assign sets of 
     * triangles with continuos enumeration to a TriangleBatch, thus assigning triangles with the numbers
     * 1,3,9 and 10 to a batch is not allowed.
     * 
     */
    class TriangleBatch {
        private:
            const Mesh* mMesh;
            int   mIndexStart, mIndexEnd;
        public:
            TriangleBatch(const Mesh* Mesh, int indexStart, int indexEnd);
            inline const Mesh* getMesh();
            inline int getStartIndex();
            inline int getEndIndex();
    };

    /*!
     * \brief Returns the mesh this TriangleBatch is part of.
     */
    inline const Mesh* TriangleBatch::getMesh() {
        return mMesh;
    }

    /*!
     * \brief Get the index of the first triangle of this batch.
     */
    inline int TriangleBatch::getStartIndex() {
        return mIndexStart;
    }

    /*!
     * \brief Get the index of the last triangle of this batch.
     */
    inline int TriangleBatch::getEndIndex() {
        return mIndexEnd;
    }

    /*!
     * \brief The SpatialHashJob represents a chunk of work to be done for spatial hashing.
     * As the algorithm is divided into two phases (see \ref SpatialHashAlgorithm) , a 
     * SpatialHashJob may be of type "phaseOne" or of type "phaseTwo".
     *
     * \li If the HashJob is a phase one job then running will perform the update phase of the algorithm;
     * all vertices will be inserted into the grid.
     * \li If the job is a phase two job, then it will process all triangle batches (\ref TriangleBatch ) , 
     * which were assigned to the job
     */
    class SpatialHashJob : public PipelineThreadJob {
        public:
            SpatialHashJob(bool phaseOne, SpatialGrid* grid);

            virtual void run();
            virtual void processResults(Pipeline* pipeline);

            inline bool isPhaseOne() const;

            void setPhaseOneData(std::list<const Mesh*>* meshList);
            void addTriangleBatch(TriangleBatch batch);

        protected:
            void phaseOne();
            void phaseTwo(TriangleBatch& b);

        private:
            SpatialGrid* mSpatialGrid;
            bool mIsPhaseOne;

            // phase one data
            std::list<const Mesh*>* mMeshList;

            // phase two data
            std::list<TriangleBatch> mBatchList;

            // results of this job
            std::list<PotentialCollidingSets> mResults;
    };

    /*!
     * \brief The SpatialHashJobCollection is a container for all the jobs of the SpatialHash
     */
    class SpatialHashJobCollection : public PipelineThreadJobCollection {
        public:
            SpatialHashJobCollection(Pipeline* p, SpatialHashAlgorithm* hash);

            virtual void setJobCompleted(ThreadJob*);

        protected:
            virtual void allJobsCompleted();

        private:
            Mutex mMutex;
            SpatialHashAlgorithm* mSpatialHashAlgorithm;
            Pipeline* mPipeline;
    };

    inline bool SpatialHashJob::isPhaseOne() const {
        return mIsPhaseOne;
    }


}

/*
 * vim: et sw=4 ts=4
 */
#endif

