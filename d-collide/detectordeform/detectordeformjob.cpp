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

#include "detectordeformjob.h"

#include "debugstream.h"
#include "narrowphase/boundingvolumecollision.h" // BoundingVolumeCollision
#include "trianglepair.h"
#include "collisioninfo.h"

namespace dcollide {
    DetectorDeformJob::DetectorDeformJob(ResultType resultType, unsigned int jobPool)
            : PipelineThreadJob(jobPool) {
        mResultsBvhNodes = 0;
        mResultsTriangles = 0;
        mResultsTriangleSets = 0;
        mResultsNarrowPhase = 0;
        switch (resultType) {
            case RESULT_TYPE_CUSTOM:
            default:
                break;
            case RESULT_TYPE_BVH_NODES:
                mResultsBvhNodes = new std::list<BoundingVolumeCollision>();
                break;
            case RESULT_TYPE_TRIANGLES:
                mResultsTriangles = new std::list<TrianglePair>();
                break;
            case RESULT_TYPE_TRIANGLESETS:
                mResultsTriangleSets = new std::list<PotentialCollidingSets>();
                break;
            case RESULT_TYPE_NARROWPHASE:
                mResultsNarrowPhase = new std::list<CollisionInfo>();
                break;
        }
    }

    DetectorDeformJob::~DetectorDeformJob() {
        delete mResultsBvhNodes;
        delete mResultsTriangles;
        delete mResultsTriangleSets;
        delete mResultsNarrowPhase;
    }

    /*!
     * Add one \ref BoundingVolumeCollision to the internal result list.
     *
     * This has an effect only if \p RESULT_TYPE_BVH_NODES was used in the
     * constructor.
     */
    void DetectorDeformJob::addResults(const BoundingVolumeCollision& c) {
        if (mResultsBvhNodes) {
            mResultsBvhNodes->push_back(c);
        }
    }

    /*!
     * Add a list of \ref BoundingVolumeCollision to the internal result list.
     *
     * The list is spliced, i.e. this function works in O(1) and \p list will be
     * empty afterwards.
     *
     * This has an effect only if \p RESULT_TYPE_BVH_NODES was used in the
     * constructor.
     */
    void DetectorDeformJob::addResults(std::list<BoundingVolumeCollision>& list) {
        if (mResultsBvhNodes) {
            mResultsBvhNodes->splice(mResultsBvhNodes->end(), list);
        }
    }

    /*!
     * Add one \ref TrianglePair to the internal result list.
     *
     * This has an effect only if \p RESULT_TYPE_TRIANGLES was used in the
     * constructor.
     */
    void DetectorDeformJob::addResults(const TrianglePair& t) {
        if (mResultsTriangles) {
            mResultsTriangles->push_back(t);
        }
    }

    /*!
     * Add one \ref PotentialCollidingSets to the internal result list.
     *
     * This has an effect only if \p RESULT_TYPE_TRIANGLESETS was used in the
     * constructor.
     */
    void DetectorDeformJob::addResults(const PotentialCollidingSets& set) {
        if (mResultsTriangleSets) {
            mResultsTriangleSets->push_back(set);
        }
    }
    /*!
     * Add a list of \ref TrianglePair to the internal result list.
     *
     * The list is spliced, i.e. this function works in O(1) and \p list will be
     * empty afterwards.
     *
     * This has an effect only if \p RESULT_TYPE_TRIANGLES was used in the
     * constructor.
     */
    void DetectorDeformJob::addResults(std::list<TrianglePair>& list) {
        if (mResultsTriangles) {
            mResultsTriangles->splice(mResultsTriangles->end(), list);
        }
    }

    /*!
     * Add one \ref CollisionInfo to the internal result list.
     *
     * This has an effect only if \p RESULT_TYPE_NARROWPHASE was used in the
     * constructor.
     */
    void DetectorDeformJob::addResults(const CollisionInfo& narrowPhaseResult) {
        if (mResultsNarrowPhase) {
            mResultsNarrowPhase->push_back(narrowPhaseResult);
        }
    }

    /*!
     * Add a list of \ref CollisionInfo to the internal result list.
     *
     * The list is spliced, i.e. this function works in O(1) and \p list will be
     * empty afterwards.
     *
     * This has an effect only if \p RESULT_TYPE_NARROWPHASE was used in the
     * constructor.
     */
    void DetectorDeformJob::addResults(std::list<CollisionInfo>& narrowPhaseResults) {
        if (mResultsNarrowPhase) {
            mResultsNarrowPhase->splice(mResultsNarrowPhase->end(), narrowPhaseResults);
        }
    }

    /*!
     * This implementation of \ref PipelineThreadJob::processResults takes the
     * results that have been added to this job using one of the \ref addResults
     * methods and returns them to the \ref Pipeline.
     *
     * You usually do not need to overwrite this method.
     */
    void DetectorDeformJob::processResults(Pipeline* pipeline) {
        if (mResultsBvhNodes) {
            pipeline->processAndSpliceMiddlePhaseResults(*mResultsBvhNodes);
        }
        if (mResultsTriangles) {
            pipeline->processAndSpliceMiddlePhaseResults(*mResultsTriangles);
        }
        if (mResultsTriangleSets) {
            pipeline->processAndSpliceMiddlePhaseResults(*mResultsTriangleSets);
        }
        if (mResultsNarrowPhase) {
            pipeline->processAndSpliceMiddlePhaseResults(*mResultsNarrowPhase);
        }
    }


}

/*
 * vim: et sw=4 ts=4
 */
