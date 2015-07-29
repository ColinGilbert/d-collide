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

#ifndef DCOLLIDE_DETECTOR_DEFORM_JOB_H
#define DCOLLIDE_DETECTOR_DEFORM_JOB_H

#include "pipeline.h"

#include <list>

namespace dcollide {
    class Proxy;
    struct BoundingVolumeCollision;
    struct TrianglePair;
    struct CollisionInfo;

    /*!
     * \brief Base class for \ref PipelineThreadJob objects of \ref
     * DetectorDeformAlgorithm derived classes
     *
     * This class is meant to simplify the creation of \ref PipelineThreadJob
     * for a \ref DetectorDeformAlgorithm derived class.
     *
     * Different deformable algorithms may have totally different result types,
     * this class helps at returning them to the \ref Pipeline which possibly
     * creates \ref NarrowPhaseJob objects from them. The currently supported
     * result types are listed in \ref ResultType.
     *
     * A derived class should usually simply reimplement \ref run and call the
     * appropriate \ref addResults method whenever it has calculated some of its
     * results. The \ref Pipeline takes care of calling \ref processResults,
     * which is already implemented to return all results added using \ref
     * addResults to the \ref Pipeline.
     */
    class DetectorDeformJob : public PipelineThreadJob {
        public:
            enum ResultType {
                /*!
                 * Indicates that this job returns pairs of \ref BvhNode, such
                 * as \ref DeformableBvhNode. The \ref NarrowPhase is meant to
                 * retrieve the triangles from both nodes of the pair and
                 * intersect them with each other.
                 *
                 * See also \ref addResults
                 */
                RESULT_TYPE_BVH_NODES,

                /*!
                 * Indicates that this job returns pairs of \ref Triangle. The
                 * \ref NarrowPhase is meant to intersect both triangles of the
                 * pair with each other.
                 *
                 * See also \ref addResults
                 */
                RESULT_TYPE_TRIANGLES,

                /*!
                 * Indicates that this job returns \ref PotentialCollidingSets
                 * \ref NarrowPhase is meant to use TriangleSetIntersector
                 *
                 * See also \ref addResults
                 */
                RESULT_TYPE_TRIANGLESETS,

                /*!
                 * Inidcates that this job returns actual narrowphase results,
                 * i.e. the \ref NarrowPhase does not have to process these
                 * results.
                 *
                 * See also \ref addResults
                 */
                RESULT_TYPE_NARROWPHASE,

                /*!
                 * Not automatic result processing. You need to reimplement \ref
                 * processResults to return your results.
                 */
                RESULT_TYPE_CUSTOM
            };
        public:
            DetectorDeformJob(ResultType resultType, unsigned int jobPool = 0);
            virtual ~DetectorDeformJob();

            virtual void processResults(Pipeline* pipeline);

        protected:
            void addResults(const BoundingVolumeCollision& c);
            void addResults(std::list<BoundingVolumeCollision>& list);

            void addResults(const TrianglePair& t);
            void addResults(std::list<TrianglePair>& list);

            void addResults(const PotentialCollidingSets& set);

            
            void addResults(const CollisionInfo& narrowPhaseResult);
            void addResults(std::list<CollisionInfo>& narrowPhaseResults);

        private:
            std::list<BoundingVolumeCollision>* mResultsBvhNodes;
            std::list<TrianglePair>* mResultsTriangles;//TODO remove these
            std::list<PotentialCollidingSets>* mResultsTriangleSets;
            std::list<CollisionInfo>* mResultsNarrowPhase;
    };
}

#endif
/*
 * vim: et sw=4 ts=4
 */
