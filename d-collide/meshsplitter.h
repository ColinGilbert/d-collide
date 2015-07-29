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
       notice, this list of conditions and the following disclaimer.           *
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

#ifndef DCOLLIDE_MESHSPLITTER_H
#define DCOLLIDE_MESHSPLITTER_H

#include "datatypes/list.h"

#include <utility>
#include <vector>
#include <list>

namespace dcollide {
    class World;
    class BvhNode;
    class Mesh;
    class MeshPart;
    class Vertex;
    class MeshSplitter;

    /**
     * \brief Class that creates \ref MeshSplitter object.
     *
     * This class is used by \ref Proxy to create a new \ref MeshSplitter object
     * when a large mesh needs to be split into parts.
     *
     * The default implementation simply creates \ref DefaultMeshSplitter
     * objects. You may derive this class and reimplement \ref
     * createMeshSplitter to change this behaviour.
     *
     * If you derive this class, you need to:
     * \li reimplement \ref createMeshSplitter
     * \li reimplement \ref clone
     * \li give an object of the derived class to \ref
     *     WorldParameters::setMeshSplitterFactory before creating the \ref
     *     World object
     */
    class MeshSplitterFactory {
        public:
            MeshSplitterFactory();
            virtual ~MeshSplitterFactory();

            virtual MeshSplitter* createMeshSplitter(World* world, Mesh* mesh, BvhNode* topBvhNode);

            // AB: this method must be reimplemented to make
            // WorldParameters::operator=() behave correctly (without segfault).
            // a reference counting approach would be nicer, but a lot more
            // work.
            virtual MeshSplitterFactory* clone();
    };

    // note: we always create the hierarchy top-down here!
    //       -> bottom-up would probably possible, but splitMesh() would need
    //          support for that.
    //          also note that both, splitMesh() and calculateSplit() must
    //          either use bottom-up or top-down, mixing is not possible.
    /**
     * \brief Class that splits \ref Mesh objects into \ref MeshPart objects
     *
     * Abstract helper class to \ref Proxy and \ref BvhNode.
     *
     * This class takes a \ref Mesh or \ref MeshPart object and uses some
     * criterium to split it up into \ref MeshPart objects. This is in
     * particular useful for \ref BoundingVolume hierarchy creation, every new
     * split-level is a new level in the hierarchy.
     *
     * Derived classes should implement \ref calculateSplit to do the actual
     * splitting.
     *
     * Note that if you derive from this class, you will also need to
     * reimplement \ref MeshSplitterFactory to create an object from your class
     * and call \ref WorldParameters::setMeshSplitterFactory to make use of your
     * new factory.
     */
    class MeshSplitter {
        public:
            MeshSplitter(World* world, Mesh* mesh, BvhNode* topBvhNode);
            virtual ~MeshSplitter();

            virtual void startSplitting();

        protected:
            void splitMesh(BvhNode* parentNode, MeshPart* meshPart);

            /*!
             * This method is called by \ref startSplitting to do the actual
             * splitting.
             *
             * See \ref splitMesh for further information.
             */
            virtual std::list<MeshPart*> calculateSplit(BvhNode* parentNode, MeshPart* parent) = 0;

            virtual BvhNode* createBvhNode(MeshPart* meshPart) const;

            World* getWorld() const;
            Mesh* getMesh() const;
            BvhNode* getTopLevelBvhNode() const;

        private:
            World* mWorld;
            Mesh* mMesh;
            BvhNode* mTopLevelBvhNode;
            unsigned int mMaxMeshPartTriangles;
    };


    class DefaultMeshSplitter : public MeshSplitter {
        public:
            DefaultMeshSplitter(World* world, Mesh* mesh, BvhNode* topBvhNode);
            virtual ~DefaultMeshSplitter();

            virtual void startSplitting();

        protected:
            struct MyVertex {
                inline explicit MyVertex(Vertex* v);
                inline MyVertex();

                inline void reset();

                Vertex* vertex;
                int xMin;
                int xMax;
                int yMin;
                int yMax;
                int zMin;
                int zMax;
                bool used;
            };

        protected:
            virtual std::list<MeshPart*> calculateSplit(BvhNode* parentNode, MeshPart* parent);

            void makeVertices(std::vector<Vertex*>* vertices, const List<Vertex*>& list);

        private:
            std::vector<MyVertex> mVertices;
            List<MyVertex*> mUsedVertices;
            List<Vertex*> mXMinVertices;
            List<Vertex*> mXMaxVertices;
            List<Vertex*> mYMinVertices;
            List<Vertex*> mYMaxVertices;
            List<Vertex*> mZMinVertices;
            List<Vertex*> mZMaxVertices;
    };

    inline DefaultMeshSplitter::MyVertex::MyVertex(Vertex* v) {
        vertex = v;
        reset();
    }
    inline DefaultMeshSplitter::MyVertex::MyVertex() {
        vertex = 0;
        reset();
    }
    inline void DefaultMeshSplitter::MyVertex::reset() {
        xMin = 0;
        xMax = 0;
        yMin = 0;
        yMax = 0;
        zMin = 0;
        zMax = 0;
        used = false;
    }
}

#endif
/*
 * vim: et sw=4 ts=4
 */
