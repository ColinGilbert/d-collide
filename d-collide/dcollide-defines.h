/*******************************************************************************
 *  Copyright (C) 2007 by the members of PG 510, University of Dortmund:       *
 *              d-collide-devel@lists.sourceforge.net                          *
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


#ifndef DCOLLIDE_DCOLLIDE_DEFINES_H
#define DCOLLIDE_DCOLLIDE_DEFINES_H

// Defines if we use AABBs or KDOPS
// for the BoundingVolume Hierarchy of rigid proxies
//
// define to use AABBs, undefine to use k-DOPs
// NOTE: this define is kind of obsolete: you can use
// BoundingVolume::setCreateBoundingVolumeType() to change the default BV type.
// this define now only sets the default value used by
// setCreateBoundingVolumeType().
#define USE_AABB 1

/*! \brief Limit for the number of children for a BvhNode
 * Since the traverse-algorithm in detectorrigid does pairwise
 * checks for collisions of children, it makes sense to limit th
 * number of children allowed for a node.
 * If the node gets its MAX_CHILDREN_COUNT+1-th child, the hierarchy
 * will be updated by generating an extra layer of hierarchy to
 * lessen the number of children again
 */
#define MAX_BVHNODE_CHILDREN_COUNT 5

// FIXME: this value is kind of obsolete. the value in WorldParameters should be
// used only instead.
/*! \brief Limit for the number of triangles in a BvhNode leaf
 *
 * This is the default value only - dont change this. Use
 * WorldParameters::setRigidMaxMeshPartTriangles() instead!
 */
#define MAX_BVHNODE_LEAF_TRIANGLES 4


/*! \brief Default narrowphase strategies (1-fast to 4-as accurate as possible)
 * see \ref NarrowPhase::Strategy
 */
#define NARROWPHASE_DEFAULT_STRATEGY_SPHERE_SPHERE 2
#define NARROWPHASE_DEFAULT_STRATEGY_SPHERE_MESH 1
#define NARROWPHASE_DEFAULT_STRATEGY_MESH_MESH 2
#define NARROWPHASE_DEFAULT_STRATEGY_BOX_BOX 2
#define NARROWPHASE_DEFAULT_STRATEGY_BOX_SPHERE 2

#endif //DCOLLIDE_DCOLLIDE_DEFINES_H
