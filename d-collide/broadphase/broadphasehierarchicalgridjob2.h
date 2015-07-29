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

#ifndef DCOLLIDE_BROADPHASEHIERARCHICALGRIDJOB2_H
#define DCOLLIDE_BROADPHASEHIERARCHICALGRIDJOB2_H

#include "broadphasejob.h"

#include <set>

namespace dcollide {
    class BroadPhaseHierarchicalGrid;
    class Proxy;
    class HierarchicalGrid;

    // AB: alternative BroadPhaseHierarchicalGridJob. instead of going down the
    // tree and returning all pairs at the leafs, this jobs goes through all
    // proxies and checks all HierarchicalGrid objects that proxy is in for
    // other proxies
    //
    // the complexity of both approaches is exactly the same, but this version
    // may be more efficient for multithreading:
    // - we need to check for duplicates inside the outer loop only, i.e.:
    //   for each proxy p1:
    //     list local_set;
    //     for each proxy p2 in the same HierarchicalGrid:
    //       if (p2 is not in local_set)
    //          local_set.push_back(p2)
    //   --> the local_set is obviously much smaller this way than it would be
    //       if it had to be outside the first for loop (as it is the case for
    //       BroadPhaseHierarchicalGridJob).
    //   --> since the local_set is local, it does NOT have to be protected by a
    //       mutex
    // - for n proxies we can start m jobs (with n > m) by adding n/m proxies to
    //   every job.
    class BroadPhaseHierarchicalGridJob2 : public BroadPhaseJob {
        public:
            BroadPhaseHierarchicalGridJob2(unsigned int intendedJobPoolIndex,
                    BroadPhaseHierarchicalGrid* bp);
            ~BroadPhaseHierarchicalGridJob2();

            inline void setSingleProxyOnly(bool singleProxyOnly);
            void addProxies(const std::list<Proxy*>& proxies,
std::list<Proxy*>::const_iterator& it, int maxCount);
            inline void addProxy(Proxy* proxy);

            void clear();

        protected:
            virtual void run();
            void checkProxyForCollisions(Proxy* proxy);

        private:
            List<Proxy*> mProxies;
            bool mSingleProxyOnly;

            // FIXME: measure construction time!
            //        if it is a problem
            //        -> maybe use allocation pool for jobs?
            //        -> maybe use std::set?
            //        -> maybe use std::sort() and std::unique() on the list
            //           once it is completed?
            std::set<const Proxy*> mSet;
//            Set<const Proxy*> mSet;
   };

    /*!
     * \brief add one proxy to the list which proxies should be checked
     */
    inline void BroadPhaseHierarchicalGridJob2::addProxy(Proxy* proxy) {
       mProxies.push_back(proxy);
    }

    inline void BroadPhaseHierarchicalGridJob2::setSingleProxyOnly
            (bool singleProxyOnly) {
        mSingleProxyOnly = singleProxyOnly;
    }
}

#endif
/*
 * vim: et sw=4 ts=4
 */
