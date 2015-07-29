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

#include "proxyfactory.h"

#include "exceptions/exception.h"
#include "proxy.h"
#include "world.h"

namespace dcollide {
    /*!
     * Constructs a new ProxyFactory object.
     *
     * Can not be called directly, use \ref World::getProxyFactory
     * instead.
     */
    ProxyFactory::ProxyFactory(World* world) {
        mWorld = world;
    }

    ProxyFactory::~ProxyFactory() {
    }

    /*!
     * Create a new \ref Proxy object for use in the \ref World object that this
     * factory belongs to. The proxy has no geometrical shape. It can act as a
     * containter for other proxies. See also \ref Proxy::Proxy.
     *
     * Note that the Proxy is NOT added to the world, you have to add it either
     * to another proxy (see \ref Proxy::addChild) or to the World (see \ref
     * World::addProxy) yourself.
     *
     * Ownership notice: the ownership of the returned object is given to the
     * caller, i.e. the object is NOT automatically deleted.
     */
    Proxy* ProxyFactory::createProxy(ProxyTypes type) {
        return new Proxy(mWorld, type);
    }
    /*!
     * \overload
     * 
     * Creates a new Proxy with the given geometrical shape.
     * It can also act as a container for more shapes.
     */
    Proxy* ProxyFactory::createProxy(Shape* shape, ProxyTypes type) {
        return new Proxy(mWorld, shape, type);
    }

}
/*
 * vim: et sw=4 ts=4
 */
