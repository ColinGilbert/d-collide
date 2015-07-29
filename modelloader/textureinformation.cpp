/*******************************************************************************
 *  Copyright (C) 2007 by the members of PG 510, University of Dortmund:       *
 *              d-collide-devel@lists.sourceforge.net                          *
 *     Andreas Beckermann, Christian Bode, Marcel Ens, Sebastian Ens,          *
 *     Martin Fassbach, Maximilian Hegele, Daniel Haus, Oliver Horst,         *
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

#include "textureinformation.h"

#include <math/vector.h>

namespace ModelLoader {
    TextureInformation::TextureInformation() {
        mTextured = false;
        mTexelSharing = true;
        mTransparent = false;
        mTransparency = 1.0;
        // mode? -> mirrored, tiled, ...
        //
        // rename to MaterialInformation? --> also store additional data
        // (especially lighting)
    }

    TextureInformation::TextureInformation(const TextureInformation& t) {
        *this = t;
    }

    TextureInformation::~TextureInformation() {
    }

    TextureInformation& TextureInformation::operator=(const TextureInformation& t) {
        mTextured = t.mTextured;
        mTexels = t.mTexels;
        mTransparency = t.mTransparency;
        mTransparent = t.mTransparent;
        mTexelSharing = t.mTexelSharing;
        mTextureFileName = t.mTextureFileName;
        return *this;
    }

    void TextureInformation::setTextured(bool t) {
        mTextured = t;
    }

    bool TextureInformation::isTextured() const {
        return mTextured;
    }

    void TextureInformation::setTransparent(float alpha) {
        if (alpha >= 0.9999999) {
            mTransparent = false;
            mTransparency = 1.0;
        } else {
            mTransparent = true;
            mTransparency = alpha;
        }
    }

    bool TextureInformation::isTransparent() const {
        return mTransparent;
    }

    float TextureInformation::getTransparency() const {
        return mTransparency;
    }

    /*!
     * Set the texels, i.e. texture coordinates for each \ref
     * dcollide::Mesh that this TextureInformation object is used for.
      *
     * Normally each texel has z=0.0, since they define coordinates on a 2D
     * texture map. Also note that values should be in [0.0, 1.0].
    *
     * \param shareTexels If TRUE (the default) then there must be as many
     * texels in the \p texels vector as vertices in the \ref dcollide::Mesh.
     * If FALSE, then there must be as many texels in the \p texels vector as
     * triangles in the \ref dcollide::Mesh times three, i.e. one texel per
     * triangle vertex. The difference can be seen e.g. in a simple box: every
     * point belongs to 3 sides and possibly to up to 6 triangles. For each of
     * the three sides, there usually must be a different texel for that point.
     * If you use exactly 8 vertices (one vertex per point) for that box, you
     * need to disable texel sharing, because you need more texels than
     * vertices. If you consider a two pairs (pointA,texelA) and (pointA,texelB)
     * as two different vertices (although they are at the same point), then you
     * should enable texel sharing, as you already duplicated the points.
     */
    void TextureInformation::setTexels(const std::vector<dcollide::Vector3>& texels, bool shareTexels) {
        mTexels = texels;
        mTexelSharing = shareTexels;
    }

    const std::vector<dcollide::Vector3>& TextureInformation::getTexels() const {
        return mTexels;
    }

    bool TextureInformation::getTexelSharing() const {
        return mTexelSharing;
    }

    void TextureInformation::setTextureFileName(const std::string& fileName) {
        mTextureFileName = fileName;
    }
    const std::string& TextureInformation::getTextureFileName() const {
        return mTextureFileName;
    }
}

/*
 * vim: et sw=4 ts=4
 */
