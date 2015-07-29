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

#include "shapes/mesh/triangle.h"

#include "shapes/mesh/vertex.h"
#include "bvhnode.h"
#include "exceptions/exception.h"
#include "math/plane.h"

namespace dcollide {

    /*!
     * Creates an triangle out of the given three vertices \p v1, \p v2 and
     * \p v3. The missing normal vectors will be marked as invalid
     * (see Triangle::getNormalVector() for more details) and possibly
     * calculated later on. When they got calculated, then they are filled up
     * with a normal vector whichrepresents the normal vector of the plane that
     * is given by the three vertices.
     * 
     * Throws an MeshTopologyException when one of the given vertices is null.
     * 
     * OWNERSHIP NOTICE:
     * This class does NOT take ownership of the vertex pointers in \p v1,
     * \p v2 and \p v3.
     */
    Triangle::Triangle(Vertex* v1, Vertex* v2, Vertex* v3) {
        
        if (!v1 || !v2 || !v3) {
            throw MeshTopologyException(
                "The triangle hasn't enough vertices!" );
        }


        mVertices[0] = v1;
        mVertices[1] = v2;
        mVertices[2] = v3;
        
        mNormalsInitialized = false;
        mNormalConeInitialized = false;
        mNormalVectorInitialized = false;
        mWorldCoordinatesNormalVectorInitialized = false;
    }
    
    
    /*!
     * Deletes objects according to the ownership notices.
     */
    Triangle::~Triangle() {
        
        for (std::vector<Vector3*>::iterator iter = mNormals.begin();
             iter != mNormals.end();
             ++iter) {
            
            delete *iter;
        }
    }
    
    /*!
     * \brief Initializes the triangle object
     * 
     * This method is invoked by different constructors to do the same generic
     * initializing of the triangle object. This includes setting the internal
     * vertices variable mVertices and the internal normals variable mNormals.
     * 
     * When one or more of the given normal vectors \p n1, \p n2 or \p n3 equals
     * null it is replaced by a internally calculated value, see
     * Triangle::updateNormals() for more details).
     * 
     * Throws an MeshTopologyException when one of the given vertices is null.
     */
    void Triangle::init(Vertex* v1, Vector3* n1,
                        Vertex* v2, Vector3* n2,
                        Vertex* v3, Vector3* n3) {
        
        if (!v1 || !v2 || !v3) {
            throw MeshTopologyException(
                "The triangle hasn't enough vertices!" );
        }
        
       
        mVertices[0] = v1;
        mVertices[1] = v2;
        mVertices[2] = v3;

             
        
        if (!n1 || !n2 || !n3) {
            updateNormals();
            
            mNormals[0] = (!n1) ? mNormals[0] : n1;
            mNormals[1] = (!n2) ? mNormals[1] : n2;
            mNormals[2] = (!n3) ? mNormals[2] : n3;
            
        } else {
            mNormals = std::vector<Vector3*>(3);

            mNormals[0] = n1;
            mNormals[1] = n2;
            mNormals[2] = n3;
        }
        
        mNormalsInitialized = true;
        mNormalConeInitialized = false;
        mNormalVectorInitialized = false;
        mWorldCoordinatesNormalVectorInitialized = false;
    }
    
    
    /*!
     * \brief Sets the normal vectors of vertices to the current normal vector.
     * 
     * This method updates the private class member variable \p mNormals,
     * therefor the member variable mNormalVector is used. As mNormalVector is a
     * cached value it is calculated when not valid.
     * See Triangle::updateNormalVector() for more details on the calculation.
     */
    void Triangle::updateNormals() {
        
        if (!mNormalVectorInitialized) {
            updateNormalVector();
        }

        if (mNormals.size() != 3) {
            for (std::vector<Vector3*>::iterator iter = mNormals.begin(); iter != mNormals.end(); ++iter) {
                delete *iter;
            }

            mNormals = std::vector<Vector3*>(3);
            mNormals[0] = new Vector3(mNormalVector);
            mNormals[1] = new Vector3(mNormalVector);
            mNormals[2] = new Vector3(mNormalVector);
        } else {
            *mNormals[0] = mNormalVector;
            *mNormals[1] = mNormalVector;
            *mNormals[2] = mNormalVector;
        }

        mNormalsInitialized = true;
    }
    
    /*!
     * \brief Updates the normal vector of this triangle
     * 
     * This method updates the private class attribute \p mNormalVector,
     * therefor the normal vector of the triangle is calculated. Within the
     * calculation we consider the anticlockwise notation of the vertices, so
     * that we get the normal vector which points in the right direction.  
     */
    void Triangle::updateNormalVector() {
        
        Vector3 line1, line2;
        
        line1 = mVertices[1]->getPosition() - mVertices[0]->getPosition();
        line2 = mVertices[2]->getPosition() - mVertices[1]->getPosition();

        mNormalVector = Vector3(line1 * line2);
        mNormalVector.normalize();

        mNormalVectorInitialized = true;
    }

    /*!
     * \brief Updates the transformed normal vector in world coordinates
     * 
     * This method updates private class attribute mWorldCoordinatesNormalVector
     * therefor the normal vector of the triangle is calculated in world
     * coordinates. Like we do in Triangle::updateNormalVector() we consider the
     * anticlockwise notation of the vertices within the calculation, so that we
     * get the normal vector which points in the right direction. 
     */
    void Triangle::updateWorldCoordinatesNormalVector() {
        
        Vector3 line1, line2;
        
        line1 = mVertices[1]->getWorldPosition() - mVertices[0]->getWorldPosition();
        line2 = mVertices[2]->getWorldPosition() - mVertices[1]->getWorldPosition();

        mWorldCoordinatesNormalVector = Vector3(line1 * line2);
        mWorldCoordinatesNormalVector.normalize();

        mWorldCoordinatesNormalVectorInitialized = true;
    }

    /*!
     * \brief Updates the normal cone of this triangle
     * 
     * This method updates the private class attribute \p mNormalCone, therefor
     * a new normal cone is calculated out of the normal vector of the triangle.
     * We use the cached value mNormalVector here, if it isn't valid we
     * calculate a new normal vector through a call to
     * Triangle::updateNormalVector().
     */
    void Triangle::updateNormalCone() { 
        
        if (!mNormalVectorInitialized) {
            updateNormalVector();
        }
        
        mNormalCone = NormalCone(mNormalVector, 0.0);
        
        mNormalConeInitialized = true;
    }
    
    /*!
     * \brief Checks if two triangles share an edge
     * 
     * \return true if at least one shared edge was found and false otherwise.
     */
    bool Triangle::isEdgeAdjacentTo(const Triangle* t) const {
        for (std::set<Triangle*>::const_iterator iter = mEdgeAdjacentTriangles.begin();
             iter != mEdgeAdjacentTriangles.end(); ++iter) {
            
            if (t == (*iter)) {
                return true;
            }
        }
    
        return false;
    }
   
    /*!
     * \brief Checks if two triangles share a vertex
     * 
     * \return true if at least one shared vertex was found and false otherwise.
     */
    bool Triangle::isVertexAdjacentTo(const Triangle* t) const {
        for (std::set<Triangle*>::const_iterator iter = mVertexAdjacentTriangles.begin();
                        iter != mVertexAdjacentTriangles.end(); ++iter) {
            if (t == (*iter)) {
                return true;
            }
        }

        return false;
    }    

    /*!
     * \breif Checks if the given vertex \p v is an element of this triangle.
     */
    bool Triangle::hasElement(const Vertex* v) const {
        for(int i = 0; i<= 2; i++ ) {
            if(mVertices[i] == v ) {
               return true;
            }
        }
        return false;
    }
    
    /*!
     * \brief Set the normal vectors to the given new values.
     * 
     * While setting the normal vectors for each vertex the old vector objects
     * are destroyed according to the ownership notice.
     * 
     * OWNERSHIP NOTICE:
     * The triangle takes the ownership of the given vector pointers in
     * \p normals.
     */ 
    void Triangle::setNormals(const std::vector<Vector3*>& normals) {
        
        for (std::vector<Vector3*>::iterator iter = mNormals.begin();
             iter != mNormals.end();
             ++iter) {
            
            delete *iter;
        }
        
        mNormals = normals;
    }
    
    /*!
     * \brief calculates and stores Edge vectors and their dot products
     * 
     * they will be used in \ref Triangle::containsPoint()
     */ 
    void Triangle::calculateEdgeData(){
        mEdge01 = mVertices[1]->getWorldPosition() -  mVertices[0]->getWorldPosition();
        mEdge02 = mVertices[2]->getWorldPosition() -  mVertices[0]->getWorldPosition();
        mEdge12 = mVertices[2]->getWorldPosition() -  mVertices[1]->getWorldPosition();
                
        // Compute dot products
        mDot0101 = Vector3::dotProduct(mEdge01, mEdge01);
        mDot0102 = Vector3::dotProduct(mEdge01, mEdge02);
        mDot0202 = Vector3::dotProduct(mEdge02, mEdge02);

    }
    
    /*!
     * \brief checks if the given point is within the triangle area
     * 
     * the check is made by calculating the barycentric coordinates of the point.
     * for reference see http://www.blackpawn.com/texts/pointinpoly/default.html
     * 
     * to speed up the test, you have the following options:
     * \param skipPlaneTest use this if you are sure that the point is on the
     *                      plane defined by the triangle
     * \param usePrecachedEdgeDate use this if you called \ref calculateEdgeData
     *                             manually before
     * 
     * Use \p outU and \p outV to retrieve the result of the 
     * baricentric coordinates calculation.
     * They are set only if the point is on the plane defined by the triangle.
     */
    bool Triangle::containsPoint(const Vector3& point, bool skipPlaneTest,
                                                bool usePrecachedEdgeData,
                                                real* outU, real* outV) const{
        if (!skipPlaneTest) {
            Plane triPlane( getWorldCoordinatesNormalVector(),
                                mVertices[0]->getWorldPosition(),
                                false); //normal vector is already normalized)
            if (! triPlane.isOnPlane(point, 0.0)) {
                return false;
            }
        }
        if (!usePrecachedEdgeData) {
            const_cast<Triangle*>(this)->calculateEdgeData();
        }
        
        //calculate remaining vectors and dot products
        Vector3 v2p = point -mVertices[0]->getWorldPosition(); // P - A

        real dot01v2p = Vector3::dotProduct(mEdge01, v2p);
        real dot02v2p = Vector3::dotProduct(mEdge02, v2p);
        
        // Compute barycentric coordinates
        real invDenom = 1 / (mDot0202 * mDot0101 - mDot0102 * mDot0102);
        real u = (mDot0101 * dot02v2p - mDot0102 * dot01v2p) * invDenom;
        real v = (mDot0202 * dot01v2p - mDot0102 * dot02v2p) * invDenom;

        if (outU) {
            *outU = u;
        }
        if (outV) {
            *outV = v;
        }

        return (u >= 0) && (v >= 0) && (u + v <= 1);
    }
    
}
/*
 * vim: et sw=4 ts=4
 */
