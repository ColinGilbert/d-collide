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


#ifndef DCOLLIDE_<FILENAME>
#define DCOLLIDE_<FILENAME>


namespace dcollide {

    //------constants, sorted by type-------
    /*! Doxygen documentation sample for constants
     *  \brief short description of the constant
     */
    const int SAMPLE_CONSTANT = 42;


    //------definitions, sorted by type-----
    //--------basic types-----------


    //-----------enums--------------
    /*! Doxygen documentation sample for enums
     *  \brief short description of the enum
     */
    enum SampleEnum {
        ENUM_ELEMENT1,
        ENUM_ELEMENT2
    };

    //-----------structs------------
    /*! Doxygen documentation sample for enums
     *  \brief short description of the enum
     */
    struct SampleStruct {
        int value;
        Object o1;
    };

    //-----------classes------------

    /*! Doxygen documentation sample for classes
     * \brief one-line short description
     * This is just a sample class without functionality.
     * \author <authorname>
     */
    class SampleClass {
        public:
            // Constructor(s) and Destructor
            SampleClass();
            ~SampleClass();

            // public class members and methods
            int getSampleMember(bool param1);
            double sampleFunction(double param1, double param2);

        private:
            // private class members and methods, sorted by type
            int mSampleMember;
    };


    //------------ Implementation of short methods -------------
    /*! Doxygen documentation sample for short methods
     *  \brief getter for mSampleMember
     *  \param param1 describe the first parameter
     *  \return int mSampleMember
     */
    SampleClass::getSampleMember(bool param1) {
        return mSampleMember;
    }
}

#endif // DCOLLIDE_<FILENAME>
/*
 * vim: et sw=4 ts=4
 */
