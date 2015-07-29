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

Function checkMSVC
    
    Var /GLOBAL MSVC_FOUND
    Var /GLOBAL MSVC_SP1_FOUND
    
    Push $0
    Push $1
    Push $2
    
    DetailPrint "Searching for Microsoft Visual C++ compiler..."
    
    StrCpy $MSVC_FOUND 0
    StrCpy $MSVC_SP1_FOUND 0
    
    ReadEnvStr $0 'VCInstallDir'
    IfErrors msvcNotFound
    
    ClearErrors
    StrCmp $0 "" msvcNotFound msvcFileFound
    
    msvcFileFound:
        ${GetFileVersion} "$0\bin\c1xx.dll" $1
        IfErrors msvcNotFound
        
        ${Explode} $2 "." $1
        ${If} $2 >= 1
            Pop $MSVC_FOUND
            ${If} $MSVC_FOUND = 14
                ${If} $2 >= 2
                    Pop $3
                    ${If} $3 = 0
                        ${If} $2 >= 3
                            Pop $3
                            ${If} $3 = 50727
                                ${If} $2 >= 4
                                    Pop $3
                                    ${StrLoc} $4 $3 " (" ">"
                                    
                                    ${If} $4 != ""
                                        StrCpy $3 $3 $4
                                    ${EndIf}
                                    
                                    ${If} $3 = 42
                                        GoTo msvcFound
                                    ${ElseIf} $3 > 42
                                        GoTo msvcSP1Found
                                    ${EndIf}
                                ${EndIf}
                            ${ElseIf} $3 > 50727
                                GoTo msvcSP1Found
                            ${EndIf}
                        ${EndIf}
                    ${ElseIf} $3 > 0
                        GoTo msvcSP1Found
                    ${EndIf}
                ${EndIf}
            ${ElseIf} $MSVC_FOUND > 14
                GoTo msvcSP1Found
            ${EndIf}
        ${EndIf}
        
        GoTo msvcNotFound
        
        
    msvcSP1Found:
        StrCpy $MSVC_SP1_FOUND 1
    
    msvcFound:
        DetailPrint "Found supported version of the Microsoft Visual C++ compiler!"
        GoTo msvcRestore
        
    msvcNotFound:
        StrCpy $MSVC_FOUND 0
        ClearErrors
        DetailPrint "No supported version of the Microsoft Visual C++ compiler found..."

    msvcRestore:
        Exch $0
        Exch $1
        Exch $2
    
FunctionEnd

Function checkICC
    
    Var /GLOBAL ICC_FOUND
    
    Push $0
    Push $1
    Push $2
    Push $3
    
    DetailPrint "Searching for Intel C++ compiler..."
    
    StrCpy $0 0
    StrCpy $1 0
    StrCpy $2 0
    StrCpy $3 0
    
    iccNextOne:
        EnumRegKey $1 HKCU "Software\Intel\Compilers\C++" $0
        StrCmp $1 "" iccCancel
        IntOp $0 $0 + 1
        
        ReadRegDWORD $2 HKCU "Software\Intel\Compilers\C++\$1" "Major Version"
        ReadRegDWORD $3 HKCU "Software\Intel\Compilers\C++\$1" "Minor Version"
        IfErrors iccNextOne
        
        ${If} $2 >= 10
            ${If} $3 >= 0
                GoTo iccFound
            ${Else}
                GoTo iccNextOne
            ${EndIf}
        ${Else}
            GoTo iccNextOne
        ${EndIf}
        
    iccCancel:
        StrCpy $ICC_FOUND 0
        DetailPrint "No supported Intel C++ compiler found..."
        Goto iccRestore
        
    iccFound:
        StrCpy $ICC_FOUND $2
        DetailPrint "Intel C++ compiler version $2.$3 found..."
        
    iccRestore:
        Exch $0
        Exch $1
        Exch $2
        Exch $3
    
FunctionEnd
