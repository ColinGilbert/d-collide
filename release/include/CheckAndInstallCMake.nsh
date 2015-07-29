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

Function checkAndInstallCMake
    
    Var /GLOBAL CMAKE_MAJOR_VERSION
    Var /GLOBAL CMAKE_MINOR_VERSION
    Var /GLOBAL CMAKE_PATCH_VERSION
    
    
    DetailPrint "Searching for CMake..."
    
    nsExec::ExecToStack /TIMEOUT=30000 '"cmake" "--version"'
    pop $5
    StrCmp $5 0 cmakeFound cmakeInstall
    
    cmakeFound:
        DetailPrint "Validating CMake version..."
        
        pop $6
        
        ; strip trailing 'cmake version'
        StrCpy $6 $6 "" 14
        
        ; is a patch version included in version string?
        ${StrLoc} $7 $6 "-patch" ">"
        StrCmp $7 "" cmakeVersion cmakePatchedVersion
        
        ; process patch version info -> $CMAKE_PATCH_VERSION         
        cmakePatchedVersion:
            ${StrStr} $8 $6 "patch"
            StrCpy $8 $8 "" 6
            StrCpy $CMAKE_PATCH_VERSION $8
            
            ; strip patch version info
            StrCpy $6 $6 $7
            
        ; process major and minor version info
        cmakeVersion:
            ${Explode} $7 "." $6
            
            ; process major version info -> $CMAKE_MAJOR_VERSION
            pop $8
            StrCpy $CMAKE_MAJOR_VERSION $8
            
            ; process minor version info -> $CMAKE_MINOR_VERSION
            ${If} $7 != 1
                pop $8
                StrCpy $CMAKE_MINOR_VERSION $8
            ${Else}
                StrCpy $CMAKE_MINOR_VERSION 0
            ${EndIf}
        
        ; check if the detected version is new enough
        ${If} $CMAKE_MAJOR_VERSION < 2
            GoTo cmakeFoundWithBadVersion
        ${ElseIf} $CMAKE_MAJOR_VERSION == 2
            ${If} $CMAKE_MINOR_VERSION < 4
                GoTo cmakeFoundWithBadVersion
            ${ElseIf} $CMAKE_MINOR_VERSION == 4
                ${If} $CMAKE_PATCH_VERSION < 6
                    GoTo cmakeFoundWithBadVersion
                ${EndIf}
            ${EndIf}
        ${EndIf}
        
        DetailPrint "CMake version $CMAKE_MAJOR_VERSION.$CMAKE_MINOR_VERSION-patch $CMAKE_PATCH_VERSION found!"
        return
        
    cmakeFoundWithBadVersion:
        DetailPrint "Updating CMake..."
        GoTo cmakeInstall
        
    
    ; download setup routine and install it
    cmakeInstall:
        DetailPrint "Downloading CMake setup..."
        
        NSISdl::Download /TIMEOUT=30000 "${URL_CMAKE}" "$PLUGINSDIR\cmake-setup.exe"
        
        Pop $2
        StrCmp $2 "success" cmakeDownloadSucceeded 
        StrCmp $2 "cancel" cmakeDownloadCanceled cmakeDownloadFailed
        
        cmakeDownloadSucceeded:
            HideWindow
            ExecWait "$PLUGINSDIR\cmake-setup.exe"
            BringToFront
            return
        
        cmakeDownloadCanceled:
            MessageBox MB_OK \
                        "Download of CMake canceled by user!$\nYou wont be able to configure the source when you don't have CMake installed.$\n$\nPlease download and install it manually from http://www.cmake.org."
            return
        
        cmakeDownloadFailed:
            MessageBox MB_ICONEXCLAMATION|MB_RETRYCANCEL \
                       "Couldn't download CMake!$\nYou wont be able to configure the source when you don't have CMake installed.$\n$\nPlease download and install it manually from http://www.cmake.org or hit 'Retry' to try to download it again." \
                       IDRETRY cmakeInstall
            return

FunctionEnd
