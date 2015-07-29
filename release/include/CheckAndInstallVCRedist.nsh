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

Function checkAndInstallVCRedist
    
    ${If} $MSVC_FOUND = 0
        ClearErrors
        ReadRegDword $0 HKLM "SOFTWARE\Microsoft\Windows\CurrentVersion\Uninstall\{7299052b-02a4-4627-81f2-1818da5d550d}" "Version"

        ; If VC redistributables aren't installed already do so
        IfErrors vcredistInstall
        return
        
        vcredistInstall:
            ; Download and install VC redistributables
            DetailPrint "Downloading VC redistrubutable installer..."
            
            NSISdl::Download /TIMEOUT=30000 "${URL_VC_REDIST}" "$PLUGINSDIR\vcredist_x86.exe"
            Pop $0
            StrCmp $0 "success" vcredistDownloadSucceeded
            StrCmp $0 "cancel" vcredistDownloadCanceled vcredistDownloadFailed
            
            vcredistDownloadSucceeded:
                DetailPrint "Installing VC redistributables..."
                
                ExecWait '"$PLUGINSDIR\vcredist_x86.exe" /q:a /c:"VCREDI~3.EXE /q:a /c:""msiexec /i vcredist.msi /qb!"" "'
                IfErrors vcredistInstallationFailed
                return
            
            vcredistDownloadCanceled:
                MessageBox MB_OK \
                           "Download of VC redistributable installer canceled by user!$\nYou wont be able to start the demo application without the VC redistributables.$\n$\nPlease download and install it manually from http://www.microsoft.com."
                return
            
            vcredistDownloadFailed:
                MessageBox MB_ICONEXCLAMATION|MB_OK \
                           "Couldn't download the VC redistributable installer!$\nYou wont be able to start the demo application without the VC redistributables.$\n$\nPlease download and install it manually from http://www.microsoft.com."
                return
            
            vcredistInstallationFailed:
                MessageBox MB_ICONEXCLAMATION|MB_OK \
                           "Couldn't install the VC redistributables!$\nYou wont be able to start the demo application without the VC redistributables.$\n$\nPlease download and install it manually from http://www.microsoft.com."
                return
            
    ${EndIf}
    
FunctionEnd
