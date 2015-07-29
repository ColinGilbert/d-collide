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

Function checkAndInstallPkgconfig

    DetailPrint "Searching pkg-config..."
    
    nsExec::ExecToStack /TIMEOUT=30000 '"pkg-config" "--version"'
    pop $5
    StrCmp $5 0 pkgconfigFound pkgconfigInstall
    
    pkgconfigFound:
        pop $5
        DetailPrint "pkg-config version $5 found!"
        return
        
    pkgconfigInstall:
        ; download and install pkgconfig files
        
        DetailPrint "Downloading pkg-config files..."
        
        NSISdl::Download /TIMEOUT=30000 "${URL_PKGCONFIG}" "$PLUGINSDIR\pkgconfig.zip"
        
        Pop $0
        StrCmp $0 "success" pkgconfigDownloadSucceeded
        StrCmp $0 "cancel" pkgconfigDownloadCanceled pkgconfigDownloadFailed
        
        NSISdl::Download /TIMEOUT=30000 "${URL_PKGCONFIG_FILES}" "$PLUGINSDIR\pkgconfig-files.zip"
        
        Pop $0
        StrCmp $0 "success" pkgconfigDownloadSucceeded
        StrCmp $0 "cancel" pkgconfigDownloadCanceled pkgconfigDownloadFailed
        
        pkgconfigDownloadSucceeded:
            DetailPrint "Installing pkg-config..."
            
            ZipDLL::extractall "$PLUGINSDIR\pkgconfig.zip" "$PLUGINSDIR"
            ZipDLL::extractall "$PLUGINSDIR\pkgconfig-files.zip" "$PLUGINSDIR"
            
            !insertmacro MoveFile "$PLUGINSDIR\pkgconfig\LICENSE" "$INSTDIR\LICENSE.pkgconfig"
            ${FileCopy} "$PLUGINSDIR\pkgconfig\bin\pkg-config.exe" "$INSTDIR\bin"
            ${FileCopy} "$PLUGINSDIR\pkgconfig\bin\intl.dll" "$INSTDIR\bin"
            ${FileCopy} "$PLUGINSDIR\pkgconfig\bin\iconv.dll" "$INSTDIR\bin"
            ${FileCopy} "$PLUGINSDIR\pkgconfig\bin\libglib-2.0-0.dll" "$INSTDIR\bin"
            
            ReadEnvStr $1 "OGRE_HOME"
            ${FileCopy} "$PLUGINSDIR\CEGUI.pc" "$1\lib\pkgconfig"
            ${FileCopy} "$PLUGINSDIR\OGRE.pc" "$1\lib\pkgconfig"
            ${FileCopy} "$PLUGINSDIR\OIS.pc" "$1\lib\pkgconfig"
            
            return
            
        pkgconfigDownloadCanceled:
            MessageBox MB_OK \
                        "Download of pkg-config canceled by user!$\nCMake want be able to find some dependencies without pkg-config, thus you want be able to compile the d-collide project.$\n$\nPlease download and install it manually from http://pkg-config.freedesktop.org."
            return
            
        pkgconfigDownloadFailed:
            MessageBox MB_ICONEXCLAMATION|MB_RETRYCANCEL \
                       "Couldn't download pkg-config!$\nCMake want be able to find some dependencies without pkg-config, thus you want be able to compile the d-collide project.$\n$\nPlease download and install it manually from http://pkg-config.freedesktop.org or hit 'Retry' to try to download it again." \
                       IDRETRY pkgconfigInstall
            return
        
FunctionEnd
