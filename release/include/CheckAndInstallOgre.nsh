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

Function checkAndInstallOgre
    
    DetailPrint "Search for OgreSDK..."
    
    StrLen $1 $ENV_OGRE_HOME
    IntCmp $1 0 ogreNotFound ogreNotFound ogreFound
    
    ogreNotFound:
        DetailPrint "Downloading OgreSDK setup..."
        
        NSISdl::Download /TIMEOUT=30000 "$URL_OGRE" "$PLUGINSDIR\ogre-setup.exe"
        
        Pop $2
        StrCmp $2 "success" orgeDownloadSucceeded 
        StrCmp $2 "cancel" ogreDownloadCanceled orgeDownloadFailed
        
        orgeDownloadSucceeded:
            HideWindow
            ExecWait "$PLUGINSDIR\ogre-setup.exe"
            BringToFront
            
            Push "%OGRE_HOME%\bin\release"
            Call AddToPath
            
            return
        
        ogreDownloadCanceled:
            MessageBox MB_OK \
                       "Download of OgreSDK canceled by user!$\nYou wont be able to start the demo application without the OgreSDK.$\n$\nPlease download and install it manually from http://www.ogre3d.org."
            return
        
        orgeDownloadFailed:
            MessageBox MB_ICONEXCLAMATION|MB_RETRYCANCEL \
                       "Couldn't download the OgreSDK!$\nYou wont be able to start the demo application without the OgreSDK.$\n$\nPlease download and install it manually from http://www.ogre3d.org or hit 'Retry' to try to download it again." \
                       IDRETRY ogreNotFound
            return
    
    ogreFound:
        DetailPrint "OgreSDK found..."
        
        Push "%OGRE_HOME%\bin\release"
        Call AddToPath
        
FunctionEnd
