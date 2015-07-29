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

Function checkAndInstallLib3ds
    
    DetailPrint "Searching for lib3ds include..."
    
    StrCmp $ENV_INCLUDE "" lib3dsInstall
    
    ${locate::Open} "$ENV_INCLUDE" "/F=1 /D=1 /N=file.h /B=0" $0
    StrCmp $0 0 lib3dsError
    
    ${locate::Find} $0 $1 $2 $3 $4 $5 $6
    ${locate::Close} $0
    
    ; split of last folder
    ${RIndexOf} $0 $2 "\"
    StrLen $1 $2
    IntOp $0 $1 - $0
    IntOp $0 $0 + 1
    
    StrCpy $0 $2 "" $0
    StrCpy $1 $3
    
    
    ; search lib3ds shared library
    DetailPrint "Searching for lib3ds shared library..."
    
    ReadEnvStr $9 'PATH'
    ${StrReplace} $9 ';' '|' $9
    
    ${locate::Open} $9 "/F=1 /D=0 /N=lib3ds.dll /B=0" $2
    StrCmp $2 0 lib3dsError
    
    ${locate::Find} $2 $3 $4 $5 $6 $7 $8
    ${locate::Close} $2
    
    ${locate::Unload}
    
    
    ; check if both includes and library are present
    ${If} $0 == "lib3ds"
        ${If} $1 == "file.h"
            ${If} $5 == "lib3ds.dll"
                GoTo lib3dsFound
            ${EndIf}
        ${EndIf}
    ${EndIf}
    
    
    lib3dsInstall:
        ; download and install lib3ds files
        DetailPrint "Downloading lib3ds files..."
        
        NSISdl::Download /TIMEOUT=30000 "${URL_LIB3DS}" "$PLUGINSDIR\lib3ds.zip"
        
        Pop $0
        StrCmp $0 "success" lib3dsDownloadSucceeded 
        StrCmp $0 "cancel" lib3dsDownloadCanceled lib3dsDownloadFailed
        
        lib3dsDownloadSucceeded:
            DetailPrint "Installing lib3ds..."
            
            ZipDLL::extractall "$PLUGINSDIR\lib3ds.zip" "$PLUGINSDIR"
            
            !insertmacro MoveFile "$PLUGINSDIR\lib3ds\LICENSE" "$INSTDIR\LICENSE.lib3ds"
            ${FileCopy} "$PLUGINSDIR\lib3ds\bin\lib3ds.dll" "$INSTDIR\bin"
            ${FileCopy} "$PLUGINSDIR\lib3ds\bin\lib3ds_d.dll" "$INSTDIR\bin"
            ${FileCopy} "$PLUGINSDIR\lib3ds\lib\lib3ds.lib" "$INSTDIR\lib"
            ${FileCopy} "$PLUGINSDIR\lib3ds\lib\lib3ds_d.lib" "$INSTDIR\lib"
            ${FileCopy} "$PLUGINSDIR\lib3ds\lib\lib3ds.exp" "$INSTDIR\lib"
            ${FileCopy} "$PLUGINSDIR\lib3ds\lib\lib3ds_d.exp" "$INSTDIR\lib"
            ${FileCopy} "$PLUGINSDIR\lib3ds\include\lib3ds" "$INSTDIR\include"
            
            return
            
        lib3dsDownloadCanceled:
            MessageBox MB_OK \
                       "Download of lib3ds canceled by user!$\nYou wont be able to compile nor to run the demo application without the lib3ds library.$\n$\nPlease download and install it manually from http://lib3ds.sourceforge.net/."
            return
            
        lib3dsDownloadFailed:
            MessageBox MB_ICONEXCLAMATION|MB_RETRYCANCEL \
                       "Couldn't download lib3ds!$\nYou wont be able to compile nor to run the demo application without the lib3ds library.$\n$\nPlease download and install it manually from http://lib3ds.sourceforge.net/ or hit 'Retry' to try to download it again." \
                       IDRETRY lib3dsInstall
            return
            
    lib3dsFound:
        DetailPrint "Found lib3ds..."
        return
        
    lib3dsError:
        ${locate::Unload}
        DetailPrint "Error during search process..."
        MessageBox MB_ICONEXCLAMATION|MB_YESNO \
                   "An error occured during the search for a possibly installed version of the lib3ds library.$\n$\n Do you wish to install the lib3ds library?" \
                   IDYES lib3dsInstall
FunctionEnd
