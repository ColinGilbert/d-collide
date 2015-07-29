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

Function checkAndInstallPthreads

    DetailPrint "Searching for pthreads includes..."
    
    StrCmp $ENV_INCLUDE "" pthreadsInstall
    
    ${locate::Open} "$ENV_INCLUDE" "/F=1 /D=1 /N=pthread.h /B=0" $0
    StrCmp $0 0 pthreadsError
    
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
    DetailPrint "Searching for pthreads shared library..."
    
    ReadEnvStr $9 'PATH'
    ${StrReplace} $9 ';' '|' $9
    
    ${locate::Open} $9 "/F=1 /D=1 /N=pthreadVC2.dll /B=0" $2
    StrCmp $2 0 pthreadsError
    
    ${locate::Find} $2 $3 $4 $5 $6 $7 $8
    ${locate::Close} $2
    
    ${locate::Unload}
    
    
    ; check if both includes and library are present
    ${If} $1 == "pthread.h"
        ${If} $5 == "pthreadVC2.dll"
            GoTo pthreadsFound
        ${EndIf}
    ${EndIf}
    
    
    pthreadsInstall:
        ; download and install pthreads
        CreateDirectory `$INSTDIR\include\pthreads`
        
        MetaDl::Download "${URL_PTHREADS}/include/pthread.h" "$INSTDIR\include\pthreads\pthread.h"
        Pop $0
        StrCmp $0 "success" +2
        StrCmp $0 "cancel" pthreadsDownloadCanceled pthreadsDownloadFailed
        
        MetaDl::Download "${URL_PTHREADS}/include/sched.h" "$INSTDIR\include\pthreads\sched.h"
        Pop $0
        StrCmp $0 "success" +2
        StrCmp $0 "cancel" pthreadsDownloadCanceled pthreadsDownloadFailed
        
        MetaDl::Download "${URL_PTHREADS}/include/semaphore.h" "$INSTDIR\include\pthreads\semaphore.h"
        Pop $0
        StrCmp $0 "success" +2
        StrCmp $0 "cancel" pthreadsDownloadCanceled pthreadsDownloadFailed
        
        MetaDl::Download "${URL_PTHREADS}/lib/pthreadVC2.dll" "$INSTDIR\bin\pthreadVC2.dll"
        Pop $0
        StrCmp $0 "success" +2
        StrCmp $0 "cancel" pthreadsDownloadCanceled pthreadsDownloadFailed
        
        MetaDl::Download "${URL_PTHREADS}/lib/pthreadVC2.lib" "$INSTDIR\lib\pthreadVC2.lib"
        Pop $0
        StrCmp $0 "success" +2
        StrCmp $0 "cancel" pthreadsDownloadCanceled pthreadsDownloadFailed
        
        MetaDl::Download "${URL_PTHREADS}/COPYING" "$INSTDIR\LICENSE.pthreads"
        Pop $0
        StrCmp $0 "success" +2
        StrCmp $0 "cancel" pthreadsDownloadCanceled pthreadsDownloadFailed
        
        return
        
        
        pthreadsDownloadCanceled:
            MessageBox MB_OK \
                       "Download of pthreads canceled by user!$\nYou wont be able to compile nor to use the d-collide library without the pthreads library.$\n$\nPlease download and install it manually from http://sourceware.org/pthreads-win32/."
            return
            
        pthreadsDownloadFailed:
            MessageBox MB_ICONEXCLAMATION|MB_RETRYCANCEL \
                       "Couldn't download pthreads!$\nYou wont be able to compile nor to use the d-collide library without the pthreads library.$\n$\nPlease download and install it manually from http://sourceware.org/pthreads-win32/ or hit 'Retry' to try to download it again." \
                       IDRETRY pthreadsInstall
            return
    
    pthreadsFound:
        DetailPrint "Found pthreads..."
        return
        
    pthreadsError:
        ${locate::Unload}
        DetailPrint "Error during search process..."
        MessageBox MB_ICONEXCLAMATION|MB_YESNO \
                   "An error occured during the search for a possibly installed version of the pthreads library.$\n$\n Do you wish to install the pthreads library?" \
                   IDYES pthreadsInstall
    
FunctionEnd
