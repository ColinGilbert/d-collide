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

; Setup Build Files
;-------------------------------------------------------------------------------

    !ifndef DONT_BUILD_FILES
        !ifdef UNIX_BUILD
            !system 'test -d build && rm -rf build'
            !system 'test -d files && rm -rf files'
            !system 'mkdir build'
            !system 'mkdir files'
            !cd 'build'
            !system 'cmake -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=../files ../..' = 0
            !system 'make install' = 0
            
            !cd '..'
            !system 'svn export "http://d-collide.ematia.de/svn/d-collide/trunk/d-collide" files/src'
        !else
            !system 'if exist build rmdir /S /Q build'
            !system 'if exist files rmdir /S /Q files'
            !system 'mkdir build'
            !system 'mkdir files'
            !cd 'build'
            !system 'cmake -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=../files -G "NMake Makefiles" ../..' = 0
            !system 'nmake install' = 0
            
            !cd '..'
            !system 'svn export "http://d-collide.ematia.de/svn/d-collide/trunk/d-collide" files\src'
        !endif
    !endif


; General
;-------------------------------------------------------------------------------

    ; Name and File
    Name "D-Collide"
    OutFile "dcollide-setup.exe"
    
    ; Default installation directory
    InstallDir $PROGRAMFILES\D-Collide
    
    ; Get installation directory from registry if available
    InstallDirRegKey HKCU \
                     "Software\Microsoft\Windows\CurrentVersion\Uninstall\D-Collide" \
                     "InstallLocation"
    
    ; Removes annoying copyright notice
    BrandingText " "
    
    ; For Vista compatibility
    RequestExecutionLevel admin
    
    ; Set compression method for smaller installer executables
    SetCompressor lzma


; Defines / Settings
;-------------------------------------------------------------------------------

    !define MUI_ABORTWARNING


; Includes
;-------------------------------------------------------------------------------

    !AddIncludeDir include
    
    !include "MUI.nsh"
    !include "ZipDLL.nsh"
    !include "Locate.nsh"
    !include "LogicLib.nsh"
    
    !include "DumpLog.nsh"
    !include "Explode.nsh"
    !include "GetFileVersion.nsh"
    !include "MoveFile.nsh"
    !include "RIndexOf.nsh"
    !include "StrLoc.nsh"
    !include "StrReplace.nsh"
    !include "StrStr.nsh"
    !include "AddToPath.nsh"


; Download locations
;-------------------------------------------------------------------------------

    !define URL_CMAKE           "http://www.cmake.org/files/v2.4/cmake-2.4.7-win32-x86.exe"
    !define URL_CPPUNIT_VC8     "http://home.edo.uni-dortmund.de/~horst/dcollide/cppunit-msvc8.zip"
    !define URL_CPPUNIT_VC8_SP1 "http://home.edo.uni-dortmund.de/~horst/dcollide/cppunit-msvc8-sp1.zip"
    !define URL_LIB3DS          "http://home.edo.uni-dortmund.de/~horst/dcollide/lib3ds.zip"
    !define URL_OGRE_VC71       "http://garr.dl.sourceforge.net/sourceforge/ogre/OgreSDKSetup1.4.5_VC71.exe"
    !define URL_OGRE_VC8_SP1    "http://garr.dl.sourceforge.net/sourceforge/ogre/OgreSDKSetup1.4.5_VC80.exe"
    !define URL_PTHREADS        "ftp://sourceware.org/pub/pthreads-win32/dll-latest"
    !define URL_PKGCONFIG       "http://home.edo.uni-dortmund.de/~horst/dcollide/pkgconfig.zip"
    !define URL_PKGCONFIG_FILES "http://home.edo.uni-dortmund.de/~horst/dcollide/pkgconfig-files.zip"
    !define URL_VC_REDIST       "http://download.microsoft.com/download/e/1/c/e1c773de-73ba-494a-a5ba-f24906ecf088/vcredist_x86.exe"


; Variables
;-------------------------------------------------------------------------------

    Var URL_OGRE
    Var URL_CPPUNIT
    
    Var ENV_LIB
    Var ENV_INCLUDE
    Var ENV_OGRE_HOME
    
    Var STARTMENU_FOLDER_INSTALL
    Var STARTMENU_FOLDER_UNINSTALL

    
; General Functions / Macros
;-------------------------------------------------------------------------------

    !define FileCopy `!insertmacro FileCopy`
    
    !macro FileCopy FilePath TargetDir
        CreateDirectory `${TargetDir}`
        CopyFiles /SILENT `${FilePath}` `${TargetDir}`
    !macroend


; Extended Installation Functions
;-------------------------------------------------------------------------------

    !include "DetectCompatibleCompiler.nsh"
    !include "CheckAndInstallCMake.nsh"
    !include "CheckAndInstallCppunit.nsh"
    !include "CheckAndInstallLib3ds.nsh"
    !include "CheckAndInstallOgre.nsh"
    !include "CheckAndInstallPkgconfig.nsh"
    !include "CheckAndInstallPthreads.nsh"
    !include "CheckAndInstallVCRedist.nsh"
    !include "ConfigureSystemEnvironement.nsh"


; Pages
;-------------------------------------------------------------------------------

    !insertmacro MUI_PAGE_WELCOME
    !insertmacro MUI_PAGE_COMPONENTS
    !insertmacro MUI_PAGE_LICENSE "..\LICENSE.rtf"
    !insertmacro MUI_PAGE_DIRECTORY
    !insertmacro MUI_PAGE_STARTMENU Application $STARTMENU_FOLDER_INSTALL
    !insertmacro MUI_PAGE_INSTFILES
    !insertmacro MUI_PAGE_FINISH

    !insertmacro MUI_UNPAGE_WELCOME
    !insertmacro MUI_UNPAGE_CONFIRM
    !insertmacro MUI_UNPAGE_INSTFILES
    !insertmacro MUI_UNPAGE_FINISH


; Languages
;-------------------------------------------------------------------------------

    !insertmacro MUI_LANGUAGE "English"


; Predefined Functions
;-------------------------------------------------------------------------------

    Function .onInit
    
        System::Call 'kernel32::CreateMutexA(i 0, i 0, t "d-collide.installer") i .r1 ?e'
        Pop $R0
        
        StrCmp $R0 0 +3
            MessageBox MB_OK|MB_ICONEXCLAMATION "The installer is already running!"
            Abort
        
        ReadRegStr $0 HKCU \
                   "Software\Microsoft\Windows\CurrentVersion\Uninstall\D-Collide" \
                   "InstallLocation"
        IfErrors +2 0
        StrCpy $INSTDIR $0
        
        InitPluginsDir
        
        ReadEnvStr $ENV_OGRE_HOME 'OGRE_HOME'
        ReadEnvStr $ENV_INCLUDE 'INCLUDE'
        ReadEnvStr $ENV_LIB 'LIB'
        
        ${StrReplace} $ENV_LIB ';' '|' $ENV_LIB
        ${StrReplace} $ENV_INCLUDE ';' '|' $ENV_INCLUDE
        
        StrCpy $switch_overwrite 0
        
        Call checkICC
        Call checkMSVC
        
    FunctionEnd

    Function un.onInit
    
        System::Call 'kernel32::CreateMutexA(i 0, i 0, t "d-collide.uninstaller") i .r1 ?e'
        Pop $R0
        
        StrCmp $R0 0 +3
            MessageBox MB_OK|MB_ICONEXCLAMATION "The uninstaller is already running!"
            Abort
        
        ReadRegStr $0 HKCU \
                   "Software\Microsoft\Windows\CurrentVersion\Uninstall\D-Collide" \
                   "InstallLocation"
        IfErrors 0 +2
        Abort
        StrCpy $INSTDIR $0
    
    FunctionEnd

    Function .onInstSuccess
        MessageBox MB_YESNO "Congratulations, you've just successfully installed D-Collide!$\n$\nDo you want to view the README with useful information about bug reporting,$\ncompiling instructions and many more?" IDNO NoReadme
            Exec '"notepad.exe" "$INSTDIR\README.txt"'
        
        NoReadme:
    FunctionEnd


; Installer Sections / Components
;-------------------------------------------------------------------------------

    InstType "full"
    InstType "typical"
    InstType "minimal"
    
    
    Section "Library" "lib"
        AddSize 13282
        SectionIn RO
        
        ; Install files
        SetOutPath $INSTDIR
        File /oname=README.txt ..\README
        File /oname=LICENSE.txt ..\LICENSE
        
        SetOutPath $INSTDIR\lib
        File files\lib\dcollide.lib
        
        SetOutPath $INSTDIR\bin
        File files\bin\dcollide.dll
        
        SetOutPath $INSTDIR\include
        File /r files\include\*
        
        
        ; Check Dependencies and install them if needed,
        ; afterwards configure the system enviroment variables
        Call checkAndInstallPthreads
        Call checkAndInstallVCRedist
        Call configureSystemEnvironement
        
        
        ; Add our application to the 'Add/Remove Programs' panel 
        WriteRegStr HKCU \
                    "Software\Microsoft\Windows\CurrentVersion\Uninstall\D-Collide" \
                    "DisplayName" "D-Collide - Collision Detection Library"
        
        WriteRegStr HKCU \
                    "Software\Microsoft\Windows\CurrentVersion\Uninstall\D-Collide" \
                    "UninstallString" "$INSTDIR\Uninstall.exe"
        
        WriteRegStr HKCU \
                    "Software\Microsoft\Windows\CurrentVersion\Uninstall\D-Collide" \
                    "InstallLocation" "$INSTDIR"
        
        WriteRegStr HKCU \
                    "Software\Microsoft\Windows\CurrentVersion\Uninstall\D-Collide" \
                    "DisplayIcon" "$INSTDIR\bin\d-collide.dll,0"
        
        WriteRegStr HKCU \
                    "Software\Microsoft\Windows\CurrentVersion\Uninstall\D-Collide" \
                    "Publisher" "PG 510, University of Dortmund"
        
        WriteRegStr HKCU \
                    "Software\Microsoft\Windows\CurrentVersion\Uninstall\D-Collide" \
                    "HelpLink" "http://d-collide.ematia.de"
        
        WriteRegStr HKCU \
                    "Software\Microsoft\Windows\CurrentVersion\Uninstall\D-Collide" \
                    "URLUpdateInfo" "http://d-collide.ematia.de"
        
        WriteRegStr HKCU \
                    "Software\Microsoft\Windows\CurrentVersion\Uninstall\D-Collide" \
                    "URLInfoAbout" "http://d-collide.ematia.de"
        
        WriteRegStr HKCU \
                    "Software\Microsoft\Windows\CurrentVersion\Uninstall\D-Collide" \
                    "DisplayVersion" "0.1.0 beta"                    
        
        WriteRegDWORD HKCU \
                    "Software\Microsoft\Windows\CurrentVersion\Uninstall\D-Collide" \
                    "VersionMajor" 0
        
        WriteRegDWORD HKCU \
                    "Software\Microsoft\Windows\CurrentVersion\Uninstall\D-Collide" \
                    "VersionMinor" 1
        
        WriteRegDWORD HKCU \
                    "Software\Microsoft\Windows\CurrentVersion\Uninstall\D-Collide" \
                    "NoModify" 1
        
        WriteRegDWORD HKCU \
                    "Software\Microsoft\Windows\CurrentVersion\Uninstall\D-Collide" \
                    "NoRepair" 1
        
        
        ; Create Unistaller
        WriteUninstaller "$INSTDIR\Uninstall.exe"
        
        
        ; Create start menu folder and uninstall shortcut
        !insertmacro MUI_STARTMENU_WRITE_BEGIN Application
            CreateDirectory "$SMPROGRAMS\$STARTMENU_FOLDER_INSTALL"
            CreateShortCut "$SMPROGRAMS\$STARTMENU_FOLDER_INSTALL\Uninstall.lnk" "$INSTDIR\Uninstall.exe"
        !insertmacro MUI_STARTMENU_WRITE_END
        
        
        ; Dump log to file
        StrCpy $0 "$INSTDIR\install.log"
        Push $0
        Call DumpLog
        
    SectionEnd
    
    
    Section /o "Demo Application" "demo"
        AddSize 1085
        SectionIn 1 2
        
        SetOutPath $INSTDIR\bin
        File files\bin\testapp.exe
        
        SetOutPath $INSTDIR\resources
        File /r files\resources\*

        ${If} $MSVC_FOUND > 0
            ${If} $MSVC_SP1_FOUND = 1
                ; Install OgreSDK with support for VC 8 SP1
                StrCpy $URL_OGRE "${URL_OGRE_VC8_SP1}"
            ${Else}
                ; Install OgreSDK with support for VC 8 without SP1
                StrCpy $URL_OGRE "${URL_OGRE_VC71}"
            ${EndIf}
        ${Else}
            ; Intel Compiler or no supported compiler installed
            ; Install OgreSDK with support for VC 8 SP1
            StrCpy $URL_OGRE "${URL_OGRE_VC8_SP1}"
        ${EndIf}
        
        Call checkAndInstallOgre
        Call checkAndInstallLib3ds
        
        ; Create shortcut to demo application
        !insertmacro MUI_STARTMENU_WRITE_BEGIN Application
            CreateShortCut "$SMPROGRAMS\$STARTMENU_FOLDER_INSTALL\Demo Application.lnk" "$INSTDIR\bin\testapp.exe"
        !insertmacro MUI_STARTMENU_WRITE_END
    SectionEnd
    
    Section /o "Documentation" "doc"
        AddSize 2980
        SectionIn 1 2
        
        SetOutPath $INSTDIR\doc
        File /r files\doc\*
        
        ; Create shortcut to documentation
        !insertmacro MUI_STARTMENU_WRITE_BEGIN Application
            CreateShortCut "$SMPROGRAMS\$STARTMENU_FOLDER_INSTALL\Documentation.lnk" "$INSTDIR\doc\index.html"
        !insertmacro MUI_STARTMENU_WRITE_END
    SectionEnd
    
    SectionGroup /e "Developing Files" "dev"
        Section /o "Source Code" src
            AddSize 5492
            SectionIn 1
            
            SetOutPath $INSTDIR\src
            File /r files\src\*
            
        SectionEnd
        
        Section /o "Dependencies" "dep"
            AddSize 0
            SectionIn 1
            
            ${If} $ICC_FOUND = 0
            ${OrIf} $MSVC_FOUND = 0
                MessageBox MB_ICONEXCLAMATION|MB_OK "No supported compiler found.$\nPlease install either the Intel C++ Compiler (>= version 10) and/or the Microsoft Visual C++ Compiler (>= version 8 SP1).$\n$\nAll other dependencies will be autmatically detected and installed in the further processing of this setup!"
                
                StrCpy $URL_OGRE "${URL_OGRE_VC8_SP1}"
                StrCpy $URL_CPPUNIT "${URL_CPPUNIT_VC8_SP1}"
            ${Else}
                ${If} $MSVC_FOUND > 0
                    ${If} $MSVC_SP1_FOUND = 1
                        ; Install dependencies with support for VC 8 SP1
                        StrCpy $URL_OGRE "${URL_OGRE_VC8_SP1}"
                        StrCpy $URL_CPPUNIT "${URL_CPPUNIT_VC8_SP1}"
                    ${Else}
                        ; Install dependencies with support for VC 8 without SP1
                        StrCpy $URL_OGRE "${URL_OGRE_VC71}"
                        StrCpy $URL_CPPUNIT "${URL_CPPUNIT_VC8}"
                    ${EndIf}
                ${Else}
                    ; Install dependencies with support for VC 8 SP1
                    StrCpy $URL_OGRE "${URL_OGRE_VC8_SP1}"
                    StrCpy $URL_CPPUNIT "${URL_CPPUNIT_VC8_SP1}"
                ${EndIf}
            ${EndIf}
            
            SectionGetFlags "demo" $0
            IntOp $0 $0 & ${SF_SELECTED}
            
            ${If} $0 != ${SF_SELECTED}
                Call checkAndInstallLib3ds
                Call checkAndInstallOgre
            ${EndIf}
            
            Call checkAndInstallCMake
            Call checkAndInstallCppunit
            Call checkAndInstallPkgconfig
            
        SectionEnd
    SectionGroupEnd
    
    
    LangString DESC_LIB  ${LANG_ENGLISH} "The library"
    LangString DESC_DEMO ${LANG_ENGLISH} "A demo application which demonstrates the functionallity of the library"
    LangString DESC_DOC  ${LANG_ENGLISH} "Documentation for the library (including API-Reference)"
    LangString DESC_DEV  ${LANG_ENGLISH} "All needed files if you want to contribute in developing d-collide"
    LangString DESC_SRC  ${LANG_ENGLISH} "The Source Code"
    LangString DESC_DEP  ${LANG_ENGLISH} "Needed dependencies to compile the library from source"

    !insertmacro MUI_FUNCTION_DESCRIPTION_BEGIN
        !insertmacro MUI_DESCRIPTION_TEXT ${lib}  $(DESC_LIB)
        !insertmacro MUI_DESCRIPTION_TEXT ${demo} $(DESC_DEMO)
        !insertmacro MUI_DESCRIPTION_TEXT ${doc}  $(DESC_DOC)
        !insertmacro MUI_DESCRIPTION_TEXT ${dev}  $(DESC_DEV)
        !insertmacro MUI_DESCRIPTION_TEXT ${src}  $(DESC_SRC)
        !insertmacro MUI_DESCRIPTION_TEXT ${dep}  $(DESC_DEP)
    !insertmacro MUI_FUNCTION_DESCRIPTION_END


; Uninstaller Section
;-------------------------------------------------------------------------------

    Section "Uninstall"
        
        ; Delete installed files and directories
        Delete "$INSTDIR\Uninstall.exe"
        Delete "$INSTDIR\install.log"
        
        Delete "$INSTDIR\README.txt"
        Delete "$INSTDIR\LICENSE.txt"
        Delete "$INSTDIR\bin\testapp.exe"
        Delete "$INSTDIR\lib\dcollide.lib"
        
        Delete /REBOOTOK "$INSTDIR\bin\dcollide.dll"
        
        RMDir /r "$INSTDIR\doc"
        RMDir /r "$INSTDIR\src"
        
        RMDir /r "$INSTDIR\include\d-collide"
        RMDir /r "$INSTDIR\include\pthreads"
        RMDir /r "$INSTDIR\include\cppunit"
        RMDir /r "$INSTDIR\include\lib3ds"
        
        RMDir /r "$INSTDIR\resources\cegui-skin"
        RMDir /r "$INSTDIR\resources\models"
        RMDir /r "$INSTDIR\resources\textures"
        
        
        ; Remove CppUnit files
        Delete "$INSTDIR\LICENSE.cppunit"
        
        Delete "$INSTDIR\lib\cppunit.lib"
        Delete "$INSTDIR\lib\cppunitd.lib"
        
        Delete /REBOOTOK "$INSTDIR\bin\cppunit.dll"
        Delete /REBOOTOK "$INSTDIR\bin\cppunitd.dll"
        
        ; Remove Lib3ds files
        Delete "$INSTDIR\LICENSE.lib3ds"
        
        Delete "$INSTDIR\lib\lib3ds.lib"
        Delete "$INSTDIR\lib\lib3ds_d.lib"
        Delete "$INSTDIR\lib\lib3ds.exp"
        Delete "$INSTDIR\lib\lib3ds_d.exp"
        
        Delete /REBOOTOK "$INSTDIR\bin\lib3ds.dll"
        Delete /REBOOTOK "$INSTDIR\bin\lib3ds_d.dll"
        
        ; Remove PkgConfig files
        Delete "$INSTDIR\LICENSE.pkgconfig"
        
        Delete "$INSTDIR\bin\pkg-config.exe"
        
        Delete /REBOOTOK "$INSTDIR\bin\intl.dll"
        Delete /REBOOTOK "$INSTDIR\bin\iconv.dll"
        Delete /REBOOTOK "$INSTDIR\bin\libglib-2.0-0.dll"
        
        ; Remove pThreads files
        Delete "$INSTDIR\LICENSE.pthreads"
        
        Delete "$INSTDIR\lib\pthreadVC2.lib"
        Delete /REBOOTOK "$INSTDIR\bin\pthreadVC2.dll"
        
        
        ; Remove remaining empty directories
        RMDir "$INSTDIR\bin"
        RMDir "$INSTDIR\lib"
        RMDir "$INSTDIR\include"
        RMDir "$INSTDIR\resources"
        RMDir "$INSTDIR"
        
        
        ; Delete start menu entry and all empty parent directories
        !insertmacro MUI_STARTMENU_GETFOLDER Application $STARTMENU_FOLDER_UNINSTALL
        
        Delete "$SMPROGRAMS\$STARTMENU_FOLDER_UNINSTALL\Uninstall.lnk"
        Delete "$SMPROGRAMS\$STARTMENU_FOLDER_UNINSTALL\Demo Application.lnk"
        Delete "$SMPROGRAMS\$STARTMENU_FOLDER_UNINSTALL\Documentation.lnk"
        
        StrCpy $STARTMENU_FOLDER_UNINSTALL "$SMPROGRAMS\$STARTMENU_FOLDER_UNINSTALL"
        
        startMenuDeleteLoop:
            ClearErrors
            RMDir $STARTMENU_FOLDER_UNINSTALL
            GetFullPathName $STARTMENU_FOLDER_UNINSTALL "$STARTMENU_FOLDER_UNINSTALL\.."
            
            IfErrors +2
            StrCmp $STARTMENU_FOLDER_UNINSTALL $SMPROGRAMS 0 startMenuDeleteLoop
        
        
        ; Delete entry in the 'Add/Remove Programs' panel
        DeleteRegValue HKCU "Software\Microsoft\Windows\CurrentVersion\Uninstall\D-Collide" "DisplayName"
        DeleteRegValue HKCU "Software\Microsoft\Windows\CurrentVersion\Uninstall\D-Collide" "UninstallString"
        DeleteRegValue HKCU "Software\Microsoft\Windows\CurrentVersion\Uninstall\D-Collide" "InstallLocation"
        DeleteRegValue HKCU "Software\Microsoft\Windows\CurrentVersion\Uninstall\D-Collide" "DisplayIcon"
        DeleteRegValue HKCU "Software\Microsoft\Windows\CurrentVersion\Uninstall\D-Collide" "Publisher"
        DeleteRegValue HKCU "Software\Microsoft\Windows\CurrentVersion\Uninstall\D-Collide" "HelpLink"
        DeleteRegValue HKCU "Software\Microsoft\Windows\CurrentVersion\Uninstall\D-Collide" "URLUpdateInfo"
        DeleteRegValue HKCU "Software\Microsoft\Windows\CurrentVersion\Uninstall\D-Collide" "URLInfoAbout"
        DeleteRegValue HKCU "Software\Microsoft\Windows\CurrentVersion\Uninstall\D-Collide" "DisplayVersion"                    
        DeleteRegValue HKCU "Software\Microsoft\Windows\CurrentVersion\Uninstall\D-Collide" "VersionMajor"
        DeleteRegValue HKCU "Software\Microsoft\Windows\CurrentVersion\Uninstall\D-Collide" "VersionMinor"
        DeleteRegValue HKCU "Software\Microsoft\Windows\CurrentVersion\Uninstall\D-Collide" "NoModify"
        DeleteRegValue HKCU "Software\Microsoft\Windows\CurrentVersion\Uninstall\D-Collide" "NoRepair"
        
        DeleteRegKey /ifempty HKCU "Software\Microsoft\Windows\CurrentVersion\Uninstall\D-Collide"
        
        
        ; Reset environement settings
        Call un.configureSystemEnvironement
        
    SectionEnd
