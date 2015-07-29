;%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
; MoveFile macro
;
; Author:  theblazingangel@aol.com (for the AutoPatcher project - www.autopatcher.com)
; Created: June 2007  
; Taken from: http://nsis.sourceforge.net/MoveFileFolder
;%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

;==================
; Declarations
;==================

Var switch_overwrite        ; indicates overwrite behaviour


;==================
; MoveFile macro
;==================

!macro MoveFile sourceFile destinationFile

    !define MOVEFILE_JUMP ${__LINE__}

    ; Check source actually exists

        IfFileExists "${sourceFile}" +3 0
        SetErrors
        goto done_${MOVEFILE_JUMP}

    ; Add message to details-view/install-log

        DetailPrint "Moving/renaming file: ${sourceFile} to ${destinationFile}"

    ; If destination does not already exists simply move file

        IfFileExists "${destinationFile}" +3 0
        rename "${sourceFile}" "${destinationFile}"
        goto done_${MOVEFILE_JUMP}

    ; If overwriting without 'ifnewer' check

        ${If} $switch_overwrite == 1
        delete "${destinationFile}"
        rename "${sourceFile}" "${destinationFile}"
        delete "${sourceFile}"
        goto done_${MOVEFILE_JUMP}
        ${EndIf}

    ; If destination already exists

        Push $R0
        Push $R1
        Push $R2
        push $R3

        GetFileTime "${sourceFile}" $R0 $R1
        GetFileTime "${destinationFile}" $R2 $R3

        IntCmp $R0 $R2 0 older_${MOVEFILE_JUMP} newer_${MOVEFILE_JUMP}
        IntCmp $R1 $R3 older_${MOVEFILE_JUMP} older_${MOVEFILE_JUMP} newer_${MOVEFILE_JUMP}

        older_${MOVEFILE_JUMP}:
        delete "${sourceFile}"
        goto time_check_done_${MOVEFILE_JUMP}

        newer_${MOVEFILE_JUMP}:
        delete "${destinationFile}"
        rename "${sourceFile}" "${destinationFile}"
        delete "${sourceFile}" ;incase above failed!

        time_check_done_${MOVEFILE_JUMP}:

        Pop $R3
        Pop $R2
        Pop $R1
        Pop $R0

    done_${MOVEFILE_JUMP}:

    !undef MOVEFILE_JUMP

!macroend
