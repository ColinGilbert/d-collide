; Taken from: http://nsis.sourceforge.net/GetFileVersion

Function GetFileVersion
    !define GetFileVersion `!insertmacro GetFileVersionCall`
 
    !macro GetFileVersionCall _FILE _RESULT
        Push `${_FILE}`
        Call GetFileVersion
        Pop ${_RESULT}
    !macroend
 
    Exch $0
    Push $1
    Push $2
    Push $3
    Push $4
    Push $5
    Push $6
    ClearErrors
 
    GetDllVersion '$0' $1 $2
    IfErrors error
    IntOp $3 $1 >> 16
    IntOp $3 $3 & 0x0000FFFF
    IntOp $4 $1 & 0x0000FFFF
    IntOp $5 $2 >> 16
    IntOp $5 $5 & 0x0000FFFF
    IntOp $6 $2 & 0x0000FFFF
    StrCpy $0 '$3.$4.$5.$6'
    goto end
 
    error:
        SetErrors
        StrCpy $0 ''
 
    end:
        Pop $6
        Pop $5
        Pop $4
        Pop $3
        Pop $2
        Pop $1
        Exch $0
FunctionEnd
