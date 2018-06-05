@ECHO on
ECHO Start

IF NOT Defined LiteS (
    echo Can't find enviroment path.
) ELSE (
    del /S /Q "%LiteS%\"


    xcopy /Y "LiteS\src\*.h" "%LiteS%\include\"
    
    xcopy /Y "debug\lib\x86\LiteSd.lib" "%LiteS%\lib\x86"
    xcopy /Y "release\lib\x86\LiteS.lib" "%LiteS%\lib\x86"
    
    
)
PAUSE