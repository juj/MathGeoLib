:: The warning incorrectStringBooleanError: 'Conversion of string literal "xxx" to bool always evaluates to true.' is suppressed because of the commonly used idiom 'assert(condition && "error message");'
:: The warning uninitMemberVar: 'Member variable 'ClassName::memberName' is not initialized in the constructor.' is suppressed because MathGeoLib doesn't commonly initialize members in ctors (performance).
:: The warning unusedFunction: 'The function 'funcName' is never used.' is suppressed because as a generic utility library, most of the functions are not used by the library itself.
:: The warning noConstructor: 'The class 'classname' does not have a constructor is suppressed because MathGeoLib uses the keyword 'class' for POD-like objects as well.
:: The warning ConfigurationNotChecked: 'Skipping configuration 'Complex' because it seems to be invalid. Use -D if you want to check it.' is suppressed because cppcheck runs through combinations that are not needed.

:: In suppressions.txt:
:: unusedPrivateFunction:Math/Quat.h: cppcheck does not see that the functions 'Quat::operator*/+-()' are being used.
:: A false positive: https://sourceforge.net/apps/trac/cppcheck/ticket/5788:
::    Math\Quat.cpp(689): (warning) (invalidPrintfArgType_float): %f in format string (no. 4) requires 'double' but the argument type is 'float3'.

set OLDPATH=%PATH%
set PATH=%PATH%;C:\Program Files (x86)\Cppcheck

call cppcheck --version
IF ERRORLEVEL 1 GOTO NO_CPPCHECK

call cppcheck --template "{file}({line}): ({severity}) ({id}): {message}" -UMATH_QT_INTEROP -I../src -rp=../src --enable=all --suppress=unusedFunction --suppress=noConstructor --suppress=uninitMemberVar --suppress=ConfigurationNotChecked --suppress=incorrectStringBooleanError --error-exitcode=1 --force --suppressions suppressions.txt ../src
GOTO END

:NO_CPPCHECK
echo Could not find the cppcheck executable! Please add it to system PATH and try again!

:END
set PATH=%OLDPATH%

pause
