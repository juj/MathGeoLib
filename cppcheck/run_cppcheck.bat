:: The warning 'Conversion of string literal "xxx" to bool always evaluates to true.' is suppressed because of the commonly used idiom 'assert(condition && "error message");'
:: The warning 'Member variable 'ClassName::memberName' is not initialized in the constructor.' is suppressed because MathGeoLib doesn't commonly initialize members in ctors (performance).

"C:\Program Files (x86)\Cppcheck\cppcheck" --version

"C:\Program Files (x86)\Cppcheck\cppcheck" --enable=all --suppress=uninitMemberVar --suppress=incorrectStringBooleanError --force ../src

pause

