@echo off

del /Q /S *.gcda > NUL 2> NUL
del /Q *.gcno > NUL 2> NUL

MathGeoLib.exe
SET ERRORLEV=%ERRORLEVEL%

for /R %%x in (*.gcda, *.gcno) do copy %%x . > NUL

for %%E in (*.gcda) do (
call DeleteCppSuffix.bat %%~nE .gcda
)

for %%E in (*.gcno) do (
call DeleteCppSuffix.bat %%~nE .gcno
)

set GCOVFILES=
for /f "delims=" %%i in ('type ..\MathGeoLib_code_files.txt') do set GCOVFILES=%GCOVFILES% %%i

mkdir gcov_results > NUL 2> NUL
del /Q gcov_results\*.* > NUL 2> NUL

gcov %GCOVFILES% > gcov_results\gcov_summary.txt

del /Q *.gcno > NUL 2> NUL
del /Q /S *.gcda > NUL 2> NUL

move *.gcov gcov_results >  NUL 2> NUL

:: Output the summary to stdout so that buildbot picks it up and parses it.
type gcov_results\gcov_summary.txt

cmd /C exit %ERRORLEV%
pause
