:: A helper .bat file to rename 'basefilename.suffix1.suffix2' to 'basefilename.suffix2'
:: %1 'basefilename.suffix1'
:: %2 '.suffix2'
:: This script is used to rename 'file.cpp.gcda/gcno' to form 'file.gcda/gcno'.
@ren %1%2 %~n1%2