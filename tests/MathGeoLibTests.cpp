#include "MathGeoLibTests.h"

void AddLineTests();
void AddPositiveIntersectionTests();
void AddNegativeIntersectionTests();
void AddMatrixTests();
void AddVectorTests();
void AddTransformTests();
void AddSerializationTests();
void AddClockTests();
void AddMathFuncTests();
void AddLCGTests();

void AddMathGeoLibTests()
{
	AddMathFuncTests();
	AddClockTests();
	AddLineTests();
	AddPositiveIntersectionTests();
	AddNegativeIntersectionTests();
	AddMatrixTests();
	AddVectorTests();
	AddSerializationTests();
	AddTransformTests();
	AddLCGTests();
}
