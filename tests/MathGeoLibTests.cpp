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
}
