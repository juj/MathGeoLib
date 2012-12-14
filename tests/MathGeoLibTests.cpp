#include "MathGeoLibTests.h"

void AddPositiveIntersectionTests();
void AddNegativeIntersectionTests();
void AddMatrixTests();
void AddVectorTests();
void AddSerializationTests();
void AddClockTests();

void AddMathGeoLibTests()
{
	AddPositiveIntersectionTests();
	AddNegativeIntersectionTests();
	AddMatrixTests();
	AddVectorTests();
	AddSerializationTests();
	AddClockTests();
}
