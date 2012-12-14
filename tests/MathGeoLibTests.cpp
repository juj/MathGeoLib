#include "MathGeoLibTests.h"

void AddPositiveIntersectionTests();
void AddNegativeIntersectionTests();
void AddMatrixTests();
void AddVectorTests();
void AddSerializationTests();

void AddMathGeoLibTests()
{
	AddPositiveIntersectionTests();
	AddNegativeIntersectionTests();
	AddMatrixTests();
	AddVectorTests();
	AddSerializationTests();
}
