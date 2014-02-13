#include "../src/Math/myassert.h"
#include "../src/MathGeoLib.h"
#include "../tests/TestRunner.h"

UNIQUE_TEST(TriangleMeshSet)
{
	TriangleMesh *m = new TriangleMesh();
	std::vector<float3> vertices(3*100);
	m->Set(&vertices[0], 100);
	delete m;
}