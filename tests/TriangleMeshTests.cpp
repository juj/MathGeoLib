#include "../src/Math/myassert.h"
#include "../src/MathGeoLib.h"
#include "../tests/TestRunner.h"

UNIQUE_TEST(TriangleMeshSet)
{
	TriangleMesh *m = new TriangleMesh();
	std::vector<float3> vertices(3*100);
	std::vector<float3> vertices2(3*100);
	m->Set(&vertices[0], 100);
	m->Set(&vertices2[0], 100);
	delete m;
}

UNIQUE_TEST(TriangleMeshSet2)
{
	TriangleMesh m, m2;
	std::vector<float3> vertices(3*100);
	std::vector<float3> vertices2(3*100);
	m.Set(&vertices[0], 100);
	m2 = m;
	m2.Set(&vertices2[0], 100);
	m.Set(&vertices2[0], 100);
}
