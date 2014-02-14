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

// See https://github.com/juj/MathGeoLib/issues/23
TEST(TriangleMeshSet3)
{
	TriangleMesh* triangleMesh = new TriangleMesh();
	float p[] =
	{
		12800.000f, 12850.000f, 8000.000f, 12800.000f, 12850.000f, 8100.000f, 13800.000f, 12849.982f, 8200.000f,
		13800.000f, 12849.982f, 8200.000f, 13800.000f, 12849.982f, 8100.000f, 12800.000f, 12850.000f, 8000.000f,
		13800.000f, 12849.982f, 8100.000f, 13800.000f, 12849.982f, 8200.000f, 13799.969f, 11049.982f, 8200.000f,
		13799.969f, 11049.982f, 8200.000f, 13799.969f, 11049.982f, 8100.000f, 13800.000f, 12849.982f, 8100.000f,
		13799.969f, 11049.982f, 8100.000f, 13799.969f, 11049.982f, 8200.000f, 12799.969f, 11050.000f, 8100.000f,
		12799.969f, 11050.000f, 8100.000f, 12799.969f, 11050.000f, 8000.000f, 13799.969f, 11049.982f, 8100.000f,
		12799.969f, 11050.000f, 8000.000f, 12799.969f, 11050.000f, 8100.000f, 12800.000f, 12850.000f, 8100.000f,
		12800.000f, 12850.000f, 8100.000f, 12800.000f, 12850.000f, 8000.000f, 12799.969f, 11050.000f, 8000.000f,
		12800.000f, 12850.000f, 8100.000f, 12799.969f, 11050.000f, 8100.000f, 13799.969f, 11049.982f, 8200.000f,
		13799.969f, 11049.982f, 8200.000f, 13800.000f, 12849.982f, 8200.000f, 12800.000f, 12850.000f, 8100.000f,
		12800.000f, 12850.000f, 8000.000f, 13800.000f, 12849.982f, 8100.000f, 13799.969f, 11049.982f, 8100.000f,
		13799.969f, 11049.982f, 8100.000f, 12799.969f, 11050.000f, 8000.000f, 12800.000f, 12850.000f, 8000.000f
	};
	triangleMesh->Set((float3*)p, 12);
	triangleMesh->Set((Triangle*)p, 12);
	delete triangleMesh;
}
