#include <stdio.h>
#include <stdlib.h>

#include "../src/MathGeoLib.h"
#include "../src/Math/myassert.h"
#include "TestRunner.h"

Polyhedron RandomPolyhedronContainingPoint(const vec &pt);

RANDOMIZED_TEST(Polyhedron_intersects_itself)
{
	vec pt;
	if (rng.Int(0, 20) == 0)
		pt = POINT_VEC_SCALAR(0.f);
	else
		pt = vec::RandomBox(rng, POINT_VEC_SCALAR(-SCALE), POINT_VEC_SCALAR(SCALE));
	Polyhedron p = RandomPolyhedronContainingPoint(pt);
	assert(p.Intersects(p));
}

UNIQUE_TEST(Platonic_solids_contain_zero)
{
	for(int i = 0; i < 5; ++i)
	{
		vec pt = POINT_VEC_SCALAR(0.f);
		Polyhedron p;
		switch(i)
		{
		case 0: p = Polyhedron::Tetrahedron(pt, SCALE); break;
		case 1: p = Polyhedron::Octahedron(pt, SCALE); break;
		case 2: p = Polyhedron::Hexahedron(pt, SCALE); break;
		case 3: p = Polyhedron::Icosahedron(pt, SCALE); break;
		default: p = Polyhedron::Dodecahedron(pt, SCALE); break;
		}
		assert(p.Contains(pt));
	}
}

UNIQUE_TEST(PolyhedronContainsPointCase)
{
	Polyhedron p;
	p.v.push_back(POINT_VEC(-115.40511f, -65.362495f, -102.40900f));
	p.v.push_back(POINT_VEC(-115.40511f, -65.362495f, -39.771103f));
	p.v.push_back(POINT_VEC(-115.40511f, -18.697929f, -102.40900f));
	p.v.push_back(POINT_VEC(-115.40511f, -18.697929f, -39.771103f));
	p.v.push_back(POINT_VEC(-16.392548f, -65.362495f, -102.40900f));
	p.v.push_back(POINT_VEC(-16.392548f, -65.362495f, -39.771103f));
	p.v.push_back(POINT_VEC(-16.392548f, -18.697929f, -102.40900f));
	p.v.push_back(POINT_VEC(-16.392548f, -18.697929f, -39.771103f));
	
	int i0[4] = {0,1,3,2};
	int i1[4] = {4,6,7,5};
	int i2[4] = {0,4,5,1};
	int i3[4] = {7,6,2,3};
	int i4[4] = {0,2,6,4};
	int i5[4] = {1,5,7,3};
	Polyhedron::Face f0; f0.v.insert(f0.v.end(), i0, i0+4); p.f.push_back(f0);
	Polyhedron::Face f1; f1.v.insert(f1.v.end(), i1, i1+4); p.f.push_back(f1);
	Polyhedron::Face f2; f2.v.insert(f2.v.end(), i2, i2+4); p.f.push_back(f2);
	Polyhedron::Face f3; f3.v.insert(f3.v.end(), i3, i3+4); p.f.push_back(f3);
	Polyhedron::Face f4; f4.v.insert(f4.v.end(), i4, i4+4); p.f.push_back(f4);
	Polyhedron::Face f5; f5.v.insert(f5.v.end(), i5, i5+4); p.f.push_back(f5);

	AABB a(POINT_VEC(-115.40511f, -65.362495f, -102.40900f),
		POINT_VEC(-16.392548f,-18.697929f, -39.771103f));

	assert(p.Contains(a.CenterPoint()));
}

