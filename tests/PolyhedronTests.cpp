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
