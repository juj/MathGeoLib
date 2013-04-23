/* Copyright Jukka Jylänki

   Licensed under the Apache License, Version 2.0 (the "License");
   you may not use this file except in compliance with the License.
   You may obtain a copy of the License at

       http://www.apache.org/licenses/LICENSE-2.0

   Unless required by applicable law or agreed to in writing, software
   distributed under the License is distributed on an "AS IS" BASIS,
   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
   See the License for the specific language governing permissions and
   limitations under the License. */

/** @file GeometryAll.h
	@author Jukka Jylänki
	@brief A conveniency file to include all files in the folder Geometry/. */

#pragma once

#include "../MathBuildConfig.h"
#include "../Math/MathNamespace.h"
#ifdef WIN32
// Windows GDI has a global function named Polygon. I am not renaming my Polygon object just for its sake, especially since
// I'm not expecting anyone to co-use this library with GDI. Kill the Polygon function from Windows. and force-include
// Windows.h here to erase that function signature.

// If this is an issue for you, do the following things to use MathGeoLib from behind a namespace:
// 1. Enable MATH_ENABLE_NAMESPACE and disable MATH_AUTO_USE_NAMESPACE in MathBuildConfig.h
// 2. Comment out the following three lines.
#define Polygon Polygon_unused
#include <Windows.h>
#undef Polygon
#endif

#include "AABB.h"
#include "AABB2D.h"
#include "Capsule.h"
#include "Circle.h"
#include "Frustum.h"
#include "GeometryAll.h"
#include "HitInfo.h"
#include "KDTree.h"
#include "Line.h"
#include "LineSegment.h"
#include "OBB.h"
#include "Plane.h"
#include "Polygon.h"
#include "Polyhedron.h"
#include "QuadTree.h"
#include "Ray.h"
#include "Sphere.h"
#include "Triangle.h"
#include "TriangleMesh.h"
#include "GeomType.h"
