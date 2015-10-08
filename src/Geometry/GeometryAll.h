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

#if defined(WIN32) && defined(MATH_WIN32_INTEROP)
#include "../Math/InclWindows.h"
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
#include "PBVolume.h"
