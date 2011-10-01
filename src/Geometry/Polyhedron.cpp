#include <set>
#include <utility>
#include "assume.h"
#include "Geometry/Plane.h"
#include "Geometry/Polygon.h"
#include "Geometry/Polyhedron.h"
#include "Geometry/LineSegment.h"
#include "Geometry/Triangle.h"

int Polyhedron::NumEdges() const
{
    return EdgeIndices().size();
}

LineSegment Polyhedron::Edge(int i) const
{
    std::vector<LineSegment> edges = Edges();
    return edges[i];
}

std::vector<LineSegment> Polyhedron::Edges() const
{
    std::vector<std::pair<int, int> > edges = EdgeIndices();
    std::vector<LineSegment> edgeLines;
    edgeLines.reserve(edges.size());
    for(size_t i = 0; i < edges.size(); ++i)
        edgeLines.push_back(LineSegment(Vertex(edges[i].first), Vertex(edges[i].second)));

    return edgeLines;
}

std::vector<std::pair<int, int> > Polyhedron::EdgeIndices() const
{
    std::set<std::pair<int, int> > uniqueEdges;
    for(int i = 0; i < NumFaces(); ++i)
    {
        int x = f[i].v.back();
        for(size_t j = 0; j < f[i].v.size(); ++j)
        {
            int y = f[i].v[j];
            uniqueEdges.insert(std::make_pair(std::min(x, y), std::max(x, y)));
            x = y;
        }
    }

    std::vector<std::pair<int, int> >edges;
    edges.insert(edges.end(), uniqueEdges.begin(), uniqueEdges.end());
    return edges;
}

Polygon Polyhedron::FacePolygon(int faceIndex) const
{
    Polygon p;
    p.p.reserve(f[faceIndex].v.size());
    for(size_t v = 0; v < f[faceIndex].v.size(); ++v)
        p.p.push_back(Vertex(f[faceIndex].v[v]));
    return p;
}

Plane Polyhedron::FacePlane(int faceIndex) const
{
    return FacePolygon(faceIndex).PlaneCCW();
}

bool Polyhedron::IsClosed() const
{
    std::set<std::pair<int, int> > uniqueEdges;
    for(int i = 0; i < NumFaces(); ++i)
    {
        assume(FacePolygon(i).IsPlanar());
        assume(FacePolygon(i).IsSimple());
        int x = f[i].v.back();
        for(size_t j = 0; j < f[i].v.size(); ++j)
        {
            int y = f[i].v[j];
            if (uniqueEdges.find(std::make_pair(x, y)) != uniqueEdges.end())
                return false; // This edge is being used twice! Cannot be simple and closed.
            uniqueEdges.insert(std::make_pair(x, y));
            x = y;
        }
    }

    for(std::set<std::pair<int, int> >::iterator iter = uniqueEdges.begin(); 
        iter != uniqueEdges.end(); ++iter)
    {
        std::pair<int, int> reverse = std::make_pair(iter->second, iter->first);
        if (uniqueEdges.find(reverse) == uniqueEdges.end())
            return false;
    }

    return true;
}

bool Polyhedron::IsConvex() const
{
    for(int f = 0; f < NumFaces(); ++f)
    {
        Plane p = FacePlane(f);
        for(int i = 0; i < NumVertices(); ++i)
            if (p.SignedDistance(Vertex(i)) > 1e-3f) // Tolerate a small epsilon error.
                return false;
    }
    return true;
}
/*
bool Polyhedron::Contains(const float3 &point) const
{
    assume(false && "Not implemented!");
}
*/

bool Polyhedron::ContainsConvex(const float3 &point) const
{
    assume(IsConvex());
    for(int i = 0; i < NumFaces(); ++i)
        if (FacePlane(i).SignedDistance(point) > 0.f)
            return false;

    return true;
}

bool Polyhedron::ContainsConvex(const LineSegment &lineSegment) const
{
    return ContainsConvex(lineSegment.a) && ContainsConvex(lineSegment.b);
}

bool Polyhedron::ContainsConvex(const Triangle &triangle) const
{
    return ContainsConvex(triangle.a) && ContainsConvex(triangle.b) && ContainsConvex(triangle.c);
}
