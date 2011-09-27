#include <set>
#include <utility>
#include "Geometry/Plane.h"
#include "Geometry/Polygon.h"
#include "Geometry/Polyhedron.h"
#include "Geometry/LineSegment.h"

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
            uniqueEdges.insert(std::make_pair(x, y));
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

Plane Polyhedron::FacePlaneCCW(int faceIndex) const
{
    return FacePolygon(faceIndex).PlaneCCW();
}

Plane Polyhedron::FacePlaneCW(int faceIndex) const
{
    return FacePolygon(faceIndex).PlaneCW();
}
