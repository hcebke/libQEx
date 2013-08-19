/*
 * Copyright 2013 Computer Graphics Group, RWTH Aachen University
 * Author: Hans-Christian Ebke <ebke@cs.rwth-aachen.de>
 *
 * This file is part of QEx.
 * 
 * QEx is free software: you can redistribute it and/or modify it under
 * the terms of the GNU General Public License as published by the Free
 * Software Foundation, either version 3 of the License, or (at your
 * option) any later version.
 * 
 * QEx is distributed in the hope that it will be useful, but WITHOUT ANY
 * WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License
 * for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with QEx.  If not, see <http://www.gnu.org/licenses/>.
 */

/// @file

#ifdef __cplusplus
#include <cstddef>
#include <OpenMesh/Core/Mesh/Traits.hh>
#include <OpenMesh/Core/Mesh/PolyMesh_ArrayKernelT.hh>
#include <OpenMesh/Core/Mesh/TriMesh_ArrayKernelT.hh>
#include <vector>

extern "C" {

namespace QEx {

struct PolyTraits : public OpenMesh::DefaultTraits {
        typedef OpenMesh::Vec3d Point;
        typedef OpenMesh::Vec3d Normal;
        typedef OpenMesh::Vec4f Color;
};

/// A shorthand for a suitable OpenMesh quad mesh type.
typedef OpenMesh::PolyMesh_ArrayKernelT<PolyTraits>  QuadMesh;

struct TriTraits : public OpenMesh::DefaultTraits {
    typedef OpenMesh::Vec3d Point;
    typedef OpenMesh::Vec3d Normal;
    typedef OpenMesh::Vec4f Color;
};

/// A shorthand for a suitable OpenMesh triangle mesh type.
typedef OpenMesh::TriMesh_ArrayKernelT<TriTraits>  TriMesh;

typedef QuadMesh *QuadMesh_t;
typedef TriMesh *TriMesh_t;
typedef const std::vector<unsigned int> *const_ValenceVector_t;
typedef const std::vector<OpenMesh::Vec2d> *const_UVVector_t;

/**
 * Extract a quad mesh from the given triangle mesh using the supplied per-halfedge UVs as
 * the (relaxed) integer grid map.
 *
 * @param in_triMesh The input triangle mesh.
 * @param in_uvs The per-halfedge UVs. Precondition: in_uvs->size() == in_triMesh->n_halfedges()
 * @param in_vertexValences The vertex valences. Optional. Precondition: in_vertexValences == 0 || in_vertexValences->size() == in_triMesh->n_vertices()
 * @param out_quadMesh The result will be output into the supplied quad mesh.
 */
void extractQuadMeshOM(TriMesh_t in_triMesh, const_UVVector_t in_uvs, const_ValenceVector_t in_vertexValences, QuadMesh_t out_quadMesh);

} /* namespace QEx */

#endif

typedef unsigned int qex_Index;
typedef unsigned int qex_Valence;

typedef struct {
    double x[3];
} qex_Point3;

typedef struct {
    double x[2];
} qex_Point2;

typedef struct {
    qex_Index indices[3];
} qex_Tri;

typedef struct {
    qex_Point2 uvs[3];
} qex_UVTri;

typedef struct {
    qex_Index indices[4];
} qex_Quad;

typedef struct {
    unsigned int vertex_count;
    unsigned int tri_count;

    /** Pointer to an array of vertex_count vertices. */
    qex_Point3 *vertices;

    /** Pointer to an array of tri_count triangles. */
    qex_Tri *tris;

    /** Pointer to an array of tri_count triangle UVs. */
    qex_UVTri *uvTris;
} qex_TriMesh;

typedef struct {
    unsigned int vertex_count;
    unsigned int quad_count;

    /** Pointer to an array of vertex_count vertices. */
    qex_Point3 *vertices;

    /** Pointer to an array of quad_count quads. */
    qex_Quad *quads;
} qex_QuadMesh;

/**
 * Extract a quad mesh from the given triangle mesh with a (relaxed) integer grid map.
 *
 * @param in_triMesh A pointer to the input triangle mesh.
 *
 * @param in_vertexValences May be NULL. If not NULL it has to point to an array of in_triMesh.vertex_count
 * integers each one corresponding to the valence of the given input vertex. If this argument is omitted, the
 * valence of each vertex is determined from the cross field which can lead to ambiguous results in certain
 * degenerate cases. If you know that your integer grid map is non-degenerate (i.e. contains no fold-overs and
 * triangles degenerated to a line or a point in the parameter domain) you can safely omit this argument.
 *
 * @param out_quadMesh A pointer to a qex_QuadMesh struct. The qex_QuadMesh has to be uninitialized.
 * It is the responsibility of the caller to free() the vertices and quads members that are returned.
 */
void qex_extractQuadMesh(qex_TriMesh const * in_triMesh, qex_Valence *in_vertexValences, qex_QuadMesh * out_quadMesh);

#ifdef __cplusplus
} // extern "C"


namespace QEx {

/**
 * @brief (Semi-)Generic version of extractQuadMeshOM(). Usable with different but identical traits.
 * @see extractQuadMeshOM
 */
template<typename TriMeshT, typename QuadMeshT>
inline void extractQuadMeshOMT(TriMeshT *in_triMesh, const_UVVector_t in_uvs, const_ValenceVector_t in_vertexValences, QuadMeshT *out_quadMesh) {
    extractQuadMeshOM(OpenMesh::MeshCast<TriMesh_t, TriMeshT*>::cast(in_triMesh), in_uvs, in_vertexValences, OpenMesh::MeshCast<QuadMesh_t, QuadMeshT*>::cast(out_quadMesh));
}

} /* namespace QEx */

#endif
