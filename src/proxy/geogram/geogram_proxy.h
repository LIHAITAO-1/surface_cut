#pragma once

#include <geogram/basic/common.h>
#include <geogram/basic/logger.h>
#include <geogram/basic/command_line.h>
#include <geogram/basic/command_line_args.h>
#include <geogram/basic/stopwatch.h>
#include <geogram/mesh/mesh.h>
#include <geogram/mesh/mesh_io.h>
#include <geogram/mesh/mesh_repair.h>
#include <geogram/mesh/mesh_fill_holes.h>
#include <geogram/mesh/mesh_surface_intersection.h>
#include <geogram/mesh/mesh_geometry.h>
#include <geogram/mesh/mesh_remesh.h>

#include "basic/typedef.h"

namespace GEO_Proxy {

    inline void set_mesh_point(
            GEO::Mesh &M, GEO::index_t v, const double *coords, GEO::index_t dim
    ) {
        geo_debug_assert(M.vertices.dimension() >= dim);
        if (M.vertices.single_precision()) {
            float *p = M.vertices.single_precision_point_ptr(v);
            for (GEO::index_t c = 0; c < dim; ++c) {
                p[c] = float(coords[c]);
            }
        } else {
            double *p = M.vertices.point_ptr(v);
            for (GEO::index_t c = 0; c < dim; ++c) {
                p[c] = coords[c];
            }
        }
    }

    inline base_type::Vector3 get_mesh_point(GEO::Mesh &M, GEO::index_t v
    ) {
        if (M.vertices.single_precision()) {
            float *p = M.vertices.single_precision_point_ptr(v);
            return base_type::Vector3(p[0], p[1], p[2]);
        } else {
            double *p = M.vertices.point_ptr(v);
            return base_type::Vector3(p[0], p[1], p[2]);
        }
    }


    void get_geogram_mesh(base_type::Triangle_Soup_Mesh &m, GEO::Mesh &mesh_geo);

    void get_tri_mesh(base_type::Triangle_Soup_Mesh &m, GEO::Mesh &mesh_geo);

    void get_tet_mesh(base_type::Tetrahedra_Mesh &m, GEO::Mesh &mesh_geo);
}



