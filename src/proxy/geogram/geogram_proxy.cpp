//
// Created by xmy on 2024/2/22.
//

#ifndef TETGEO_GEOGRAM_PROXY_H
#define TETGEO_GEOGRAM_PROXY_H

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
#include "geogram_proxy.h"

namespace GEO_Proxy {
    void get_geogram_mesh(base_type::Triangle_Soup_Mesh &m, GEO::Mesh &mesh_geo) {
        mesh_geo.vertices.create_vertices(m.vertex_pool.size());

        for (int i = 0; i < m.vertex_pool.size(); i++) {
            base_type::Vertex *v = (base_type::Vertex *) m.vertex_pool[i];
            double xyz[3];
            xyz[0] = v->position.x;
            xyz[1] = v->position.y;
            xyz[2] = v->position.z;
            set_mesh_point(mesh_geo, i, xyz, 3);
        }
        for (int i = 0; i < m.face_pool.size(); i++) {
            base_type::Face *f = (base_type::Face *) m.face_pool[i];

            GEO::index_t f_index = mesh_geo.facets.create_triangle(f->p1->static_index, f->p2->static_index,
                                                                   f->p3->static_index);
        }
        //mesh_geo.facets.connect();
    }

    void get_tri_mesh(base_type::Triangle_Soup_Mesh &m, GEO::Mesh &mesh_geo) {

        m.clear();

        for (GEO::index_t i = 0; i < mesh_geo.vertices.nb(); i++) {
            base_type::Vector3 p = get_mesh_point(mesh_geo, i);
            base_type::Vertex *v = (base_type::Vertex *) base_type::Vertex::allocate_from_pool(&m.vertex_pool, p);
        }
        for (GEO::index_t i = 0; i < mesh_geo.facets.nb(); i++) {
            assert(mesh_geo.facets.nb_vertices(i) == 3);

            GEO::index_t p1_index = mesh_geo.facets.vertex(i, 0);
            GEO::index_t p2_index = mesh_geo.facets.vertex(i, 1);
            GEO::index_t p3_index = mesh_geo.facets.vertex(i, 2);

            base_type::Vertex *v_p1 = (base_type::Vertex *) m.vertex_pool[p1_index];
            base_type::Vertex *v_p2 = (base_type::Vertex *) m.vertex_pool[p2_index];
            base_type::Vertex *v_p3 = (base_type::Vertex *) m.vertex_pool[p3_index];

            base_type::Face *f = (base_type::Face *) base_type::Face::allocate_from_pool(&m.face_pool, v_p1, v_p2, v_p3);
        }

        m.connect_edge_by_face();
    }

    void get_tet_mesh(base_type::Tetrahedra_Mesh &m, GEO::Mesh &mesh_geo) {
        for (GEO::index_t i = 0; i < mesh_geo.vertices.nb(); i++) {
            base_type::Vector3 p = get_mesh_point(mesh_geo, i);
            base_type::Vertex *v = (base_type::Vertex *) base_type::Vertex::allocate_from_pool(&m.vertex_pool, p);
        }
        for (GEO::index_t i = 0; i < mesh_geo.cells.nb(); i++) {
            assert(mesh_geo.cells.type(i) == GEO::MESH_TET);

            GEO::index_t p1_index = mesh_geo.cells.vertex(i, 0);
            GEO::index_t p2_index = mesh_geo.cells.vertex(i, 1);
            GEO::index_t p3_index = mesh_geo.cells.vertex(i, 2);
            GEO::index_t p4_index = mesh_geo.cells.vertex(i, 3);

            base_type::Vertex *v_p1 = (base_type::Vertex *) m.vertex_pool[p1_index];
            base_type::Vertex *v_p2 = (base_type::Vertex *) m.vertex_pool[p2_index];
            base_type::Vertex *v_p3 = (base_type::Vertex *) m.vertex_pool[p3_index];
            base_type::Vertex *v_p4 = (base_type::Vertex *) m.vertex_pool[p4_index];

            base_type::Tetrahedra *t = (base_type::Tetrahedra *) base_type::Tetrahedra::allocate_from_pool(&m.tet_pool, v_p1, v_p2, v_p3, v_p4);
        }

    }
}


#endif //TETGEO_GEOGRAM_PROXY_H
