//
// Created by xmyci on 27/02/2024.
//
#include "run_pipeline.h"
#include "TriIntersectionTri.h"
#include "geogram/mesh/mesh_AABB.h"
#include <vector>
#include <geogram/mesh/mesh_remesh.h>
#include "proxy/geogram/geogram_proxy.h"

using namespace std;
using namespace base_type;
//using namespace Geometrical_Predicates;

enum Vtx_state {
    Inside,
    OnEdge,
    OnVtx
};


enum Tri_tri_cut_case {
    P1_P2_Inside = 1,

    P1_Inside_P2_OnEdge,
    P1_OnEdge_P2_Inside,

    P1_P2_OnEdge,

    P1_OnVtx_P2_Inside,
    P1_Inside_P2_OnVtx,

    P1_OnVtx_P2_OnEdge,
    P1_OnEdge_P2_OnVtx,

    P1_P2_OnVtx,
    No_Cut
};

struct Cut_result {
    Tri_tri_cut_case cut_case;
    Vertex *c_v[2];
    Edge *c_e[2];
};

void end_2_end() {
    auto get_vtx_state = [](double a, double b) {
        if ((a + b) < 1 - 1e-6 && a > 1e-6 && b > 1e-6) {
            return Inside;
        }
        if ((abs(a) < 1e-6 && abs(b) < 1e-6) ||
            (abs(abs(a) - 1) < 1e-6 && abs(b) < 1e-6) ||
            (abs(abs(b) - 1) < 1e-6 && abs(a) < 1e-6) ){
            return OnVtx;
        }
        if (abs(a) < 1e-6 || abs(b) < 1e-6 || abs((a + b) - 1) < 1e-6) {
            return OnEdge;
        }
    };

    auto point_uv_calulate_triangle = [](Triangle3d tri, base_type::Vector3 intersection) -> std::pair<double, double> {
        auto u = tri.p2 - tri.p1;
        auto v = tri.p3 - tri.p1;
        auto Q = tri.p1;

        base_type::Vector3 cvu = cross(u, v);

        base_type::Vector3 planar_hitpt_vector = intersection - Q;
        auto w = cvu / dot(cvu, cvu);
        auto alpha = dot(w, cross(planar_hitpt_vector, v));
        auto beta = dot(w, cross(u, planar_hitpt_vector));

        return {alpha, beta};
    };

    auto edge_split = [](Triangle_Soup_Mesh &mesh, base_type::Edge *e, const base_type::Vector3 &p) -> std::pair<Vertex *, std::array<Edge *, 2> > {

        //     orig
        //     /|\            /|\
        //    / | \          / | \
        //   /f1|f2\        /  |  \
        //  /   |   \  ->  /___|___\
        //  \   |   /      \   |   /
        //   \  |  /        \  |  /
        //    \ | /          \ | /
        //     \|/            \|/
        //     end

        Vertex *new_vtx = Vertex::allocate_from_pool(&mesh.vertex_pool, p);

        if (e->connect_face_array->size() == 2){
            base_type::Face *f1 = (*(e->connect_face_array))[0];
            base_type::Face *f2 = (*(e->connect_face_array))[1];

            Vertex *v_orig = e->orig;
            Vertex *v_end = e->end;

            Vertex *v_t1 = base_type::Face::get_disjoin_face_no_share_vtx(f2, f1);
            Vertex *v_t2 = base_type::Face::get_disjoin_face_no_share_vtx(f1, f2);

//        Edge *e_t11 = f1->disjoin_edge[base_type::Face::get_vtx_index(f1, v_end)];
            Edge *e_t12 = f1->disjoin_edge[base_type::Face::get_vtx_index(f1, v_orig)];

//        Edge *e_t21 = f2->disjoin_edge[base_type::Face::get_vtx_index(f2, v_end)];
            Edge *e_t22 = f2->disjoin_edge[base_type::Face::get_vtx_index(f2, v_orig)];

            //new
            Face *new_f1 = Face::allocate_from_pool(&mesh.face_pool, v_t1, v_end, new_vtx);
            Face *new_f2 = Face::allocate_from_pool(&mesh.face_pool, v_t2, v_end, new_vtx);

            Edge *new_e0 = Edge::allocate_from_pool(&mesh.edge_pool, new_vtx, v_end);
            Edge *new_e1 = Edge::allocate_from_pool(&mesh.edge_pool, new_vtx, v_t1);
            Edge *new_e2 = Edge::allocate_from_pool(&mesh.edge_pool, new_vtx, v_t2);

            //change e
            e->end = new_vtx;

            //change f1 and f2
            Face::set_vtx(f1, new_vtx, Face::get_vtx_index(f1, v_end));
            Face::set_vtx(f2, new_vtx, Face::get_vtx_index(f2, v_end));

            f1->disjoin_edge[Face::get_vtx_index(f1, v_orig)] = new_e1;
            f2->disjoin_edge[Face::get_vtx_index(f2, v_orig)] = new_e2;

            //change new face and edge
            (*(new_e0->connect_face_array)).push_back(new_f1);
            (*(new_e0->connect_face_array)).push_back(new_f2);
            (*(new_e1->connect_face_array)).push_back(f1);
            (*(new_e1->connect_face_array)).push_back(new_f1);
            (*(new_e2->connect_face_array)).push_back(f2);
            (*(new_e2->connect_face_array)).push_back(new_f2);

            new_f1->disjoin_edge[0] = new_e0;
            new_f2->disjoin_edge[0] = new_e0;
            new_f1->disjoin_edge[1] = new_e1;
            new_f2->disjoin_edge[1] = new_e2;
            new_f1->disjoin_edge[2] = e_t12;
            new_f2->disjoin_edge[2] = e_t22;

            Edge::del_connect_face(e_t12, f1);
            Edge::del_connect_face(e_t22, f2);
            Edge::add_connect_face(e_t12, new_f1);
            Edge::add_connect_face(e_t22, new_f2);

            std::array<Edge *, 2> new_edge({new_e0, e});

            return {new_vtx, new_edge};
        }
        if (e->connect_face_array->size() == 1){
            base_type::Face *f1 = (*(e->connect_face_array))[0];

            Vertex *v_orig = e->orig;
            Vertex *v_end = e->end;
            Vertex *v_t1;

            if (f1->p1 != v_orig && f1->p1 != v_end){
                v_t1 = f1->p1;
            }
            if (f1->p2 != v_orig && f1->p2 != v_end){
                v_t1 = f1->p2;
            }
            if (f1->p3 != v_orig && f1->p3 != v_end){
                v_t1 = f1->p3;
            }

            //new
            Face *new_f1 = Face::allocate_from_pool(&mesh.face_pool, v_t1, v_orig, new_vtx);
            Face *new_f2 = Face::allocate_from_pool(&mesh.face_pool, v_t1, v_end, new_vtx);

            Edge *new_e0 = Edge::allocate_from_pool(&mesh.edge_pool, new_vtx, v_orig);
            Edge *new_e1 = Edge::allocate_from_pool(&mesh.edge_pool, new_vtx, v_end);
            Edge *new_e2 = Edge::allocate_from_pool(&mesh.edge_pool, new_vtx, v_t1);

            (*(new_e0->connect_face_array)).push_back(new_f1);
            (*(new_e1->connect_face_array)).push_back(new_f2);
            (*(new_e2->connect_face_array)).push_back(new_f1);
            (*(new_e2->connect_face_array)).push_back(new_f2);

            Edge *edge1 = Face::get_edge_from_two_vertex(f1, v_orig, v_t1);//f1->disjoin_edge[Face::get_vtx_index(f1, v_end)];
            Edge *edge2 = Face::get_edge_from_two_vertex(f1, v_end, v_t1);//f1->disjoin_edge[Face::get_vtx_index(f1, v_orig)];

            new_f1->disjoin_edge[0] = new_e0;
            new_f1->disjoin_edge[1] = new_e2;
            new_f1->disjoin_edge[2] = edge1;

            new_f2->disjoin_edge[0] = new_e1;
            new_f2->disjoin_edge[1] = new_e2;
            new_f2->disjoin_edge[2] = edge2;

            Edge::del_connect_face(edge1, f1);
            Edge::del_connect_face(edge2, f1);
            Edge::add_connect_face(edge1, new_f1);
            Edge::add_connect_face(edge2, new_f2);

//            new_f1->mark = true;
//            new_f2->mark = true;

            //delete
            mesh.face_pool.deallocate(f1);
            mesh.edge_pool.deallocate(e);

            std::array<Edge *, 2> new_edge({new_e2, e});

            return {new_vtx, new_edge};
        }
        if (e->connect_face_array->size() == 0){
            assert(false);
        }
    };

    auto tri_split = [get_vtx_state, point_uv_calulate_triangle, &edge_split](Triangle_Soup_Mesh &mesh, base_type::Face *f, base_type::Vector3 &p, Face *&f1_new, Face *&f2_new, Face *&f3_new, Vertex *&new_vtx) {
        double alpha, beta;

        std::tie(alpha, beta) = point_uv_calulate_triangle({f->p1->position, f->p2->position, f->p3->position}, p);
        Vtx_state vtx_state = get_vtx_state(alpha, beta);

        if (vtx_state == OnEdge) {
            std::array<Edge *, 2> new_e1;
            Edge *e = abs(1 - alpha - beta) < 1e-6 ? f->disjoin_edge[0] : (abs(alpha) < 1e-6 ? f->disjoin_edge[1] : f->disjoin_edge[2]);
            std::tie(new_vtx, new_e1) = edge_split(mesh, e, p);
        } else if (vtx_state == Inside) {

            Vertex *p1 = f->p1;
            Vertex *p2 = f->p2;
            Vertex *p3 = f->p3;

            Edge *e1 = f->disjoin_edge[0];
            Edge *e2 = f->disjoin_edge[1];
            Edge *e3 = f->disjoin_edge[2];

            //creat
            new_vtx = Vertex::allocate_from_pool(&mesh.vertex_pool, p);

            f1_new = Face::allocate_from_pool(&mesh.face_pool, new_vtx, p2, p3);
            f2_new = Face::allocate_from_pool(&mesh.face_pool, new_vtx, p3, p1);
            f3_new = Face::allocate_from_pool(&mesh.face_pool, new_vtx, p1, p2);

            Edge *e1_new = Edge::allocate_from_pool(&mesh.edge_pool, new_vtx, p1);
            Edge *e2_new = Edge::allocate_from_pool(&mesh.edge_pool, new_vtx, p2);
            Edge *e3_new = Edge::allocate_from_pool(&mesh.edge_pool, new_vtx, p3);

            //change
            Edge::del_connect_face(e1, f);
            Edge::del_connect_face(e2, f);
            Edge::del_connect_face(e3, f);

            Edge::add_connect_face(e1, f1_new);
            Edge::add_connect_face(e2, f2_new);
            Edge::add_connect_face(e3, f3_new);

            Edge::add_connect_face(e1_new, f2_new);
            Edge::add_connect_face(e1_new, f3_new);

            Edge::add_connect_face(e2_new, f3_new);
            Edge::add_connect_face(e2_new, f1_new);

            Edge::add_connect_face(e3_new, f1_new);
            Edge::add_connect_face(e3_new, f2_new);

            f1_new->disjoin_edge[0] = e1;
            f1_new->disjoin_edge[1] = e3_new;
            f1_new->disjoin_edge[2] = e2_new;

            f2_new->disjoin_edge[0] = e2;
            f2_new->disjoin_edge[1] = e1_new;
            f2_new->disjoin_edge[2] = e3_new;

            f3_new->disjoin_edge[0] = e3;
            f3_new->disjoin_edge[1] = e2_new;
            f3_new->disjoin_edge[2] = e1_new;

            f1_new->mark = true;
            f2_new->mark = true;
            f3_new->mark = true;

            //delete
            mesh.face_pool.deallocate(f);

        } else {
            assert(false);
        }
    };

    auto tri_tri_cut = [&](base_type::Face *f1, base_type::Face *f2, base_type::Vector3 &p1, base_type::Vector3 &p2) -> bool {
        Triangle tri1(f1->p1->position, f1->p2->position, f1->p3->position);
        Triangle tri2(f2->p1->position, f2->p2->position, f2->p3->position);

        vector<Vector3> pts;
        if (ComputeLineWithTwoTriangle(tri1, tri2, pts)) {
            p1 = pts[0];
            p2 = pts[1];
            return true;
        } else
            return false;
    };

    auto clear_all_tri_mark = [](Triangle_Soup_Mesh &mesh) {
        for (int i = 0; i < mesh.face_pool.size(); i++) {
            base_type::Face *f_mesh = (base_type::Face *) mesh.face_pool[i];
            f_mesh->mark = false;
        }
    };

    auto tri_mark = [](std::vector<Face *> &f_array) {
        for (auto f: f_array) {
            f->mark = true;
        }
    };

    auto get_cut_result = [point_uv_calulate_triangle, get_vtx_state](base_type::Face *f_insert, const base_type::Vector3 &p1, const base_type::Vector3 &p2) -> Cut_result {

        double alpha_p1, beta_p1;
        double alpha_p2, beta_p2;
        std::tie(alpha_p1, beta_p1) = point_uv_calulate_triangle({f_insert->p1->position, f_insert->p2->position, f_insert->p3->position}, p1);
        std::tie(alpha_p2, beta_p2) = point_uv_calulate_triangle({f_insert->p1->position, f_insert->p2->position, f_insert->p3->position}, p2);

        Cut_result cut_result;

        Vtx_state p1_state = get_vtx_state(alpha_p1, beta_p1);
        Vtx_state p2_state = get_vtx_state(alpha_p2, beta_p2);

        // case 1 p1 and p2 inside f_mesh , split f_mesh
        if (p1_state == Inside && p2_state == Inside) {
            cut_result.cut_case = P1_P2_Inside;
        } else if (p1_state == Inside && p2_state == OnEdge) {
            cut_result.c_e[1] = abs(1 - alpha_p2 - beta_p2) < 1e-6 ? f_insert->disjoin_edge[0] : (abs(alpha_p2) < 1e-6 ? f_insert->disjoin_edge[1] : f_insert->disjoin_edge[2]);
            cut_result.cut_case = P1_Inside_P2_OnEdge;
        } else if (p1_state == OnEdge && p2_state == Inside) {
            cut_result.c_e[0] = abs(1 - alpha_p1 - beta_p1) < 1e-6 ? f_insert->disjoin_edge[0] : (abs(alpha_p1) < 1e-6 ? f_insert->disjoin_edge[1] : f_insert->disjoin_edge[2]);
            cut_result.cut_case = P1_OnEdge_P2_Inside;
        } else if (p1_state == OnEdge && p2_state == OnEdge) {
            cut_result.c_e[0] = abs(1 - alpha_p1 - beta_p1) < 1e-6 ? f_insert->disjoin_edge[0] : (abs(alpha_p1) < 1e-6 ? f_insert->disjoin_edge[1] : f_insert->disjoin_edge[2]);
            cut_result.c_e[1] = abs(1 - alpha_p2 - beta_p2) < 1e-6 ? f_insert->disjoin_edge[0] : (abs(alpha_p2) < 1e-6 ? f_insert->disjoin_edge[1] : f_insert->disjoin_edge[2]);
            cut_result.cut_case = P1_P2_OnEdge;
        } else if (p1_state == OnVtx && p2_state == Inside) {
            cut_result.c_v[0] = abs(alpha_p1 + beta_p1) < 1e-6 ? f_insert->p1 : (abs(1 - beta_p1) < 1e-6 ? f_insert->p3 : f_insert->p2);
            cut_result.cut_case = P1_OnVtx_P2_Inside;
        } else if (p1_state == Inside && p2_state == OnVtx) {
            cut_result.c_v[1] = abs(alpha_p2 + beta_p2) < 1e-6 ? f_insert->p1 : (abs(1 - beta_p2) < 1e-6 ? f_insert->p3 : f_insert->p2);
            cut_result.cut_case = P1_Inside_P2_OnVtx;
        } else if (p1_state == OnVtx && p2_state == OnEdge) {
            cut_result.c_e[1] = abs(1 - alpha_p2 - beta_p2) < 1e-6 ? f_insert->disjoin_edge[0] : (abs(alpha_p2) < 1e-6 ? f_insert->disjoin_edge[1] : f_insert->disjoin_edge[2]);
            cut_result.c_v[0] = abs(alpha_p1 + beta_p1) < 1e-6 ? f_insert->p1 : (abs(1 - beta_p1) < 1e-6 ? f_insert->p3 : f_insert->p2);
            cut_result.cut_case = P1_OnVtx_P2_OnEdge;
        } else if (p1_state == OnEdge && p2_state == OnVtx) {
            cut_result.c_e[0] = abs(1 - alpha_p1 - beta_p1) < 1e-6 ? f_insert->disjoin_edge[0] : (abs(alpha_p1) < 1e-6 ? f_insert->disjoin_edge[1] : f_insert->disjoin_edge[2]);
            cut_result.c_v[1] = abs(alpha_p2 + beta_p2) < 1e-6 ? f_insert->p1 : (abs(1 - beta_p2) < 1e-6 ? f_insert->p3 : f_insert->p2);
            cut_result.cut_case = P1_OnEdge_P2_OnVtx;
        } else if (p1_state == OnVtx && p2_state == OnVtx) {
            cut_result.c_v[0] = abs(alpha_p1 + beta_p1) < 1e-6 ? f_insert->p1 : (abs(1 - beta_p1) < 1e-6 ? f_insert->p3 : f_insert->p2);
            cut_result.c_v[1] = abs(alpha_p2 + beta_p2) < 1e-6 ? f_insert->p1 : (abs(1 - beta_p2) < 1e-6 ? f_insert->p3 : f_insert->p2);
            cut_result.cut_case = P1_P2_OnVtx;
        } else {
            assert(false);
        }
        return cut_result;
    };

    auto insert_one_tri = [&](Triangle_Soup_Mesh &mesh, base_type::Face *f_insert) {
        clear_all_tri_mark(mesh);

        insert_start:
        for (int i = 0; i < mesh.face_pool.size(); i++) {
            if (i == 3)
                int aaaa = 0;

            base_type::Face *f_mesh = (base_type::Face *) mesh.face_pool[i];
            if (f_mesh->mark == true) {
                continue;
            } else {
                f_mesh->mark = true;
            }

            base_type::Vector3 p1;
            base_type::Vector3 p2;
            if (!tri_tri_cut(f_mesh, f_insert, p1, p2)) {
                continue;
            }

            auto c_r = get_cut_result(f_mesh, p1, p2);

            switch (c_r.cut_case) {
                case P1_P2_Inside : {
                    Vertex *new_v1, *new_v2;
                    Face *f1, *f2, *f3;
                    Face *f11, *f22, *f33;

                    tri_split(mesh, f_mesh, p1, f1, f2, f3, new_v1);
                    Triangle tri = Triangle(f1->p1->position, f1->p2->position, f1->p3->position);
                    bool flag = true;
                    if (InTriangle(tri, p2) != -1 && flag){
                        tri_split(mesh, f1, p2, f11, f22, f33, new_v2);
                        flag = false;
                    }
                    tri = Triangle(f2->p1->position, f2->p2->position, f2->p3->position);
                    if (InTriangle(tri, p2) != -1 && flag){
                        tri_split(mesh, f2, p2, f11, f22, f33, new_v2);
                        flag = false;
                    }
                    tri = Triangle(f3->p1->position, f3->p2->position, f3->p3->position);
                    if (InTriangle(tri, p2) != -1 && flag){
                        tri_split(mesh, f3, p2, f11, f22, f33, new_v2);
                        flag = false;
                    }

                    auto edge_find = Edge::find_edge(&mesh.edge_pool, new_v1, new_v2);
                    edge_find->special = true;
                    //tri_mark(*(edge_find->connect_face_array));

                    new_v1->special = true;
                    new_v2->special = true;
                    break;
                }
                case P1_Inside_P2_OnEdge:{
                    Vertex *new_v1, *new_v2;
                    Face *f1, *f2, *f3;
                    std::array<Edge *, 2> new_e1;

                    tri_split(mesh, f_mesh, p1, f1, f2, f3, new_v1);
                    std::tie(new_v2, new_e1) = edge_split(mesh, c_r.c_e[0], p2);

                    auto edge_find = Edge::find_edge(&mesh.edge_pool, new_v1, new_v2);
                    edge_find->special = true;
                    tri_mark(*(edge_find->connect_face_array));
                    new_v1->special = true;
                    new_v2->special = true;
                    break;
                }
                case P1_OnEdge_P2_Inside: {
                    Vertex *new_v1, *new_v2;
                    Face *f1, *f2, *f3;
                    std::array<Edge *, 2> new_e1;

                    tri_split(mesh, f_mesh, p2, f1, f2, f3, new_v1);
                    std::tie(new_v2, new_e1) = edge_split(mesh, c_r.c_e[0], p1);

                    auto edge_find = Edge::find_edge(&mesh.edge_pool, new_v1, new_v2);
                    edge_find->special = true;
                    tri_mark(*(edge_find->connect_face_array));
                    new_v1->special = true;
                    new_v2->special = true;
                    break;
                }

                case P1_P2_OnEdge: {
                    Vertex *new_v1, *new_v2;
                    std::array<Edge *, 2> new_e1, new_e2;

                    std::tie(new_v1, new_e1) = edge_split(mesh, c_r.c_e[0], p1);
                    std::tie(new_v2, new_e2) = edge_split(mesh, c_r.c_e[1], p2);
                    auto edge_find = Edge::find_edge(&mesh.edge_pool, new_v1, new_v2);
                    edge_find->special = true;
                    tri_mark(*(edge_find->connect_face_array));
                    new_v1->special = true;
                    new_v2->special = true;
                    break;
                }

                case P1_OnVtx_P2_Inside:{
                    Vertex *new_v1, *new_v2;
                    Face *f1, *f2, *f3;

                    if (p1.distance(f_mesh->p1->position) < 1e-6){
                        new_v1 = f_mesh->p1;
                    }
                    if (p1.distance(f_mesh->p2->position) < 1e-6){
                        new_v1 = f_mesh->p2;
                    }
                    if (p1.distance(f_mesh->p3->position) < 1e-6){
                        new_v1 = f_mesh->p3;
                    }

                    tri_split(mesh, f_mesh, p2, f1, f2, f3, new_v2);
                    auto edge_find = Edge::find_edge(&mesh.edge_pool, new_v1, new_v2);
                    edge_find->special = true;
//                    tri_mark(*(edge_find->connect_face_array));
                    new_v1->special = true;
                    new_v2->special = true;
                    break;
                }
                case P1_Inside_P2_OnVtx: {
                    Vertex *new_v1, *new_v2;
                    Face *f1, *f2, *f3;

                    if (p2.distance(f_mesh->p1->position) < 1e-6){
                        new_v2 = f_mesh->p1;
                    }
                    if (p2.distance(f_mesh->p2->position) < 1e-6){
                        new_v2 = f_mesh->p2;
                    }
                    if (p2.distance(f_mesh->p3->position) < 1e-6){
                        new_v2 = f_mesh->p3;
                    }

                    tri_split(mesh, f_mesh, p1, f1, f2, f3, new_v1);
                    auto edge_find = Edge::find_edge(&mesh.edge_pool, new_v1, new_v2);
                    edge_find->special = true;
//                    tri_mark(*(edge_find->connect_face_array));
                    new_v1->special = true;
                    new_v2->special = true;
                    break;
                }

                case P1_OnVtx_P2_OnEdge: {
                    Vertex *new_v;
                    std::array<Edge *, 2> new_e;
                    std::tie(new_v, new_e) = edge_split(mesh, c_r.c_e[1], p2);
                    auto edge_find = Edge::find_edge(&mesh.edge_pool, new_v, c_r.c_v[0]);
                    edge_find->special = true;
                    tri_mark(*(edge_find->connect_face_array));
                    new_v->special = true;
                    break;
                }
                case P1_OnEdge_P2_OnVtx: {
                    Vertex *new_v;
                    std::array<Edge *, 2> new_e;
                    std::tie(new_v, new_e) = edge_split(mesh, c_r.c_e[0], p1);
                    auto edge_find = Edge::find_edge(&mesh.edge_pool, new_v, c_r.c_v[1]);
                    edge_find->special = true;
                    tri_mark(*(edge_find->connect_face_array));
                    edge_find->special = true;
                    new_v->special = true;
                    break;
                }

                case P1_P2_OnVtx: {
                    Vertex *new_v1, *new_v2;
                    new_v1 = c_r.c_v[0];
                    new_v2 = c_r.c_v[1];
                    auto edge_find = Edge::find_edge(&mesh.edge_pool, new_v1, new_v2);
                    tri_mark(*(edge_find->connect_face_array));
                    edge_find->special = true;

                    new_v1->special = true;
                    new_v2->special = true;
                    break;
                }

                case No_Cut: {
                    break;
                }
                default:
                    assert(false);
            }

            goto insert_start;
        }
    };

    logger().info("Step 1: Compute Point");
    Triangle_Soup_Mesh meshCube;
    Triangle_Soup_Mesh meshCube2;
    Triangle_Soup_Mesh meshCurve;
    Triangle_Soup_Mesh meshCurve2;

    meshCube.load_from_file("D:/xmy/model/8-2.obj");
    meshCube2.load_from_file("D:/xmy/model/8-2.obj");
    meshCurve.load_from_file("D:/xmy/model/fm38.obj");
//    meshCurve2.load_from_file("D:/xmy/model/curve3.obj");

    //step 1: use meshCurve to subdivide meshCube

    for (int i = 0; i < meshCurve.face_pool.size(); i++) {
        auto f = (base_type::Face *) meshCurve.face_pool[i];
        insert_one_tri(meshCube, f);
    }

    meshCube.save("D:/xmy/model", "output");

    for (int i = 0; i < meshCube2.face_pool.size(); i++) {
    //for (int i = 0; i < 100; i++) {
        auto f = (base_type::Face *) meshCube2.face_pool[i];
        insert_one_tri(meshCurve, f);
    }

//    for (int i = 0; i < meshCube.edge_pool.size(); i++) {
//        auto e = (Edge *) meshCube.edge_pool[i];
//        if (e->orig->special && e->end->special)
//            e->special = true;
//    }

//    for (int i = 0; i < meshCurve.edge_pool.size(); i++) {
//        auto e = (Edge *) meshCurve.edge_pool[i];
//        if (e->orig->special && e->end->special)
//            e->special = true;
//    }

    meshCurve.save("D:/xmy/model", "outputCurve");

    //step 2: depart mesh by special edge
    auto get_unmarked_face = [](Triangle_Soup_Mesh &mesh) -> Face * {
        for (int i = 0; i < mesh.face_pool.size(); i++) {
            base_type::Face *f = (base_type::Face *) mesh.face_pool[i];
            if (f->mark == false)
                return f;
        }
        return nullptr;
    };

    clear_all_tri_mark(meshCube);
    clear_all_tri_mark(meshCurve);

    auto unmarked_face = get_unmarked_face(meshCube);
    auto unmarked_face_Curve = get_unmarked_face(meshCurve);

    Triangle_Soup_Mesh CutFace;
    int part_index = 0;
    do {
        std::vector<Face *> face_stack = {unmarked_face_Curve};
        std::vector<Face *> face_array;

        while (!face_stack.empty()) {
            auto back = face_stack.back();
            face_stack.pop_back();
            back->mark = true;
            face_array.push_back(back);
            for (int i = 0; i < 3; i++){
                if (back->disjoin_edge[i]->special == false && back->disjoin_edge[i]->connect_face_array->size() == 2) {
                    auto f = Face::get_disjoin_face(back, back->disjoin_edge[i]);
                    if (!f->mark)
                        face_stack.push_back(f);
                }
            }
        }

        unmarked_face_Curve = get_unmarked_face(meshCurve);

        Triangle_Soup_Mesh part;

        for (auto f: face_array) {
            auto v1 = part.add_vtx(f->p1->position);
            auto v2 = part.add_vtx(f->p2->position);
            auto v3 = part.add_vtx(f->p3->position);
            part.add_face(v1, v2, v3);
//            auto vv1 = CutFace.add_vtx(f->p1->position);
//            auto vv2 = CutFace.add_vtx(f->p2->position);
//            auto vv3 = CutFace.add_vtx(f->p3->position);
//            CutFace.add_face(v1, v2, v3);
        }
        part.save("D:/xmy/model", "output_Curve" + std::to_string(part_index++));
    } while (unmarked_face_Curve);

    meshCurve2.load_from_file("D:/xmy/model/output_Curve1.vtu");
//    meshCurve2.save("D:/xmy/model", "output_xxx");

    part_index = 0;
    do {
        std::vector<Face *> face_stack = {unmarked_face};
        std::vector<Face *> face_array;

        while (!face_stack.empty()) {
            auto back = face_stack.back();
            face_stack.pop_back();
            back->mark = true;
            face_array.push_back(back);
            for (int i = 0; i < 3; i++){
                if (back->disjoin_edge[i]->special == false) {
                    auto f = Face::get_disjoin_face(back, back->disjoin_edge[i]);
                    if (!f->mark)
                        face_stack.push_back(f);
                }
            }
        }

        unmarked_face = get_unmarked_face(meshCube);

        Triangle_Soup_Mesh part;

        for (auto f: face_array) {
            auto v1 = part.add_vtx(f->p1->position);
            auto v2 = part.add_vtx(f->p2->position);
            auto v3 = part.add_vtx(f->p3->position);
            part.add_face(v1, v2, v3);
        }

        for (int i = 0; i < meshCurve2.face_pool.size(); i++){
            auto f = (base_type::Face *) meshCurve2.face_pool[i];
            auto v1 = part.add_vtx(f->p1->position);
            auto v2 = part.add_vtx(f->p2->position);
            auto v3 = part.add_vtx(f->p3->position);
            part.add_face(v1, v2, v3);
        }
        part.save("D:/xmy/model", "output_Cube" + std::to_string(part_index++));
    } while (unmarked_face);

    logger().info("end");

}



