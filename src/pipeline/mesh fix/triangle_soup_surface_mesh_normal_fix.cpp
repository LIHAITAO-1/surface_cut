//
// Created by xmyci on 27/02/2024.
//
#include "triangle_soup_surface_mesh_normal_fix.h"


void triangle_soup_surface_mesh_normal_fix(base_type::Triangle_Soup_Mesh &mesh) {
    using namespace base_type;
    using namespace Geometrical_Predicates;

    auto get_disjoin_face = [&mesh](Face *f, int i) -> Face * {
        auto e = f->disjoin_edge[i];
        if (e->connect_face_array->size() < 2) {
            std::string msg("input mesh:" + mesh.file_path + " has hole! unable to fix normal, abort! the hold is nearby face index:" +
                            std::to_string(f->static_index) +
                            " , position:(" + std::to_string(f->p1->position.x) +
                            "," + std::to_string(f->p1->position.y) + "," + std::to_string(f->p1->position.z) + ")"
            );
            ASSERT_MSG(e->connect_face_array->size() == 2, msg);
            return nullptr;
        }

        if ((*e->connect_face_array)[0] != f)
            return (*e->connect_face_array)[0];

        return (*e->connect_face_array)[1];
    };

    auto change_neighbor_face_normal = [&mesh, get_disjoin_face](Face *f, int i) -> Face * {

        auto f_neighbor = get_disjoin_face(f, i);
        ASSERT_MSG (f_neighbor != nullptr, "input mesh:" + mesh.file_path + " has hole!");

        if (f_neighbor->mark == false) {
            f_neighbor->mark = true;
            auto e_share = f->disjoin_edge[i];

            auto non_share_vtx = i == 0 ? f->p1 : (i == 1 ? f->p2 : f->p3);
            auto non_share_vtx_neighbor = e_share == f_neighbor->disjoin_edge[0] ? f_neighbor->p1 : (e_share == f_neighbor->disjoin_edge[1] ? f_neighbor->p2 : f_neighbor->p3);

            auto f_normal = cross(f->p2->position - f->p1->position, f->p3->position - f->p1->position);
            auto f_normal_neighbor = cross(f_neighbor->p2->position - f_neighbor->p1->position, f_neighbor->p3->position - f_neighbor->p1->position);

            auto f_o = cross((non_share_vtx->position - e_share->end->position), (e_share->orig->position - e_share->end->position)); //non -> share
            auto f_o_neighbor = cross((e_share->orig->position - e_share->end->position), (non_share_vtx_neighbor->position - e_share->end->position));//share-> non

            auto ori = dot(f_o, f_normal) * dot(f_o_neighbor, f_normal_neighbor);
            if (ori < 0) {//share_edge -> non_share_edge
                std::swap(f_neighbor->p1, f_neighbor->p3);
                std::swap(f_neighbor->disjoin_edge[0], f_neighbor->disjoin_edge[2]);
            }
        }
        return f_neighbor;
    };

    auto need_add_to_stack = [get_disjoin_face](Face *f) {
        bool b1 = true, b2 = true, b3 = true;

        auto f1 = get_disjoin_face(f, 0);
        if (f1 != nullptr)
            b1 = f1->mark;

        auto f2 = get_disjoin_face(f, 1);
        if (f2 != nullptr)
            b2 = f2->mark;

        auto f3 = get_disjoin_face(f, 2);
        if (f3 != nullptr)
            b3 = f3->mark;
        return b1 && b2 && b3;
    };

    auto march_neighbor_face = [need_add_to_stack, change_neighbor_face_normal](std::vector<Face *> &stack) -> void {

        auto f = stack.back();
        stack.pop_back();

        auto f_0 = change_neighbor_face_normal(f, 0);
        if (need_add_to_stack(f_0) == false)
            stack.push_back(f_0);
        auto f_1 = change_neighbor_face_normal(f, 1);
        if (need_add_to_stack(f_1) == false)
            stack.push_back(f_1);
        auto f_2 = change_neighbor_face_normal(f, 2);
        if (need_add_to_stack(f_2) == false)
            stack.push_back(f_2);

    };
    //fix_normal
    auto f_start = (Face *) mesh.face_pool[0];
    f_start->mark = true;
    std::vector<Face *> stack;
    stack.push_back(f_start);
    while (stack.size() > 0)
        march_neighbor_face(stack);

    //Evaluate the wd
    //Divergence theorem   https://en.wikipedia.org/wiki/Divergence_theorem
    //https://math.stackexchange.com/questions/689418/how-to-compute-surface-normal-pointing-out-of-the-object
    double sum = 0;
    for (auto i = 0; i < mesh.face_pool.size(); i++) {
        auto f = (Face *) mesh.face_pool[i];
        double f_area = get_triangle_area({f->p1->position, f->p2->position, f->p3->position});
        Vector3 n = base_type::Face::get_face_normal(f);
        Vector3 mid = (f->p1->position + f->p2->position + f->p3->position) / 3;
        sum += mid.x * n.x * f_area;
    }
    if (sum > 0)
        for (auto i = 0; i < mesh.face_pool.size(); i++) {
            auto f = (Face *) mesh.face_pool[i];
            std::swap(f->p1, f->p3);
            std::swap(f->disjoin_edge[0], f->disjoin_edge[2]);
        }

}