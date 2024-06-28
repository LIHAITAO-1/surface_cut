#pragma once

#include <vector>
#include <set>
#include <cassert>
#include <random>
#include <iostream>
#include <string>
#include <cmath>
#include <stack>
#include <map>
#include <unordered_map>
#include <set>
#include <unordered_set>

//#include <Eigen/Sparse>

#include "basic/data structure/memory_pool.h"
#include "basic/math/vector3.h"
#include "basic/geometrical predicates/predicates_wrapper.h"
#include "utils/mesh loader/mesh_loader.h"
#include "utils/log/logger.h"
#include "macro.h"


typedef unsigned uint;

namespace base_type {
    using namespace Geometrical_Predicates;
    struct Edge;
    struct Tetrahedra;
    struct Face;

    struct Vertex {
        Vector3 position;
        std::vector<Tetrahedra *> *connect_tetrahedra_array;
        std::vector<Face *> *connect_face_array;
        std::vector<Edge *> *connect_edge_array;
        uint static_index;
        bool special;
        int type;
        double data;

        static Vertex *allocate_from_pool(MemoryPool *pool) {
            auto v = (Vertex *) pool->allocate();
            v->connect_tetrahedra_array = new std::vector<Tetrahedra *>();
            v->connect_edge_array = new std::vector<Edge *>();
            v->connect_face_array = new std::vector<Face *>();
            v->static_index = pool->size() - 1;
            v->special = false;
            v->type = 0;
            return v;
        }

        static Vertex *allocate_from_pool(MemoryPool *pool, Vector3 position) {
            auto v = (Vertex *) pool->allocate();
            v->position = position;
            v->connect_tetrahedra_array = new std::vector<Tetrahedra *>();
            v->connect_edge_array = new std::vector<Edge *>();
            v->connect_face_array = new std::vector<Face *>();
            v->static_index = pool->size() - 1;
            v->special = false;
            v->type = 0;
            return v;
        }

        static void get_position_array(MemoryPool *pool, std::vector<base_type::Vector3> &array) {

            array.resize((*pool).size());
            for (int i = 0; i < (*pool).size(); i++) {
                auto v = (Vertex *) (*pool)[i];
                array[i] = v->position;
            }
        }


    };

    struct Edge {
        Vertex *orig;
        Vertex *end;
        std::vector<Face *> *connect_face_array;

        bool special;

        static Edge *allocate_from_pool(MemoryPool *pool, Vertex *_orig, Vertex *_end) {
            auto e = (Edge *) pool->allocate();
            e->orig = _orig;
            e->end = _end;
            e->special = false;
            _orig->connect_edge_array->push_back(e);
            _end->connect_edge_array->push_back(e);

            e->connect_face_array = new std::vector<Face *>();
            return e;
        }

        static bool is_connected(Edge *e1, Edge *e2, Vertex *&v) {
            if (e1->orig == e2->orig || e1->orig == e2->end) {
                v = e1->orig;
                return true;
            }

            if (e1->end == e2->orig || e1->end == e2->end) {
                v = e1->end;
                return true;
            }

            return false;
        }


        static bool is_connected(Edge *e1, Edge *e2) {
            if (e1->orig == e2->orig || e1->orig == e2->end) {

                return true;
            }

            if (e1->end == e2->orig || e1->end == e2->end) {

                return true;
            }

            return false;
        }

        static void del_connect_face(Edge *e, Face *f) {
            auto iter = std::find((*(e->connect_face_array)).begin(), (*(e->connect_face_array)).end(), f);
            if (iter != (*(e->connect_face_array)).end())
                (*(e->connect_face_array)).erase(iter);
            else
                assert(false);

        }

        static void add_connect_face(Edge *e, Face *f) {
            auto iter = std::find((*(e->connect_face_array)).begin(), (*(e->connect_face_array)).end(), f);
            if (iter == (*(e->connect_face_array)).end())

                (*(e->connect_face_array)).push_back(f);
            else
                assert(false);
        }

        static Edge *find_edge(MemoryPool *pool, Vertex *v1, Vertex *v2) {
            for (int i = 0; i < pool->size(); i++) {
                Edge *e = (Edge *) (*pool)[i];
                if ((e->orig == v1 || e->end == v1) && (e->orig == v2 || e->end == v2)) {
                    return e;
                }
            }
            return nullptr;
        }

    };

    struct Face {
        int face_type;
        Vertex *p1;
        Vertex *p2;
        Vertex *p3;
        Tetrahedra *disjoin_tet[2];//0 is ccw , 1 is cw
        Edge *disjoin_edge[3];
        bool is_boundary_face = false;
        bool draw_read = false;
        bool mark = false;
        int static_index;


        static Face *allocate_from_pool(MemoryPool *pool, Vertex *_p1, Vertex *_p2, Vertex *_p3) {
            auto f = (Face *) pool->allocate();
            f->disjoin_tet[0] = nullptr;
            f->disjoin_tet[1] = nullptr;
            f->disjoin_edge[0] = nullptr;
            f->disjoin_edge[1] = nullptr;
            f->disjoin_edge[2] = nullptr;
            f->p1 = _p1;
            f->p2 = _p2;
            f->p3 = _p3;
            f->is_boundary_face = true;
            f->draw_read = false;
            f->mark = false;
            f->face_type = 0;
            _p1->connect_face_array->push_back(f);
            _p2->connect_face_array->push_back(f);
            _p3->connect_face_array->push_back(f);
            f->static_index = pool->size() - 1;
            return f;
        }

        static void get_static_index_array(MemoryPool *pool, std::vector<uint> &array) {
            array.resize((*pool).size() * 3);
            for (int i = 0; i < (*pool).size(); i++) {
                auto f = (Face *) (*pool)[i];
                array[i * 3] = f->p1->static_index;
                array[i * 3 + 1] = f->p2->static_index;
                array[i * 3 + 2] = f->p3->static_index;
            }
        }

        static Face *get_disjoin_face(Face *f, int index) {
            Edge *e = f->disjoin_edge[index];
            auto f1 = (*(e->connect_face_array))[0];
            auto f2 = (*(e->connect_face_array))[1];
            assert(f1 != nullptr && f2 != nullptr);

            if (f1 == f)
                return f2;
            if (f2 == f)
                return f1;
            assert(false);
            return nullptr;
        }

        static Face *get_disjoin_face(Face *f, Edge *e) {
            auto f1 = (*(e->connect_face_array))[0];
            auto f2 = (*(e->connect_face_array))[1];
            assert(f1 != nullptr && f2 != nullptr);

            if (f1 == f)
                return f2;
            if (f2 == f)
                return f1;
            assert(false);
            return nullptr;
        }

        static Edge *get_share_edge(Face *f1, Face *f2) {
            assert(is_disjoin_face(f1, f2));
            bool b1 = f2->p1 == f1->p1 || f2->p1 == f1->p2 || f2->p1 == f1->p3;
            bool b2 = f2->p2 == f1->p1 || f2->p2 == f1->p2 || f2->p2 == f1->p3;
            bool b3 = f2->p3 == f1->p1 || f2->p3 == f1->p2 || f2->p3 == f1->p3;

            if (b1 == false)
                return f2->disjoin_edge[0];
            if (b2 == false)
                return f2->disjoin_edge[1];
            if (b3 == false)
                return f2->disjoin_edge[2];

            assert(false);
            return nullptr;
        }

        static std::pair<Edge *, Edge *> get_non_share_edge(Face *f1, Face *f2) {
            assert(is_disjoin_face(f1, f2));
            bool b1 = f2->p1 == f1->p1 || f2->p1 == f1->p2 || f2->p1 == f1->p3;
            bool b2 = f2->p2 == f1->p1 || f2->p2 == f1->p2 || f2->p2 == f1->p3;
            bool b3 = f2->p3 == f1->p1 || f2->p3 == f1->p2 || f2->p3 == f1->p3;

            if (b1 == false)
                return {f2->disjoin_edge[1], f2->disjoin_edge[2]};
            if (b2 == false)
                return {f2->disjoin_edge[0], f2->disjoin_edge[2]};
            if (b3 == false)
                return {f2->disjoin_edge[1], f2->disjoin_edge[2]};
            assert(false);
            return {nullptr, nullptr};
        }

        static void set_vtx(Face *f, Vertex *v, int index) {
            assert(index < 3 && index >= 0);
            switch (index) {
                case 0: {
                    f->p1 = v;
                    break;
                }
                case 1: {
                    f->p2 = v;
                    break;
                }
                default: {
                    f->p3 = v;
                    break;
                }
            }
        }

        static int get_vtx_index(Face *f, Vertex *v) {
            if (f->p1 == v)
                return 0;
            if (f->p2 == v)
                return 1;
            if (f->p3 == v)
                return 2;
            assert(false);
            return -1;
        }

        static Vertex *get_disjoin_face_no_share_vtx(Face *f1, Face *f2) {
            if (is_disjoin_face(f1, f2) == false) {
                assert(false);
            }

            bool b1 = f2->p1 == f1->p1 || f2->p1 == f1->p2 || f2->p1 == f1->p3;
            bool b2 = f2->p2 == f1->p1 || f2->p2 == f1->p2 || f2->p2 == f1->p3;
            bool b3 = f2->p3 == f1->p1 || f2->p3 == f1->p2 || f2->p3 == f1->p3;

            if (b1 == false)
                return f2->p1;
            if (b2 == false)
                return f2->p2;
            if (b3 == false)
                return f2->p3;
            assert(false);
            return nullptr;
        }

        static bool is_disjoin_face(Face *f1, Face *f2) {
            bool b1 = f1->p1 == f2->p1 || f1->p1 == f2->p2 || f1->p1 == f2->p3;
            bool b2 = f1->p2 == f2->p1 || f1->p2 == f2->p2 || f1->p2 == f2->p3;
            bool b3 = f1->p3 == f2->p1 || f1->p3 == f2->p2 || f1->p3 == f2->p3;
            int i = 0;
            if (b1)
                i++;
            if (b2)
                i++;
            if (b3)
                i++;
            return i == 2;
        }

        static Vector3 get_face_normal(Face *f) {
            //left hand cross
            auto u = f->p2->position - f->p1->position;
            auto v = f->p3->position - f->p1->position;
            auto n = v.cross(u).normalise();
            return n;
        }

        static bool is_connected(Face *f, Edge *e) {
            int i = 0;

            if (e->orig == f->p1 || e->orig == f->p2 || e->orig == f->p3) {
                i++;
            }
            if (e->end == f->p1 || e->end == f->p2 || e->end == f->p3) {
                i++;
            }
            return i == 2;
        }

        Vertex *get_vtx(int i) {

            if (i == 0)
                return p1;

            if (i == 1)
                return p2;

            if (i == 2)
                return p3;

            assert(false);
            return nullptr;
        }

        static bool edge_equel(Edge *e1, Vertex *v1, Vertex *v2) {
            assert(e1 != nullptr && v1 != nullptr && v2 != nullptr);
            if ((e1->orig == v1 && e1->end == v2) ||
                (e1->orig == v2 && e1->end == v1)) {
                return true;
            }
            return false;
        }

        static Edge *get_edge_from_two_vertex(Face *f, Vertex *v1, Vertex *v2) {
            if (f->disjoin_edge[0] == nullptr || f->disjoin_edge[1] == nullptr || f->disjoin_edge[2] == nullptr)
                return nullptr;
            if (edge_equel(f->disjoin_edge[0], v1, v2))
                return f->disjoin_edge[0];
            if (edge_equel(f->disjoin_edge[1], v1, v2))
                return f->disjoin_edge[1];
            if (edge_equel(f->disjoin_edge[2], v1, v2))
                return f->disjoin_edge[2];
        }

        static void face_swap(Face *f1, Face *f2) {
            static int times = 0;
            times++;
            logger().warn("find zero face!");

            assert(is_disjoin_face(f1, f2));
            auto share_edge = get_share_edge(f1, f2);

            auto [non_share_f1_e0, non_share_f1_e1] = get_non_share_edge(f2, f1);
            auto [non_share_f2_e0, non_share_f2_e1] = get_non_share_edge(f1, f2);
            if (Edge::is_connected(non_share_f1_e0, non_share_f2_e0) == false)
                std::swap(non_share_f2_e0, non_share_f2_e1);

            auto share_vtx_e0 = share_edge->orig;
            auto share_vtx_e1 = share_edge->end;
            Vertex *v_connect;
            assert(Edge::is_connected(non_share_f1_e0, non_share_f2_e0, v_connect));
            if (v_connect != share_vtx_e0)
                std::swap(share_vtx_e0, share_vtx_e1);

            auto vtx_no_share_in_f1 = get_disjoin_face_no_share_vtx(f2, f1);
            auto vtx_no_share_in_f2 = get_disjoin_face_no_share_vtx(f1, f2);

            //swep edge
            share_edge->orig = vtx_no_share_in_f1;
            share_edge->end = vtx_no_share_in_f2;

            //re set face vtx
            {
                auto index = get_vtx_index(f1, share_vtx_e1);
                set_vtx(f1, vtx_no_share_in_f2, index);

                index = get_vtx_index(f2, share_vtx_e0);
                set_vtx(f2, vtx_no_share_in_f1, index);
            }

            //re connect edge adn face
            {
                auto index = get_vtx_index(f1, share_vtx_e0);
                f1->disjoin_edge[index] = share_edge;

                index = get_vtx_index(f1, vtx_no_share_in_f1);
                f1->disjoin_edge[index] = non_share_f2_e0;

                index = get_vtx_index(f2, share_vtx_e1);
                f2->disjoin_edge[index] = share_edge;

                index = get_vtx_index(f2, vtx_no_share_in_f2);
                f2->disjoin_edge[index] = non_share_f1_e1;
            }

        }
    };

    struct Tetrahedra {
        std::vector<int> *type_array;
        std::vector<double> *data_array;
        int static_index;
        Vertex *p1;
        Vertex *p2;
        Vertex *p3;
        Vertex *p4;

        Tetrahedra *neighbors[4];
        Face *faces[4];
        bool mark;


        //for draw
        bool draw_red = false;


        static Tetrahedra *allocate_from_pool(MemoryPool *pool, Vertex *_p1, Vertex *_p2, Vertex *_p3, Vertex *_p4) {
            auto t = (Tetrahedra *) pool->allocate();
            t->p1 = _p1;
            t->p2 = _p2;
            t->p3 = _p3;
            t->p4 = _p4;
            t->type_array = new std::vector<int>;
            t->data_array = new std::vector<double>;

            t->static_index = (*pool).size() - 1;
            t->neighbors[0] = nullptr;
            t->neighbors[1] = nullptr;
            t->neighbors[2] = nullptr;
            t->neighbors[3] = nullptr;

            t->faces[0] = nullptr;
            t->faces[1] = nullptr;
            t->faces[2] = nullptr;
            t->faces[3] = nullptr;

            t->mark = false;
            t->draw_red = false;

            t->p1->connect_tetrahedra_array->push_back(t);
            t->p2->connect_tetrahedra_array->push_back(t);
            t->p3->connect_tetrahedra_array->push_back(t);
            t->p4->connect_tetrahedra_array->push_back(t);

            return t;
        }

        static void get_static_index_array(MemoryPool *pool, std::vector<uint> &array) {
            array.resize((*pool).size() * 4);
            for (int i = 0; i < (*pool).size(); i++) {
                auto f = (Tetrahedra *) (*pool)[i];
                array[i * 4] = f->p1->static_index;
                array[i * 4 + 1] = f->p2->static_index;
                array[i * 4 + 2] = f->p3->static_index;
                array[i * 4 + 3] = f->p4->static_index;
            }
        }

        Vertex *get_vtx(int i) {
            if (i == 0)
                return p1;
            if (i == 1)
                return p2;
            if (i == 2)
                return p3;
            if (i == 3)
                return p4;

            assert(false);
            return nullptr;
        }

        static bool is_tetrahedra_disjoin(Tetrahedra *t1, Tetrahedra *t2) {
            int i = 0;
            if (t1->p1 == t2->p1 || t1->p1 == t2->p2 || t1->p1 == t2->p3 || t1->p1 == t2->p4)
                i++;
            if (t1->p2 == t2->p1 || t1->p2 == t2->p2 || t1->p2 == t2->p3 || t1->p2 == t2->p4)
                i++;
            if (t1->p3 == t2->p1 || t1->p3 == t2->p2 || t1->p3 == t2->p3 || t1->p3 == t2->p4)
                i++;
            if (t1->p4 == t2->p1 || t1->p4 == t2->p2 || t1->p4 == t2->p3 || t1->p4 == t2->p4)
                i++;

            return i == 3;

        }

        static int find_disjoin_tet_non_share_vtx_index_in_t2(Tetrahedra *t1, Tetrahedra *t2) {
            bool is_vtx1_share = 0, is_vtx2_share = 0, is_vtx3_share = 0, is_vtx4_share = 0;
            is_vtx1_share = (t2->p1 == t1->p1 || t2->p1 == t1->p2 || t2->p1 == t1->p3 || t2->p1 == t1->p4);
            is_vtx2_share = (t2->p2 == t1->p1 || t2->p2 == t1->p2 || t2->p2 == t1->p3 || t2->p2 == t1->p4);
            is_vtx3_share = (t2->p3 == t1->p1 || t2->p3 == t1->p2 || t2->p3 == t1->p3 || t2->p3 == t1->p4);
            is_vtx4_share = (t2->p4 == t1->p1 || t2->p4 == t1->p2 || t2->p4 == t1->p3 || t2->p4 == t1->p4);

            int i = 0;
            int value = 0;
            if (is_vtx1_share)
                i++;
            else
                value = 0;

            if (is_vtx2_share)
                i++;
            else
                value = 1;

            if (is_vtx3_share)
                i++;
            else
                value = 2;

            if (is_vtx4_share)
                i++;
            else
                value = 3;

            assert(i == 3);

            return value;
        }

        static void bind_tet_and_face(Tetrahedra *t, Face *f) {
            bool b1 = t->p1 == f->p1 || t->p1 == f->p2 || t->p1 == f->p3;
            bool b2 = t->p2 == f->p1 || t->p2 == f->p2 || t->p2 == f->p3;
            bool b3 = t->p3 == f->p1 || t->p3 == f->p2 || t->p3 == f->p3;
            bool b4 = t->p4 == f->p1 || t->p4 == f->p2 || t->p4 == f->p3;
            int i = 0;
            if (b1)
                i++;
            if (b2)
                i++;
            if (b3)
                i++;
            if (b4)
                i++;
            assert(i == 3);
            assert(t->static_index >= 0);
            assert(f->static_index >= 0);
            bool orient;
            if (!b1) {
                orient = Geometrical_Predicates::toleft(f->p1->position, f->p2->position, f->p3->position,
                                                        t->p1->position);
                t->faces[0] = f;
            }
            if (!b2) {
                orient = toleft(f->p1->position, f->p2->position, f->p3->position, t->p2->position);
                t->faces[1] = f;
            }
            if (!b3) {
                orient = toleft(f->p1->position, f->p2->position, f->p3->position, t->p3->position);
                t->faces[2] = f;
            }
            if (!b4) {
                orient = toleft(f->p1->position, f->p2->position, f->p3->position, t->p4->position);
                t->faces[3] = f;
            }
            orient ? f->disjoin_tet[0] = t : f->disjoin_tet[1] = t;
        }
    };

    struct Triangle_Soup_Mesh {
        std::string file_path;

        MemoryPool vertex_pool;
        MemoryPool edge_pool;
        MemoryPool face_pool;

//        Triangle_Soup_Mesh(Triangle_Soup_Mesh& mesh){
//            this->file_path = mesh.file_path;
//            MemoryPool a(mesh.face_pool);
//            face_pool(a);
//        }


        Triangle_Soup_Mesh() {
            vertex_pool.initializePool(sizeof(Vertex), 1000, 8, 32);
            edge_pool.initializePool(sizeof(Edge), 1000, 8, 32);
            face_pool.initializePool(sizeof(Face), 1000 * 1.2, 8, 32);
        }


        void clear() {
            vertex_pool.restart();
            edge_pool.restart();
            face_pool.restart();
        }

        bool is_manifold_2() {
            bool b1 = (face_pool.size() * 3 == edge_pool.size() * 2);
            bool b2 = (face_pool.size() - edge_pool.size() + vertex_pool.size() == 2);
            return b1 && b2;
        }

        bool is_closed() {
            for (int i = 0; i < edge_pool.size(); i++) {
                auto e = (Edge *) edge_pool[i];
                if (e->connect_face_array->size() != 2)
                    return false;
            }
            return true;
        }

        void connect_edge_by_face() {
            auto add_edge_if_not_exist = [](MemoryPool &pool, Vertex *a, Vertex *b) -> Edge * {
                for (auto &e: *a->connect_edge_array) {
                    if ((e->end == a && e->orig == b) || (e->orig == a && e->end == b)) {
                        return e;
                    }
                }
                for (auto &e: *b->connect_edge_array) {
                    if ((e->end == a && e->orig == b) || (e->orig == a && e->end == b)) {
                        return e;
                    }
                }
                auto e = Edge::allocate_from_pool(&pool, a, b);

                return e;
            };
            assert(edge_pool.size() == 0);
            for (int i = 0; i < face_pool.size(); i++) {
                Face *f = (Face *) face_pool[i];

                auto e_23 = add_edge_if_not_exist(edge_pool, f->p2, f->p3);
                e_23->connect_face_array->push_back(f);
                f->disjoin_edge[0] = e_23;

                auto e_13 = add_edge_if_not_exist(edge_pool, f->p1, f->p3);
                e_13->connect_face_array->push_back(f);
                f->disjoin_edge[1] = e_13;

                auto e_12 = add_edge_if_not_exist(edge_pool, f->p1, f->p2);
                e_12->connect_face_array->push_back(f);
                f->disjoin_edge[2] = e_12;
            }
        }

        Vertex *add_vtx(Vector3 p) {
            auto find_same_vtx_in_pool = [](MemoryPool &pool, Vector3 p, int &index) -> Vertex * {
                for (int i = 0; i < pool.size(); i++) {
                    auto v = (Vertex *) pool[i];
                    if (vector_length_sqr(p, v->position) < 1e-8) {
                        index = i;
                        return v;
                    }

                }
                return nullptr;
            };
            int index;
            Vertex *vtx_find = find_same_vtx_in_pool(vertex_pool, p, index);
            if (vtx_find == nullptr) {
                auto v = Vertex::allocate_from_pool(&vertex_pool, p);
                return v;

            }
            return vtx_find;

        }

        Face *add_face(Vertex *v1, Vertex *v2, Vertex *v3) {
            auto find_same_face_in_pool = [](MemoryPool &pool, Vertex *p1, Vertex *p2, Vertex *p3) -> Face * {
                for (int i = 0; i < pool.size(); i++) {
                    auto f = (Face *) pool[i];
                    bool b1 = f->p1 == p1 || f->p1 == p2 || f->p1 == p3;
                    bool b2 = f->p2 == p1 || f->p2 == p2 || f->p2 == p3;
                    bool b3 = f->p3 == p1 || f->p3 == p2 || f->p3 == p3;
                    if (b1 && b2 && b3)
                        return f;
                }
                return nullptr;
            };
            Face *face_find = find_same_face_in_pool(face_pool, v1, v2, v3);

            if (face_find == nullptr) {
                assert(v1 != v2);
                assert(v1 != v3);
                assert(v2 != v3);


                return Face::allocate_from_pool(&face_pool, v1, v2, v3);

            }
            return face_find;
        }

        void load_from_file(std::string path, bool check_manifold_2 = false) {
            file_path = path;
            Mesh_Loader::FileData data;
            Mesh_Loader::load_by_extension(path.c_str(), data);

            std::unordered_map<double, std::vector<Vector3>> hashPoint;
            std::map<int, int> vtx_relocation_map;
            int index = 0;
            for (int i = 0; i < data.numberOfPoints; i++) {
                Vector3 p(data.pointList[i * 3], data.pointList[i * 3 + 1], data.pointList[i * 3 + 2]);

                double key = (double)round((p.x + p.y + p.z) * 1e6) / 1e6 ;
                if (hashPoint[key].size() == 0) {
                    hashPoint[key].push_back(p);
                    auto v = Vertex::allocate_from_pool(&vertex_pool);
                    v->position = p;
                    vtx_relocation_map[i] = vertex_pool.size() - 1;
                    index++;
                }
                else {
                    int len = 0;
                    for (int num = 0; num < hashPoint[key].size(); num++) {
                        if (vector_length_sqr(p, hashPoint[key][num]) < 1e-8) {
                            vtx_relocation_map[i] = index;
                            break;
                        }
                        else
                            len++;      
                    }
                    if (len == hashPoint[key].size()) {
                        hashPoint[key].push_back(p);
                        auto v = Vertex::allocate_from_pool(&vertex_pool);
                        v->position = p;
                        vtx_relocation_map[i] = vertex_pool.size() - 1;
                        index++;
                    }                      
                }
                  
                //auto find_same_vtx_in_pool = [](MemoryPool &pool, Vector3 p, int &index) -> Vertex * {
                //    for (int i = 0; i < pool.size(); i++) {
                //        auto v = (Vertex *) pool[i];
                //        if (vector_length_sqr(p, v->position) < 1e-8) {
                //            index = i;
                //            return v;
                //        }

                //    }
                //    return nullptr;
                //};
                //int index;
                //Vertex *vtx_find = find_same_vtx_in_pool(vertex_pool, p, index);
                //if (vtx_find == nullptr) {
                //    auto v = Vertex::allocate_from_pool(&vertex_pool);
                //    v->position = p;
                //    vtx_relocation_map[i] = vertex_pool.size() - 1;
                //} else {
                //    vtx_relocation_map[i] = index;
                //}
            }

            std::unordered_map<int, std::vector<std::vector<int>>> hashCell;

            for (int i = 0; i < data.numberOfCell; i++) {
                auto cell = data.cellList[i];
                auto p1_index = vtx_relocation_map[cell.pointList[0]];
                auto p2_index = vtx_relocation_map[cell.pointList[1]];
                auto p3_index = vtx_relocation_map[cell.pointList[2]];

                auto p1 = (Vertex *) vertex_pool[p1_index];
                auto p2 = (Vertex *) vertex_pool[p2_index];
                auto p3 = (Vertex *) vertex_pool[p3_index];

                if ((p1 == p2) || (p1 == p3) || (p2 == p3)) {
                    continue;
                }

                int key = p1_index + p2_index + p3_index;
                std::vector<int> Value;
                Value.clear();
                Value.push_back(p1_index);
                Value.push_back(p2_index);
                Value.push_back(p3_index);

                if (hashCell[key].size() == 0) {
                    hashCell[key].push_back(Value);
                    Face::allocate_from_pool(&face_pool, p1, p2, p3);
                }
                else {
                    int len = 0;
                    for (int num = 0; num < hashCell[key].size(); num++) {
                        std::vector<int> a;
                        a.push_back(hashCell[key][num][0]);
                        a.push_back(hashCell[key][num][1]);
                        a.push_back(hashCell[key][num][2]);
                        std::vector<int> b;
                        b.push_back(p1_index);
                        b.push_back(p2_index);
                        b.push_back(p3_index);
                        
                        sort(a.begin(), a.end());
                        sort(b.begin(), b.end());

                        if (a[0] == b[0] && a[1] == b[1] && a[2] == b[2]) {
                            break;
                        }
                        else
                            len++;
                    }
                    if (len == hashCell[key].size()) {
                        hashCell[key].push_back(Value);
                        Face::allocate_from_pool(&face_pool, p1, p2, p3);
                    }
                }

                //auto find_same_face_in_pool = [](MemoryPool &pool, Vertex *p1, Vertex *p2, Vertex *p3) -> Face * {
                //    for (int i = 0; i < pool.size(); i++) {
                //        auto f = (Face *) pool[i];
                //        bool b1 = f->p1 == p1 || f->p1 == p2 || f->p1 == p3;
                //        bool b2 = f->p2 == p1 || f->p2 == p2 || f->p2 == p3;
                //        bool b3 = f->p3 == p1 || f->p3 == p2 || f->p3 == p3;
                //        if (b1 && b2 && b3)
                //            return f;
                //    }
                //    return nullptr;
                //};

                //Face *face_find = find_same_face_in_pool(face_pool, p1, p2, p3);//这个地方可以优化，在顶点处保存相连面

                //if (face_find == nullptr) {


                //    //ASSERT_MSG(colinear(p1->position, p2->position, p3->position) == false, "find colinear triangle face");
                //    Face::allocate_from_pool(&face_pool, p1, p2, p3);
                //}
            }

            connect_edge_by_face();
            remove_zero_face();
        }

        void remove_zero_face() {
            for (int i = 0; i < face_pool.size(); i++) {
                auto f = (base_type::Face *) face_pool[i];
                if (colinear(f->p1->position, f->p2->position, f->p3->position)) {
                    auto l1 = vector_length_sqr(f->p2->position - f->p3->position);
                    auto l2 = vector_length_sqr(f->p1->position - f->p3->position);
                    auto l3 = vector_length_sqr(f->p2->position - f->p1->position);
                    auto max = std::max({l1, l2, l3});

                    if (l1 == max) {
                        base_type::Face::face_swap(f, base_type::Face::get_disjoin_face(f, 0));
                    }
                    if (l2 == max) {
                        base_type::Face::face_swap(f, base_type::Face::get_disjoin_face(f, 1));
                    }
                    if (l3 == max) {
                        base_type::Face::face_swap(f, base_type::Face::get_disjoin_face(f, 2));
                    }
                }
            }
            //update face static index
        }

        void save(std::string save_path, std::string save_name, bool bSave_vtu = false, bool bSave_obj = true) {
            //save file
            ASSERT_MSG(bSave_vtu || bSave_obj, "set at least one file extension be TRUE");
            using namespace Mesh_Loader;

            FileData data;

            data.set_point_number(vertex_pool.size());

            //find
            int special_edge_num = 0;
            //for (int j = 0; j < edge_pool.size(); j++) {
            //    auto e = (Edge *) edge_pool[j];
            //    if (e->special)
            //        special_edge_num++;
            //}

            data.set_cell_number(face_pool.size() + special_edge_num);
//            data.set_edge_number(edge_pool.size());

            for (int j = 0; j < vertex_pool.size(); j++) {
                const auto &vtx = (Vertex *) vertex_pool[j];
                data.pointList[j * 3] = vtx->position.x;
                data.pointList[j * 3 + 1] = vtx->position.y;
                data.pointList[j * 3 + 2] = vtx->position.z;
                //export point type
                data.pointDataInt["type"].content.push_back(vtx->type);
            }

            for (int j = 0; j < face_pool.size(); j++) {
                const auto &f = (Face *) face_pool[j];

                data.cellList[j].pointList = new int[3];
                data.cellList[j].numberOfPoints = 3;

                data.cellList[j].pointList[0] = f->p1->static_index;
                data.cellList[j].pointList[1] = f->p2->static_index;
                data.cellList[j].pointList[2] = f->p3->static_index;
            }

            int index = face_pool.size();

            //for (int j = 0; j < edge_pool.size(); j++) {
            //    const auto &e = (Edge *) edge_pool[j];
            //    if (e->special == false)
            //        continue;
            //    data.cellList[index].pointList = new int[2];
            //    data.cellList[index].numberOfPoints = 2;

            //    data.cellList[index].pointList[0] = e->orig->static_index;
            //    data.cellList[index].pointList[1] = e->end->static_index;
            //    index++;
            //}


            auto full_path = path_join(save_path, save_name + ".obj");
            //if (bSave_vtu)
            //    ASSERT_MSG(save_vtu(full_path.c_str(), data), "save fail");
            //full_path = path_join(save_path, save_name + ".obj");

            if (bSave_obj)
                ASSERT_MSG(save_obj(full_path.c_str(), data), "save fail");

        }

        double get_wind_number(Vector3 p) {
            double val = 0;
            for (int i = 0; i < face_pool.size(); i++) {
                val += get_wind_number_face_i(p, i);
            }
            return val;
        }

        double get_wind_number_face_i(Vector3 p, int face_index) {
            auto f = (Face *) face_pool[face_index];
            auto a = f->p1->position - p;
            auto b = f->p2->position - p;
            auto c = f->p3->position - p;

            auto a_len = vector_length(a);
            auto b_len = vector_length(b);
            auto c_len = vector_length(c);

            auto ab_dot = dot(a, b);
            auto bc_dot = dot(b, c);
            auto ca_dot = dot(c, a);

            double a11, a12, a13;
            double a21, a22, a23;
            double a31, a32, a33;

            a11 = a.x;
            a12 = b.x;
            a13 = c.x;
            a21 = a.y;
            a22 = b.y;
            a23 = c.y;
            a31 = a.z;
            a32 = b.z;
            a33 = c.z;

            double det_abc = a11 * a22 * a33 + a12 * a23 * a31 + a13 * a21 * a32 - a31 * a22 * a13 - a32 * a23 * a11 -
                             a21 * a12 * a33;

            double denominator = a_len * b_len * c_len + ab_dot * c_len + bc_dot * a_len + ca_dot * b_len;

            return 2 * atan2(det_abc, denominator);
        }

        void aabb() {

        }

    };

    struct Tetrahedra_Mesh {
        MemoryPool vertex_pool;
        MemoryPool tet_pool;
        MemoryPool face_pool;                                                 //used for export .msh
        std::map<std::string, std::vector<double>> additional_tet_date_double;//used for quality measure
        std::map<std::string, std::vector<int>> additional_tet_date_int;      //used for tete classfy

        Tetrahedra_Mesh() {
            vertex_pool.initializePool(sizeof(Vertex), 1000, 8, 32);
            tet_pool.initializePool(sizeof(Tetrahedra), 1000 * 1.2, 8, 32);
            face_pool.initializePool(sizeof(Face), 1000 * 1.2, 8, 32);
        }

        void clear() {
            vertex_pool.restart();
            tet_pool.restart();
            face_pool.restart();
        }

        void AABB(base_type::Vector3 &max, base_type::Vector3 &min) {
            ASSERT_MSG(vertex_pool.size() > 0, "empty mesh");
            base_type::Vertex *first_vtx = (base_type::Vertex *) vertex_pool[0];
            base_type::Vertex *EP_x_min = first_vtx;
            base_type::Vertex *EP_x_max = first_vtx;
            base_type::Vertex *EP_y_min = first_vtx;
            base_type::Vertex *EP_y_max = first_vtx;
            base_type::Vertex *EP_z_min = first_vtx;
            base_type::Vertex *EP_z_max = first_vtx;

            for (int i = 1; i < vertex_pool.size(); i++) {
                base_type::Vertex *vtx = (base_type::Vertex *) vertex_pool[i];
                if (vtx->position.x < EP_x_min->position.x)
                    EP_x_min = vtx;
                if (vtx->position.x > EP_x_max->position.x)
                    EP_x_max = vtx;

                if (vtx->position.y < EP_y_min->position.y)
                    EP_y_min = vtx;
                if (vtx->position.y > EP_y_max->position.y)
                    EP_y_max = vtx;

                if (vtx->position.z < EP_z_min->position.z)
                    EP_z_min = vtx;
                if (vtx->position.z > EP_z_max->position.z)
                    EP_z_max = vtx;
            }

            max = {EP_x_max->position.x, EP_y_max->position.y, EP_z_max->position.z};
            min = {EP_x_min->position.x, EP_y_min->position.y, EP_z_min->position.z};
        }

        bool load_from_file(std::string path) {
            Mesh_Loader::FileData data;
            if (Mesh_Loader::load_by_extension(path.c_str(), data) == false || data.numberOfPoints == 0) {
                return false;
            }

            for (int i = 0; i < data.numberOfPoints; i++) {
                base_type::Vertex::allocate_from_pool(&vertex_pool, {data.pointList[i * 3], data.pointList[i * 3 + 1],
                                                                     data.pointList[i * 3 + 2]});
            }
            for (int i = 0; i < data.numberOfCell; i++) {
                auto &cell = data.cellList[i];
                assert(cell.numberOfPoints == 4);
                base_type::Vertex *p1 = (base_type::Vertex *) vertex_pool[cell.pointList[0]];
                base_type::Vertex *p2 = (base_type::Vertex *) vertex_pool[cell.pointList[1]];
                base_type::Vertex *p3 = (base_type::Vertex *) vertex_pool[cell.pointList[2]];
                base_type::Vertex *p4 = (base_type::Vertex *) vertex_pool[cell.pointList[3]];

                bool orient = toleft(p1->position, p2->position, p3->position, p4->position);
                if (!orient) {
                    //                  fprintf(fout, "%c T4 %d %d %d %d %d\n", 'Z', tet_index++, tet->p1->static_index + 1, tet->p2->static_index + 1, tet->p3->static_index + 1, tet->p4->static_index + 1);
                } else {
                    std::swap(p1, p2);
                    //                  fprintf(fout, "%c T4 %d %d %d %d %d\n", 'Z', tet_index++, tet->p2->static_index + 1, tet->p1->static_index + 1, tet->p3->static_index + 1, tet->p4->static_index + 1);
                };

                base_type::Tetrahedra::allocate_from_pool(&tet_pool, p1, p2, p3, p4);
            }
            for (auto iter = data.cellDataInt.begin(); iter != data.cellDataInt.end(); iter++) {
                additional_tet_date_int[iter->first] = iter->second.content;
            }
            update_tet_neightbors();
            create_face_and_edge();
            return true;
        }

        void create_face_and_edge() {
            for (int i = 0; i < tet_pool.size(); i++) {
                auto create_tet_face = [this](int tet_face_index, base_type::Tetrahedra *t) {
                    if (t->faces[tet_face_index] == nullptr) {
                        if (t->neighbors[tet_face_index] != nullptr) {
                            //find adjface_vtx_index
                            auto face_index = base_type::Tetrahedra::find_disjoin_tet_non_share_vtx_index_in_t2(t,
                                                                                                                t->neighbors[tet_face_index]);
                            auto face_in_neighbor = t->neighbors[tet_face_index]->faces[face_index];
                            if (face_in_neighbor != nullptr) {
                                base_type::Tetrahedra::bind_tet_and_face(t, face_in_neighbor);
                                return;
                            }
                        }
                        base_type::Face *f;
                        if (tet_face_index == 0) {
                            f = base_type::Face::allocate_from_pool(&face_pool, t->p2, t->p3, t->p4);
                        } else if (tet_face_index == 1) {
                            f = base_type::Face::allocate_from_pool(&face_pool, t->p1, t->p3, t->p4);
                        } else if (tet_face_index == 2) {
                            f = base_type::Face::allocate_from_pool(&face_pool, t->p1, t->p2, t->p4);
                        } else if (tet_face_index == 3) {
                            f = base_type::Face::allocate_from_pool(&face_pool, t->p1, t->p2, t->p3);
                        } else {
                            assert(false);
                        }
                        base_type::Tetrahedra::bind_tet_and_face(t, f);
                    }
                };
                auto t = (base_type::Tetrahedra *) tet_pool[i];
                //create face
                create_tet_face(0, t);
                create_tet_face(1, t);
                create_tet_face(2, t);
                create_tet_face(3, t);
            }
        }

        void update_tet_neightbors() {

            for (int i = 0; i < tet_pool.size(); i++) {
                for (int j = 0; j < 4; j++) {
                    auto t = (base_type::Tetrahedra *) tet_pool[i];
                    t->neighbors[j] = nullptr;
                }
            }

            for (int i = 0; i < tet_pool.size(); i++) {
                auto t1 = (base_type::Tetrahedra *) tet_pool[i];

                for (int j = 0; j < 4; j++) {
                    auto vtx = t1->get_vtx(j);
                    for (auto t2: *vtx->connect_tetrahedra_array) {
                        if (t1 == t2)
                            continue;

                        if (base_type::Tetrahedra::is_tetrahedra_disjoin(t1, t2)) {
                            //mark t1 t2 as neighbor
                            int k = base_type::Tetrahedra::find_disjoin_tet_non_share_vtx_index_in_t2(t1, t2);
                            t2->neighbors[k] = t1;

                            k = base_type::Tetrahedra::find_disjoin_tet_non_share_vtx_index_in_t2(t2, t1);
                            t1->neighbors[k] = t2;
                        }
                    }
                }
            }
        }

        void save_as_vtu(std::string save_path, std::string save_name) {
            //save file
            using namespace Mesh_Loader;
            FileData data;
            data.set_point_number(vertex_pool.size());
            for (int j = 0; j < vertex_pool.size(); j++) {
                const auto &vtx = (Vertex *) vertex_pool[j];
                data.pointList[j * 3] = vtx->position.x;
                data.pointList[j * 3 + 1] = vtx->position.y;
                data.pointList[j * 3 + 2] = vtx->position.z;
            }

            data.set_cell_number(tet_pool.size());
            for (int j = 0; j < tet_pool.size(); j++) {
                const auto &t = (Tetrahedra *) tet_pool[j];

                data.cellList[j].pointList = new int[4];
                data.cellList[j].numberOfPoints = 4;

                assert(t->p1->static_index >= 0);
                assert(t->p2->static_index >= 0);
                assert(t->p3->static_index >= 0);
                assert(t->p4->static_index >= 0);

                data.cellList[j].pointList[0] = t->p1->static_index;
                data.cellList[j].pointList[1] = t->p2->static_index;
                data.cellList[j].pointList[2] = t->p3->static_index;
                data.cellList[j].pointList[3] = t->p4->static_index;
                for (auto &item: additional_tet_date_double) {
                    data.cellDataDouble[item.first].content.push_back(item.second[j]);
                }
            }

            //auto full_path = path_join(save_path, save_name + ".vtu");
            //ASSERT_MSG(save_vtu(full_path.c_str(), data), "save fail");
        }

        void slot_shrink(std::vector<int> &cell_class_array, std::map<int, int> &cell_class_map) {
            cell_class_array = additional_tet_date_int["excave"];

            for (int i = 0; i < cell_class_array.size(); i++) {
                if (cell_class_map.count(cell_class_array[i]) == 0)
                    cell_class_map[cell_class_array[i]] = 1;
                else {
                    cell_class_map[cell_class_array[i]]++;
                }
            }

        }

        bool save_as_analys_msh(std::string save_path, std::string save_name) {
            auto int_2_hex_string = [](int n) {
                char hex_string[20];
                sprintf(hex_string, "%x", n);
                return std::string(hex_string);
            };
            std::string out_str;
            std::string out_file_path = path_join(save_path, save_name + ".msh");
            FILE *fout = fopen(out_file_path.c_str(), "w");

            if (fout == (FILE *) NULL) {
                return false;
            }

            fprintf(fout, "(0 grid written by tetgeo)\n\n");
            fprintf(fout, "(2 3)\n\n");

            {
                fprintf(fout, "(0 face-based grid (ascii)\n");
                fprintf(fout, "   nodes:\t(10 (id start end type) (x y z ...))\n");
                fprintf(fout, "   edges:\t(11 (id start end type) (v-0 v-1 ....))\n");
                fprintf(fout, "   faces:\t(13 (id start end type etype)\n");
                fprintf(fout, "\t\t(v-0 v-1 .. v-n right-cell left-cell ...))\n");
                fprintf(fout, "   cells:\t(12 (id start end type etype))\n");
                fprintf(fout, "   parent-face:\t(59 (start end parent child) (nchilds child0 child1 ...))\n");
                fprintf(fout, "   face-pair:\t(18 (1 npairs parent shadow) (fp-0 fs-0 ...))\n");
                fprintf(fout, ")\n");
            }

            out_str = "(10 (0 1 " + int_2_hex_string(vertex_pool.size()) + " 0))\n";
            fprintf(fout, out_str.c_str());//total vtx
            out_str = "(13 (0 1 " + int_2_hex_string(face_pool.size()) + " 0))\n";
            fprintf(fout, out_str.c_str());//total face
            out_str = "(12 (0 1 " + int_2_hex_string(tet_pool.size()) + " 0))\n";
            fprintf(fout, out_str.c_str());//total tet

            //export vtx
            out_str = "(10 (1 1 " + int_2_hex_string(vertex_pool.size()) + " 2 3)(\n";
            fprintf(fout, out_str.c_str());
            for (int j = 0; j < vertex_pool.size(); j++) {
                const auto &vtx = (Vertex *) vertex_pool[j];
                fprintf(fout, "%g %g %g\n", vtx->position.x, vtx->position.y, vtx->position.z);
            }
            fprintf(fout, "))\n");

            //export face
            out_str = "(13 (1 1 " + int_2_hex_string(face_pool.size()) + " 3 3)(\n";
            fprintf(fout, out_str.c_str());
            for (int j = 0; j < face_pool.size(); j++) {
                auto f = (Face *) face_pool[j];
                int cr = 0, cl = 0;
                if (f->disjoin_tet[0] != nullptr) {
                    cr = f->disjoin_tet[0]->static_index + 1;
                }
                if (f->disjoin_tet[1] != nullptr) {
                    cl = f->disjoin_tet[1]->static_index + 1;
                }
                fprintf(fout, "%x %x %x %x %x\n", f->p1->static_index + 1, f->p2->static_index + 1,
                        f->p3->static_index + 1, cr, cl);
            }
            fprintf(fout, "))\n");


            //export cell
            std::vector<int> cell_class_array;
            std::map<int, int> cell_class_map;
            slot_shrink(cell_class_array, cell_class_map);
            //export tet by group
            int tet_index = 1;

            for (auto iter = cell_class_map.begin(); iter != cell_class_map.end(); iter++) {
                int class_id = iter->first;
                int class_size = iter->second;
                out_str = ("(12 (" + std::to_string(class_id + 2) + " " + int_2_hex_string(tet_index) + " " +
                           int_2_hex_string(class_size + tet_index - 1) + " 1 2)(\n");
                tet_index += class_size;
                fprintf(fout, out_str.c_str());
                for (int i = 0; i < cell_class_array.size(); i++) {
                    Tetrahedra *t = (Tetrahedra *) tet_pool[i];
                    if (cell_class_array[i] == class_id) {
                        fprintf(fout, "%x %x %x %x\n", t->p1->static_index + 1, t->p2->static_index + 1,
                                t->p3->static_index + 1, t->p4->static_index + 1);
                    }
                }
                fprintf(fout, "))\n");
            }


            fclose(fout);
            return true;
        }

        bool save_as_nastran_nas(std::string save_path, std::string save_name) {
            std::string out_str;
            std::string out_file_path = path_join(save_path, save_name + ".nas");
            FILE *fout = fopen(out_file_path.c_str(), "w");

            if (fout == (FILE *) NULL) {
                return false;
            }
            //export vtx
            fprintf(fout, "$ Nastran file written by tetgeo\n");
            fprintf(fout, "BEGIN BULK\n");
            for (int j = 0; j < vertex_pool.size(); j++) {
                const auto &vtx = (Vertex *) vertex_pool[j];
                fprintf(fout, "GRID*   %d                                  %e   %e\n*           %e\n",
                        vtx->static_index + 1, vtx->position.x, vtx->position.y, vtx->position.z);
            }

            std::vector<int> cell_class_array;
            std::map<int, int> cell_class_map;
            slot_shrink(cell_class_array, cell_class_map);
            for (int j = 0; j < tet_pool.size(); j++) {
                const auto &t = (Tetrahedra *) tet_pool[j];
                fprintf(fout, "CTETRA  %d      %d       %d    %d    %d    %d\n", t->static_index + 1,
                        cell_class_array[j], t->p1->static_index + 1, t->p2->static_index + 1, t->p3->static_index + 1,
                        t->p4->static_index + 1);
            }

            fprintf(fout, "ENDDATA");
            fclose(fout);
            return true;
        }

        bool save_as_stl(std::string save_path, std::string save_name) {
            std::string out_str;
            std::string out_file_path = path_join(save_path, save_name + ".stl");
            FILE *fout = fopen(out_file_path.c_str(), "w");

            if (fout == (FILE *) NULL) {
                return false;
            }


            fprintf(fout, "solid OBJECT\n");
            for (int i = 0; i < face_pool.size(); i++) {
                auto f = (Face *) face_pool[i];
                auto n = Face::get_face_normal(f);

                fprintf(fout, " facet normal %.16g %.16g %.16g\n", n.x, n.y, n.z);
                fprintf(fout, "     outer loop\n");
                fprintf(fout, "         vertex  %.16g %.16g %.16g\n", f->p1->position.x, f->p1->position.y,
                        f->p1->position.z);
                fprintf(fout, "         vertex  %.16g %.16g %.16g\n", f->p2->position.x, f->p2->position.y,
                        f->p2->position.z);
                fprintf(fout, "         vertex  %.16g %.16g %.16g\n", f->p3->position.x, f->p3->position.y,
                        f->p3->position.z);
                fprintf(fout, "     endloop\n");
                fprintf(fout, " endfacet\n");
            }
            fprintf(fout, "endsolid OBJECT\n");

            fclose(fout);
            return true;
        }
    };

}
