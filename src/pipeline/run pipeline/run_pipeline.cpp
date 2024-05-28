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


void end_2_end() {
    //Step 1: 求曲面与网格体的交点，计算相交边缘（可能存在多个）
    logger().info("Step 1: Compute Point");
    Triangle_Soup_Mesh meshCube;
    Triangle_Soup_Mesh meshCurve;
    Triangle_Soup_Mesh meshResult;
    meshCube.load_from_file("D:/xmy/model/cube.obj");
    meshCurve.load_from_file("D:/xmy/model/curve2.obj");

//    triangle_soup_surface_mesh_normal_fix(meshCube);
//    triangle_soup_surface_mesh_normal_fix(meshCurve);
    //这个方法可以修复网格的法向量，让法向量指向模型同一侧 法向量的朝向和三角形的三个点索引顺序有关，一般是满足右手法则
    //我教你怎么在paraview里面显示法向量

    Vertex *orig;
    Vertex *end;
    Vertex *tmp0, *tmp1, *tmp2;
    Edge *edge;
    Face *triCube;
    Face *triCurve;
    Face *addCube;
    vector<Face *> CubeArray;
    Face *f0;
    Face *f1;
    Face *f2;
    Face *f2_0;
    Face *f2_1;
    Edge *e0_0;
    Edge *e0_1;
    Edge *e1_0;
    Edge *e1_1;

    vector<segment> segArray;
    vector<segment> segArray2;

    CubeArray.clear();
    vector<Vector3> pts;

    auto meshCut = [](Triangle_Soup_Mesh &mesh, Face *&face_mesh, Vertex *&orig, Face *&f0, Face *&f1, Face *&f2) {
        Vertex *tmp0 = face_mesh->p1;
        Vertex *tmp1 = face_mesh->p2;
        Vertex *tmp2 = face_mesh->p3;

        Edge *e0 = Edge::allocate_from_pool(&mesh.edge_pool, orig, tmp0);
        Edge *e1 = Edge::allocate_from_pool(&mesh.edge_pool, orig, tmp1);
        Edge *e2 = Edge::allocate_from_pool(&mesh.edge_pool, orig, tmp2);

        f0 = Face::allocate_from_pool(&mesh.face_pool, orig, tmp0, tmp1);
        f1 = Face::allocate_from_pool(&mesh.face_pool, orig, tmp1, tmp2);
        f2 = Face::allocate_from_pool(&mesh.face_pool, orig, tmp2, tmp0);

        e0->connect_face_array->push_back(f0);
        e0->connect_face_array->push_back(f2);
        e1->connect_face_array->push_back(f0);
        e1->connect_face_array->push_back(f1);
        e2->connect_face_array->push_back(f1);
        e2->connect_face_array->push_back(f2);

        Edge *ed0 = Face::get_edge_from_two_vertex(face_mesh, tmp0, tmp1);
        Edge *ed1 = Face::get_edge_from_two_vertex(face_mesh, tmp1, tmp2);
        Edge *ed2 = Face::get_edge_from_two_vertex(face_mesh, tmp2, tmp0);

        f0->disjoin_edge[0] = ed0;
        f0->disjoin_edge[1] = e1;
        f0->disjoin_edge[2] = e0;
        f1->disjoin_edge[0] = ed1;
        f1->disjoin_edge[1] = e2;
        f1->disjoin_edge[2] = e1;
        f2->disjoin_edge[0] = ed2;
        f2->disjoin_edge[1] = e0;
        f2->disjoin_edge[2] = e2;


        if ((*(ed0->connect_face_array))[0] == face_mesh){
            (*(ed0->connect_face_array))[0] = f0;
        }
        else{
            (*(ed0->connect_face_array))[1] = f0;
        }
        if ((*(ed1->connect_face_array))[0] == face_mesh){
            (*(ed1->connect_face_array))[0] = f1;
        }
        else{
            (*(ed1->connect_face_array))[1] = f1;
        }
        if ((*(ed2->connect_face_array))[0] == face_mesh){
            (*(ed2->connect_face_array))[0] = f2;
        }
        else{
            (*(ed2->connect_face_array))[1] = f2;
        }

        f0->mark = true;
        f1->mark = true;
        f2->mark = true;

        mesh.face_pool.deallocate(face_mesh);
    };

    auto meshCut2 = [](Triangle_Soup_Mesh &mesh, Face *&face_mesh, Vertex *&orig, Face *&f2_0, Face *&f2_1) {
        Vertex *tmp0 = face_mesh->p1;
        Vertex *tmp1 = face_mesh->p2;
        Vertex *tmp2 = face_mesh->p3;

        Edge *e0 = Edge::allocate_from_pool(&mesh.edge_pool, orig, tmp0);
        Edge *e1 = Edge::allocate_from_pool(&mesh.edge_pool, orig, tmp1);
        Edge *e2 = Edge::allocate_from_pool(&mesh.edge_pool, orig, tmp2);

        f2_0 = Face::allocate_from_pool(&mesh.face_pool, orig, tmp0, tmp1);
        f2_1 = Face::allocate_from_pool(&mesh.face_pool, orig, tmp0, tmp2);

        f2_0->mark = true;
        f2_1->mark = true;

        Face *disjoinface = Face::get_disjoin_face(face_mesh, Face::get_edge_from_two_vertex(face_mesh, tmp1, tmp2));
        Vertex *nosharevtx = Face::get_disjoin_face_no_share_vtx(face_mesh, disjoinface);
//        Edge *noshareedge1 = Face::get_non_share_edge(face_mesh, disjoinface).first;
//        Edge *noshareedge2 = Face::get_non_share_edge(face_mesh, disjoinface).second;

        Edge *e3 = Edge::allocate_from_pool(&mesh.edge_pool, orig, nosharevtx);
        Face *f3_0 = Face::allocate_from_pool(&mesh.face_pool, orig, tmp1, nosharevtx);
        Face *f3_1 = Face::allocate_from_pool(&mesh.face_pool, orig, tmp2, nosharevtx);

        e0->connect_face_array->push_back(f2_0);
        e0->connect_face_array->push_back(f2_1);
        e1->connect_face_array->push_back(f2_0);
        e1->connect_face_array->push_back(f3_0);
        e2->connect_face_array->push_back(f2_1);
        e2->connect_face_array->push_back(f3_1);
        e3->connect_face_array->push_back(f3_0);
        e3->connect_face_array->push_back(f3_1);

        Edge *ed0 = Face::get_edge_from_two_vertex(disjoinface, tmp1, nosharevtx);
        Edge *ed1 = Face::get_edge_from_two_vertex(disjoinface, tmp2, nosharevtx);

        f2_0->disjoin_edge[0] = Face::get_edge_from_two_vertex(face_mesh, tmp0, tmp1);
        f2_0->disjoin_edge[1] = e1;
        f2_0->disjoin_edge[2] = e0;
        f2_1->disjoin_edge[0] = Face::get_edge_from_two_vertex(face_mesh, tmp0, tmp2);
        f2_1->disjoin_edge[1] = e2;
        f2_1->disjoin_edge[2] = e0;
        f3_0->disjoin_edge[0] = ed0;
        f3_0->disjoin_edge[1] = e3;
        f3_0->disjoin_edge[2] = e1;
        f3_1->disjoin_edge[0] = ed1;
        f3_1->disjoin_edge[1] = e3;
        f3_1->disjoin_edge[2] = e2;

        if ((*(ed0->connect_face_array))[0] == disjoinface){
            (*(ed0->connect_face_array))[0] = f3_0;
        }
        else{
            (*(ed0->connect_face_array))[1] = f3_0;
        }
        if ((*(ed1->connect_face_array))[0] == disjoinface){
            (*(ed1->connect_face_array))[0] = f3_1;
        }
        else{
            (*(ed1->connect_face_array))[1] = f3_1;
        }

        mesh.face_pool.deallocate(face_mesh);
        mesh.face_pool.deallocate(disjoinface);
    };


    auto meshCut3 = [](Triangle_Soup_Mesh &mesh, Face *&face_mesh, Face *&cut_mesh1, Face *&cut_mesh2, Vertex *&orig, Edge *&e1, Edge *&e2) {

        Vertex *tmp1 = face_mesh->p2;
        Vertex *tmp2 = face_mesh->p3;

        e1 = Edge::allocate_from_pool(&mesh.edge_pool, orig, tmp1);
        e2 = Edge::allocate_from_pool(&mesh.edge_pool, orig, tmp2);

        Face *disjoinface = Face::get_disjoin_face(face_mesh, Face::get_edge_from_two_vertex(face_mesh, tmp1, tmp2));
        Vertex *nosharevtx = Face::get_disjoin_face_no_share_vtx(face_mesh, disjoinface);

        Edge *e3 = Edge::allocate_from_pool(&mesh.edge_pool, orig, nosharevtx);
        Face *f3_0 = Face::allocate_from_pool(&mesh.face_pool, orig, tmp1, nosharevtx);
        Face *f3_1 = Face::allocate_from_pool(&mesh.face_pool, orig, tmp2, nosharevtx);

        e1->connect_face_array->push_back(cut_mesh1);
        e1->connect_face_array->push_back(f3_0);
        e2->connect_face_array->push_back(cut_mesh2);
        e2->connect_face_array->push_back(f3_1);
        e3->connect_face_array->push_back(f3_0);
        e3->connect_face_array->push_back(f3_1);

        Edge *ed0 = Face::get_edge_from_two_vertex(disjoinface, tmp1, nosharevtx);
        Edge *ed1 = Face::get_edge_from_two_vertex(disjoinface, tmp2, nosharevtx);

        f3_0->disjoin_edge[0] = ed0;
        f3_0->disjoin_edge[1] = e3;
        f3_0->disjoin_edge[2] = e1;
        f3_1->disjoin_edge[0] = ed1;
        f3_1->disjoin_edge[1] = e3;
        f3_1->disjoin_edge[2] = e2;

        if ((*(ed0->connect_face_array))[0] == disjoinface){
            (*(ed0->connect_face_array))[0] = f3_0;
        }
        else{
            (*(ed0->connect_face_array))[1] = f3_0;
        }
        if ((*(ed1->connect_face_array))[0] == disjoinface){
            (*(ed1->connect_face_array))[0] = f3_1;
        }
        else{
            (*(ed1->connect_face_array))[1] = f3_1;
        }

        mesh.face_pool.deallocate(disjoinface);
    };


    auto meshcut4 = [&meshCut3](Triangle_Soup_Mesh &mesh, Face *&face_mesh, Vertex *&orig, Vertex *&end, Vertex *&tmp0, Vertex *&tmp1, Vertex *&tmp2){
        Face *f0 =  Face::allocate_from_pool(&mesh.face_pool, orig, end, tmp0);
        Face *f1 =  Face::allocate_from_pool(&mesh.face_pool, orig, end, tmp1);
        Face *f2 =  Face::allocate_from_pool(&mesh.face_pool, end, tmp1, tmp2);

        Edge *e0_0;
        Edge *e0_1;
        Edge *e1_0;
        Edge *e1_1;

        meshCut3(mesh, face_mesh, f0, f1, orig, e0_0, e0_1);
        meshCut3(mesh, face_mesh, f0, f2, end, e1_0, e1_1);

        Edge *e0 = Edge::allocate_from_pool(&mesh.edge_pool, orig, end);
        Edge *e1 = Edge::allocate_from_pool(&mesh.edge_pool, end, tmp1);

        e0->connect_face_array->push_back(f0);
        e0->connect_face_array->push_back(f1);
        e1->connect_face_array->push_back(f1);
        e1->connect_face_array->push_back(f2);

        f0->disjoin_edge[0] = e0_0;
        f0->disjoin_edge[1] = e0;
        f0->disjoin_edge[2] = e1_0;
        f1->disjoin_edge[0] = e0_1;
        f1->disjoin_edge[1] = e1;
        f1->disjoin_edge[2] = e0;
        f2->disjoin_edge[0] = e1;
        f2->disjoin_edge[1] = Face::get_edge_from_two_vertex(face_mesh, tmp1, tmp2);
        f2->disjoin_edge[2] = e1_1;

        f0->mark = true;
        f1->mark = true;
        f2->mark = true;

        mesh.face_pool.deallocate(face_mesh);
    };

//    int meshCubeSize = meshCube.face_pool.size();
//    for (int i = 0; i < meshCubeSize; i++) {
//        triCube = (Face *) meshCube.face_pool[i];
//
//        Vector3 pt0_Cube = Vector3(triCube->p1->position.x, triCube->p1->position.y, triCube->p1->position.z);
//        Vector3 pt1_Cube = Vector3(triCube->p2->position.x, triCube->p2->position.y, triCube->p2->position.z);
//        Vector3 pt2_Cube = Vector3(triCube->p3->position.x, triCube->p3->position.y, triCube->p3->position.z);
//        Triangle tri1 = Triangle(pt0_Cube, pt1_Cube, pt2_Cube);
//        for (int j = 0; j < meshCurve.face_pool.size(); j++) {
//            triCurve = (Face *) meshCurve.face_pool[j];
//            Vector3 pt0_Curve = Vector3(triCurve->p1->position.x, triCurve->p1->position.y, triCurve->p1->position.z);
//            Vector3 pt1_Curve = Vector3(triCurve->p2->position.x, triCurve->p2->position.y, triCurve->p2->position.z);
//            Vector3 pt2_Curve = Vector3(triCurve->p3->position.x, triCurve->p3->position.y, triCurve->p3->position.z);
//            Triangle tri2 = Triangle(pt0_Curve, pt1_Curve, pt2_Curve);
//            //if (GetIntersectionPoints(tri1, tri2, pts) == INTERSECTION && pts.size() == 2)
//            if (ComputeLineWithTwoTriangle(tri1, tri2, pts)) {
////                CubeArray.push_back(triCube);
//                Vector3 p_result_1 = Vector3(pts[0].x, pts[0].y, pts[0].z);
//                Vector3 p_result_2 = Vector3(pts[1].x, pts[1].y, pts[1].z);
//                segment seg = segment(p_result_1, p_result_2);
//                segArray.push_back(seg);
////                orig = Vertex::allocate_from_pool(&meshCube.vertex_pool, p_result_1);
////                end = Vertex::allocate_from_pool(&meshCube.vertex_pool, p_result_2);
////                Edge::allocate_from_pool(&meshCube.edge_pool, orig, end);
//
//                pts.clear();
//            }
//        }
//    }
//

//    GEO::Mesh A;
//    GEO_Proxy::get_geogram_mesh(meshCube, A);
//
//    GEO::MeshSurfaceIntersection I(A);
//    I.intersect();
//    I.remove_internal_shells();
//    GEO::mesh_repair(A, GEO::MESH_REPAIR_DEFAULT2, 1e-5);
//    GEO_Proxy::get_tri_mesh(meshCube, A);

//    GEO::Mesh B;
//    GEO_Proxy::get_geogram_mesh(meshCurve, B);
//
//    GEO::MeshSurfaceIntersection J(B);
//    J.intersect();
//    J.remove_internal_shells();
//    GEO::mesh_repair(B, GEO::MESH_REPAIR_DEFAULT2, 1e-5);
//    GEO_Proxy::get_tri_mesh(meshCurve, B);

    logger().warn("Step 2: mesh cut");
    {
//        for (auto i = 0; i < mesh.face_pool.size(); i++) {
//            auto f = (Face *) mesh.face_pool[i];
//            double f_area = get_triangle_area({f->p1->position, f->p2->position, f->p3->position});
//
//        }
//
//        for (auto i = 0; i < meshCube.vertex_pool.size(); i++) {
//            auto v = (Vertex *) meshCube.vertex_pool[i];
//
//
//            if (i < 4) {
//                v->type = 1;
//            }
//        }
//
//        Vertex *v = Vertex::allocate_from_pool(&mesh.vertex_pool, {1, 2, 3});
//        Face *f = Face::allocate_from_pool(&mesh.face_pool, v, v, v);
    }

    ////xmy


//    meshCube.face_pool
//    meshCurve

    auto face_to_tri = [](base_type::Face *f, Triangle& tri) {
        Vector3 pt0_Cube = Vector3(f->p1->position.x, f->p1->position.y, f->p1->position.z);
        Vector3 pt1_Cube = Vector3(f->p2->position.x, f->p2->position.y, f->p2->position.z);
        Vector3 pt2_Cube = Vector3(f->p3->position.x, f->p3->position.y, f->p3->position.z);
        tri = Triangle(pt0_Cube, pt1_Cube, pt2_Cube);
    };

    auto tri_tri_cut = [&](base_type::Face *f1, base_type::Face *f2, base_type::Vector3 &p1,
                          base_type::Vector3 &p2) -> bool {
        Triangle tri1, tri2;
        face_to_tri(f1, tri1);
        face_to_tri(f2, tri2);

        vector<Vector3> pts;
        if (ComputeLineWithTwoTriangle(tri1, tri2, pts))
        {
            p1 = pts[0];
            p2 = pts[1];

            return true;
        }
        else
            return false;
    };

    auto clear_all_tri_mark = [](Triangle_Soup_Mesh &mesh) {
        for (int i = 0; i < mesh.face_pool.size(); i++) {
            base_type::Face *f_mesh = (base_type::Face *) mesh.face_pool[i];
            f_mesh->mark = false;
        }
    };

    int cubeSize;
    auto insert_one_tri = [&](Triangle_Soup_Mesh &mesh, base_type::Face *f) {
        clear_all_tri_mark(mesh);

        insert_start:
//        Triangle_Soup_Mesh tmpmesh(mesh);
        cubeSize = mesh.face_pool.size();
        for (int i = 0; i < cubeSize; i++) {
            base_type::Face *f_mesh = (base_type::Face *) mesh.face_pool[i];
            base_type::Vector3 p1;
            base_type::Vector3 p2;
            if(f_mesh->mark == true){
                continue;
            }
            else{
                f_mesh->mark = true;
            }

            if (tri_tri_cut(f_mesh, f, p1, p2)) {
                bool need_to_check_again = false;
                Triangle tri1;
                face_to_tri(f_mesh, tri1);

                // case 1 p1 and p2 inside f_mesh
                // split f_mesh , need_to_check_again = false
                if (InTriangle(tri1, p1) == 1 && InTriangle(tri1, p2) == 1) {
                    segment seg = segment(p1, p2);
                    segArray2.push_back(seg);

                    orig = Vertex::allocate_from_pool(&mesh.vertex_pool, p1);
                    end = Vertex::allocate_from_pool(&mesh.vertex_pool, p2);
                    meshCut(mesh, f_mesh, orig, f0, f1, f2);

                    Face *tmpf0, *tmpf1, *tmpf2;
                    Triangle tmpTri = Triangle(p1, tri1.m_pt[0], tri1.m_pt[1]);
                    if (InTriangle(tmpTri, p2) == 1) {
                        meshCut(mesh, f0, end, tmpf0, tmpf1, tmpf2);
                        goto insert_start;
                    }
                    tmpTri = Triangle(p1, tri1.m_pt[1], tri1.m_pt[2]);
                    if (InTriangle(tmpTri, p2) == 1) {
                        meshCut(mesh, f1, end, tmpf0, tmpf1, tmpf2);
                        goto insert_start;
                    }
                    tmpTri = Triangle(p1, tri1.m_pt[2], tri1.m_pt[0]);
                    if (InTriangle(tmpTri, p2) == 1) {
                        meshCut(mesh, f2, end, tmpf0, tmpf1, tmpf2);
                        goto insert_start;
                    }
                }

                // case 2 one of p1 and p2 on f_mesh's edge and the other inside f_mesh
                // split f_mesh and disjoin tri , need_to_check_again = true

                {
                    Vector3 p_result_1, p_result_2;
                    bool flag = false;
                    if (InTriangle(tri1, p1) == 1 && InTriangle(tri1, p2) == 0) {
                        p_result_1 = p1;
                        p_result_2 = p2;
                        flag = true;
                    }
                    if (InTriangle(tri1, p1) == 0 && InTriangle(tri1, p2) == 1) {
                        p_result_1 = p2;
                        p_result_2 = p1;
                        flag = true;
                    }
                    if (flag){
                        segment seg = segment(p_result_1, p_result_2);
                        segArray2.push_back(seg);

                        orig = Vertex::allocate_from_pool(&mesh.vertex_pool, p_result_1);
                        end = Vertex::allocate_from_pool(&mesh.vertex_pool, p_result_2);
                        meshCut(mesh, f_mesh, orig, f0, f1, f2);

                        Triangle tmpTri = Triangle(p_result_1, tri1.m_pt[0], tri1.m_pt[1]);
                        if (InTriangle(tmpTri, p_result_2) == 0) {
                            meshCut2(mesh, f0, end, f2_0, f2_1);
                            goto insert_start;
                        }
                        tmpTri = Triangle(p_result_1, tri1.m_pt[1], tri1.m_pt[2]);
                        if (InTriangle(tmpTri, p_result_2) == 0) {
                            meshCut2(mesh, f1, end, f2_0, f2_1);
                            goto insert_start;
                        }
                        tmpTri = Triangle(p_result_1, tri1.m_pt[2], tri1.m_pt[0]);
                        if (InTriangle(tmpTri, p_result_2) == 0) {
                            meshCut2(mesh, f2, end, f2_0, f2_1);
                            goto insert_start;
                        }
                    }
                }

                //case 3 p1 and p2 on the edge of f_mesh
                // split f_mesh  , need_to_check_again = true
                {
                    Vector3 p_result_1 = p1;
                    Vector3 p_result_2 = p2;
                    if (InTriangle(tri1, p_result_1) == 0 && InTriangle(tri1, p_result_2) == 0) {
                        segment seg = segment(p_result_1, p_result_2);
                        segArray2.push_back(seg);

                        orig = Vertex::allocate_from_pool(&meshCube.vertex_pool, p_result_1);
                        end = Vertex::allocate_from_pool(&meshCube.vertex_pool, p_result_2);

                        tmp0 = f_mesh->p1;
                        tmp1 = f_mesh->p2;
                        tmp2 = f_mesh->p3;

                        // f_mesh->p1->position为顶点
                        if ((ParallelJudgment(p_result_1 - f_mesh->p1->position, f_mesh->p2->position - f_mesh->p1->position)  &&
                             ParallelJudgment(p_result_2 - f_mesh->p1->position, f_mesh->p3->position - f_mesh->p1->position)) ||
                            (ParallelJudgment(p_result_1 - f_mesh->p1->position, f_mesh->p3->position - f_mesh->p1->position)  &&
                             ParallelJudgment(p_result_2 - f_mesh->p1->position, f_mesh->p2->position - f_mesh->p1->position)) ) {

                            meshcut4(mesh, f_mesh, orig, end, tmp0, tmp1, tmp2);
                            goto insert_start;
                        }

                        // f_mesh->p2->position为顶点
                        if ((ParallelJudgment(p_result_1 - f_mesh->p2->position, f_mesh->p3->position - f_mesh->p2->position)  &&
                             ParallelJudgment(p_result_2 - f_mesh->p2->position, f_mesh->p1->position - f_mesh->p2->position)) ||
                            (ParallelJudgment(p_result_1 - f_mesh->p2->position, f_mesh->p1->position - f_mesh->p2->position)  &&
                             ParallelJudgment(p_result_2 - f_mesh->p2->position, f_mesh->p3->position - f_mesh->p2->position)) ) {

                            Vertex *tmp = tmp0;
                            tmp0 = tmp2;
                            tmp2 = tmp1;
                            tmp1 = tmp;
                            meshcut4(mesh, f_mesh, orig, end, tmp0, tmp1, tmp2);
                            goto insert_start;
                        }

                        // f_mesh->p3->position为顶点
                        if ((ParallelJudgment(p_result_1 - f_mesh->p3->position, f_mesh->p1->position - f_mesh->p3->position)  &&
                             ParallelJudgment(p_result_2 - f_mesh->p3->position, f_mesh->p2->position - f_mesh->p3->position)) ||
                            (ParallelJudgment(p_result_1 - f_mesh->p3->position, f_mesh->p2->position - f_mesh->p3->position)  &&
                             ParallelJudgment(p_result_2 - f_mesh->p3->position, f_mesh->p1->position - f_mesh->p3->position)) ) {

                            Vertex *tmp = tmp0;
                            tmp0 = tmp1;
                            tmp1 = tmp2;
                            tmp2 = tmp;
                            meshcut4(mesh, f_mesh, orig, end, tmp0, tmp1, tmp2);
                            goto insert_start;
                        }

//                        // tri1.m_pt[1]为顶点
//                        if (ParallelJudgment(p_result_1 - tri1.m_pt[1], tri1.m_pt[0] - tri1.m_pt[1]) &&
//                            ParallelJudgment(p_result_2 - tri1.m_pt[1], tri1.m_pt[2] - tri1.m_pt[1])) {
//
//                            f0 = Face::allocate_from_pool(&mesh.face_pool, orig, end, tmp0);
//                            f1 = Face::allocate_from_pool(&mesh.face_pool, orig, end, tmp1);
//                            f2 = Face::allocate_from_pool(&mesh.face_pool, end, tmp0, tmp2);
//                            f0->mark = true;
//                            f1->mark = true;
//                            f2->mark = true;
//                            goto insert_start;
//                        }
//
//                        if (ParallelJudgment(p_result_1 - tri1.m_pt[1], tri1.m_pt[2] - tri1.m_pt[1]) &&
//                            ParallelJudgment(p_result_2 - tri1.m_pt[1], tri1.m_pt[0] - tri1.m_pt[1])) {
//
//                            f0 = Face::allocate_from_pool(&mesh.face_pool, orig, end, tmp0);
//                            f1 = Face::allocate_from_pool(&mesh.face_pool, orig, end, tmp1);
//                            f2 = Face::allocate_from_pool(&mesh.face_pool, orig, tmp0, tmp2);
//                            f0->mark = true;
//                            f1->mark = true;
//                            f2->mark = true;
//                            goto insert_start;
                        }
                    }

                //case 4 one of p1 and p2 on f_mesh's vertex and the other inside f_mesh
                {
                    Vector3 p_result_1;
                    Vector3 p_result_2;
                    bool flag = false;
                    if (InTriangle(tri1, p1) == 1 && InTriangle(tri1, p2) == 2) {
                        p_result_1 = p1;
                        p_result_2 = p2;
                        flag = true;
                    }
                    if (InTriangle(tri1, p1) == 2 && InTriangle(tri1, p2) == 1) {
                        p_result_1 = p2;
                        p_result_2 = p1;
                        flag = true;
                    }
                    if (flag){
                        orig = Vertex::allocate_from_pool(&mesh.vertex_pool, p_result_1);
                        segment seg = segment(p_result_1, p_result_2);
                        segArray2.push_back(seg);
                        meshCut(mesh, f_mesh, orig, f0, f1, f2);
                        goto insert_start;
                    }
                }

                //case 5 one of p1 and p2 on f_mesh's vertex and the other on the edge of f_mesh
                {
                    Vector3 p_result_1;
                    Vector3 p_result_2;
                    bool flag = false;
                    if (InTriangle(tri1, p1) == 0 && InTriangle(tri1, p2) == 2) {
                        p_result_1 = p1;
                        p_result_2 = p2;
                        flag = true;
                    }
                    if (InTriangle(tri1, p1) == 2 && InTriangle(tri1, p2) == 0) {
                        p_result_1 = p2;
                        p_result_2 = p1;
                        flag = true;
                    }
                    if (flag){
                        orig = Vertex::allocate_from_pool(&mesh.vertex_pool, p_result_1);
                        segment seg = segment(p_result_1, p_result_2);
                        segArray2.push_back(seg);
                        // p2为顶点
                        if (f_mesh->p2->position.distance(p_result_2) < 1e-6){
                            if (ParallelJudgment(p_result_1 - p_result_2, f_mesh->p3->position - f_mesh->p2->position)){
                                meshCut2(mesh, f_mesh, orig, f2_0, f2_1);
                                goto insert_start;
                            }
                            if (ParallelJudgment(p_result_1 - p_result_2, f_mesh->p1->position - f_mesh->p2->position)){
                                Vertex *tmp = f_mesh->p1;
                                f_mesh->p1 = f_mesh->p3;
                                f_mesh->p3 = tmp;
                                meshCut2(mesh, f_mesh, orig, f2_0, f2_1);
                                goto insert_start;
                            }
                            if (ParallelJudgment(p_result_1 - p_result_2, f_mesh->p1->position - f_mesh->p3->position)){
                                Vertex *tmp = f_mesh->p1;
                                f_mesh->p1 = f_mesh->p3;
                                f_mesh->p3 = f_mesh->p2;
                                f_mesh->p2 = tmp;
                                meshCut2(mesh, f_mesh, orig, f2_0, f2_1);
                                goto insert_start;
                            }
                        }
                        // p1为顶点
                        if (f_mesh->p1->position.distance(p_result_2) < 1e-6){
                            if (ParallelJudgment(p_result_1 - p_result_2, f_mesh->p2->position - f_mesh->p1->position)){
                                Vertex *tmp = f_mesh->p2;
                                f_mesh->p2 = f_mesh->p3;
                                f_mesh->p3 = f_mesh->p1;
                                f_mesh->p1 = tmp;
                                meshCut2(mesh, f_mesh, orig, f2_0, f2_1);
                                goto insert_start;
                            }
                            if (ParallelJudgment(p_result_1 - p_result_2, f_mesh->p1->position - f_mesh->p3->position)){
                                Vertex *tmp = f_mesh->p2;
                                f_mesh->p2 = f_mesh->p1;
                                f_mesh->p1 = f_mesh->p3;
                                f_mesh->p3 = tmp;
                                meshCut2(mesh, f_mesh, orig, f2_0, f2_1);
                                goto insert_start;
                            }
                            if (ParallelJudgment(p_result_1 - p_result_2, f_mesh->p2->position - f_mesh->p3->position)){
                                meshCut2(mesh, f_mesh, orig, f2_0, f2_1);
                                goto insert_start;
                            }
                        }
                        // p3为顶点
                        if (f_mesh->p3->position.distance(p_result_2) < 1e-6){
                            if (ParallelJudgment(p_result_1 - p_result_2, f_mesh->p1->position - f_mesh->p2->position)){
                                Vertex *tmp = f_mesh->p2;
                                f_mesh->p2 = f_mesh->p3;
                                f_mesh->p3 = f_mesh->p1;
                                f_mesh->p1 = tmp;
                                meshCut2(mesh, f_mesh, orig, f2_0, f2_1);
                                goto insert_start;
                            }
                            if (ParallelJudgment(p_result_1 - p_result_2, f_mesh->p1->position - f_mesh->p3->position)){
                                Vertex *tmp = f_mesh->p2;
                                f_mesh->p2 = f_mesh->p1;
                                f_mesh->p1 = f_mesh->p3;
                                f_mesh->p3 = tmp;
                                meshCut2(mesh, f_mesh, orig, f2_0, f2_1);
                                goto insert_start;
                            }
                            if (ParallelJudgment(p_result_1 - p_result_2, f_mesh->p2->position - f_mesh->p3->position)){
                                meshCut2(mesh, f_mesh, orig, f2_0, f2_1);
                                goto insert_start;
                            }
                        }
//                        tmp0 = Vertex::allocate_from_pool(&mesh.vertex_pool, tri1.m_pt[0]);
//                        tmp1 = Vertex::allocate_from_pool(&mesh.vertex_pool, tri1.m_pt[1]);
//                        tmp2 = Vertex::allocate_from_pool(&mesh.vertex_pool, tri1.m_pt[2]);
//                        //在0-1侧
//                        if (ParallelJudgment(p_result_1 - p_result_2, tri1.m_pt[1] - tri1.m_pt[0]) && p_result_2.distance(tri1.m_pt[0]) < 1e-6){
//                            f0 = Face::allocate_from_pool(&mesh.face_pool, orig, end, tmp2);
//                            f1 = Face::allocate_from_pool(&mesh.face_pool, orig, tmp1, tmp2);
//                            f0->mark = true;
//                            f1->mark = true;
//                            mesh.face_pool.deallocate(f_mesh);
//                            goto insert_start;
//                        }
//
//                        if (ParallelJudgment(p_result_1 - p_result_2, tri1.m_pt[1] - tri1.m_pt[0]) && p_result_2.distance(tri1.m_pt[1]) < 1e-6){
//                            f0 = Face::allocate_from_pool(&mesh.face_pool, orig, end, tmp2);
//                            f1 = Face::allocate_from_pool(&mesh.face_pool, orig, tmp0, tmp2);
//                            f0->mark = true;
//                            f1->mark = true;
//                            mesh.face_pool.deallocate(f_mesh);
//                            goto insert_start;
//                        }
//                        //在1-2侧
//                        if (ParallelJudgment(p_result_1 - p_result_2, tri1.m_pt[2] - tri1.m_pt[1]) && p_result_2.distance(tri1.m_pt[1]) < 1e-6){
//                            f0 = Face::allocate_from_pool(&mesh.face_pool, orig, end, tmp0);
//                            f1 = Face::allocate_from_pool(&mesh.face_pool, orig, tmp0, tmp2);
//                            f0->mark = true;
//                            f1->mark = true;
//                            mesh.face_pool.deallocate(f_mesh);
//                            goto insert_start;
//                        }
//
//                        if (ParallelJudgment(p_result_1 - p_result_2, tri1.m_pt[2] - tri1.m_pt[1]) && p_result_2.distance(tri1.m_pt[2]) < 1e-6){
//                            f0 = Face::allocate_from_pool(&mesh.face_pool, orig, end, tmp0);
//                            f1 = Face::allocate_from_pool(&mesh.face_pool, orig, tmp0, tmp1);
//                            f0->mark = true;
//                            f1->mark = true;
//                            mesh.face_pool.deallocate(f_mesh);
//                            goto insert_start;
//                        }
//                        //在0-2侧
//                        if (ParallelJudgment(p_result_1 - p_result_2, tri1.m_pt[0] - tri1.m_pt[2]) && p_result_2.distance(tri1.m_pt[0]) < 1e-6){
//                            f0 = Face::allocate_from_pool(&mesh.face_pool, orig, end, tmp1);
//                            f1 = Face::allocate_from_pool(&mesh.face_pool, orig, tmp1, tmp2);
//                            f0->mark = true;
//                            f1->mark = true;
//                            mesh.face_pool.deallocate(f_mesh);
//                            goto insert_start;
//                        }
//
//                        if (ParallelJudgment(p_result_1 - p_result_2, tri1.m_pt[0] - tri1.m_pt[2]) && p_result_2.distance(tri1.m_pt[2]) < 1e-6){
//                            f0 = Face::allocate_from_pool(&mesh.face_pool, orig, end, tmp1);
//                            f1 = Face::allocate_from_pool(&mesh.face_pool, orig, tmp0, tmp1);
//                            f0->mark = true;
//                            f1->mark = true;
//                            mesh.face_pool.deallocate(f_mesh);
//                            goto insert_start;
//                        }
//                        // 顶点-对边
//                        if (p_result_2.distance(tri1.m_pt[0]) < 1e-6){
//                            f0 = Face::allocate_from_pool(&mesh.face_pool, orig, end, tmp1);
//                            f1 = Face::allocate_from_pool(&mesh.face_pool, orig, end, tmp2);
//                            f0->mark = true;
//                            f1->mark = true;
//                            mesh.face_pool.deallocate(f_mesh);
//                            goto insert_start;
//                        }
//
//                        if (p_result_2.distance(tri1.m_pt[1]) < 1e-6){
//                            f0 = Face::allocate_from_pool(&mesh.face_pool, orig, end, tmp0);
//                            f1 = Face::allocate_from_pool(&mesh.face_pool, orig, end, tmp2);
//                            f0->mark = true;
//                            f1->mark = true;
//                            mesh.face_pool.deallocate(f_mesh);
//                            goto insert_start;
//                        }
//
//                        if (p_result_2.distance(tri1.m_pt[2]) < 1e-6){
//                            f0 = Face::allocate_from_pool(&mesh.face_pool, orig, end, tmp0);
//                            f1 = Face::allocate_from_pool(&mesh.face_pool, orig, end, tmp1);
//                            f0->mark = true;
//                            f1->mark = true;
//                            mesh.face_pool.deallocate(f_mesh);
//                            goto insert_start;
//                        }
                    }
                }

                //case 6 both p1 and p2 on the vertex of f_mesh
                {
                    if (InTriangle(tri1, p1) == 2 && InTriangle(tri1, p2) == 2){
                        segment seg = segment(p1, p2);
                        segArray2.push_back(seg);
                        //goto insert_start;
                        continue;
                    }
                }

                logger().warn("Result Error!");
                //end

//                if (need_to_check_again) {
//                    goto insert_start;
//                } else {
//                    break;
//                }
            }

        }
    };

    //step 1: use meshCurve to subdivide meshCube
    int curveSize = meshCurve.face_pool.size();
//    for (int i = 0; i < curveSize; i++) {
    for (int i = 0; i < 1; i++) {
        auto f = (base_type::Face *) meshCurve.face_pool[i];
        insert_one_tri(meshCube, f);
    }

    //step 2: 拆分meshCube by spatial edge and vertex


    logger().warn("Step 3: mesh save");
    meshCube.save("D:/xmy/model/", "output");
//    meshCurve.save("D:/xmy/model/", "output2");

    int a = 100;
    double b = 200.5;
    std::string log_content = std::format("i have vertex:{}, tet:{}", a, b);
    logger().info(log_content);
    logger().info(std::format("i have vertex:{}, tet:{}", a, b));

    logger().info("end");

}



