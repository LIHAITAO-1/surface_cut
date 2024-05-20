//
// Created by xmyci on 27/02/2024.
//
#include "run_pipeline.h"
#include "TriIntersectionTri.h"
#include "geogram/mesh/mesh_AABB.h"
#include <vector>
#include <geogram/mesh/mesh_remesh.h>

using namespace std;
using namespace base_type;
using namespace Geometrical_Predicates;


void end_2_end() {
    //Step 1: 求曲面与网格体的交点，计算相交边缘（可能存在多个）
    logger().info("Step 1: Compute Point");
    Triangle_Soup_Mesh meshCube;
    Triangle_Soup_Mesh meshCurve;
    Triangle_Soup_Mesh meshResult;
    meshCube.load_from_file("D:/xmy/model/cube.obj");
    meshCurve.load_from_file("D:/xmy/model/curve.obj");

    triangle_soup_surface_mesh_normal_fix(meshCube);
//    triangle_soup_surface_mesh_normal_fix(meshCurve);
    //这个方法可以修复网格的法向量，让法向量指向模型同一侧 法向量的朝向和三角形的三个点索引顺序有关，一般是满足右手法则
    //我教你怎么在paraview里面显示法向量

    Line3d line;
    Vertex *orig;
    Vertex *end;
    Vertex *tmp;
    Edge *edge;
    Face *triCube;
    Face *triCurve;

    vector<Vector3> pts;

//    for (int i = 1; i < 2; i++)
    for (int i = 0; i < meshCube.face_pool.size(); i++)
    {
        triCube = (Face *)meshCube.face_pool[i];
        Vector3 pt0_Cube = Vector3(triCube->p1->position.x, triCube->p1->position.y, triCube->p1->position.z);
        Vector3 pt1_Cube = Vector3(triCube->p2->position.x, triCube->p2->position.y, triCube->p2->position.z);
        Vector3 pt2_Cube = Vector3(triCube->p3->position.x, triCube->p3->position.y, triCube->p3->position.z);
        Triangle tri1 = Triangle(pt0_Cube, pt1_Cube, pt2_Cube);
        for (int j = 0; j < meshCurve.face_pool.size(); j++)
        {
            triCurve = (Face *)meshCurve.face_pool[j];
            Vector3 pt0_Curve = Vector3(triCurve->p1->position.x, triCurve->p1->position.y, triCurve->p1->position.z);
            Vector3 pt1_Curve = Vector3(triCurve->p2->position.x, triCurve->p2->position.y, triCurve->p2->position.z);
            Vector3 pt2_Curve = Vector3(triCurve->p3->position.x, triCurve->p3->position.y, triCurve->p3->position.z);
            Triangle tri2 = Triangle(pt0_Curve, pt1_Curve, pt2_Curve);
            if (GetIntersectionPoints(tri1, tri2, pts) == INTERSECTION && pts.size() == 2)
            {
                //for (int num = 0; num < pts.size(); num++)
                if ((InTriangle(tri1, pts[0]) != -1 && InTriangle(tri1, pts[1]) != -1) && (InTriangle(tri2, pts[0]) != -1 && InTriangle(tri2, pts[1]) != -1))
//                if (InTriangle(tri1, pts[0]) == 0 && InTriangle(tri1, pts[1]) == 0)
                {
                    Vector3 p_result_1 = Vector3(pts[0].x, pts[0].y, pts[0].z);
                    Vector3 p_result_2 = Vector3(pts[1].x, pts[1].y, pts[1].z);

                    orig = Vertex::allocate_from_pool(&meshCube.vertex_pool, p_result_1);
                    end = Vertex::allocate_from_pool(&meshCube.vertex_pool, p_result_2);
                    tmp = Vertex::allocate_from_pool(&meshCube.vertex_pool, {100, 100, 100});
//                    //logger().info(std::format("{} have vertex:{},{},{}", num,orig->position.x, orig->position.y, orig->position.z));
                    Face::allocate_from_pool(&meshCube.face_pool, orig, end, tmp);
                }

                pts.clear();
            }
        }
    }



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

    logger().warn("Step 3: mesh save");
    meshCube.save("D:/xmy/model/", "output");


    int a = 100;
    double b = 200.5;
    std::string log_content = std::format("i have vertex:{}, tet:{}", a, b);
    logger().info(log_content);
    logger().info(std::format("i have vertex:{}, tet:{}", a, b));

    logger().info("end");
}



//const float EPSILON = std::numeric_limits<float>::epsilon();
//bool ComputePointWithLineAndTriangle(const Line3d& line, const Triangle3d& tri, Vector3& point)
//{
//    Vector3 e0 = tri.p2 - tri.p1;
//    Vector3 e1 = tri.p3 - tri.p1;
//
//    Vector3 dir = line.p2 - line.p1;
//    Vector3 dir_norm = dir.normalise();
//
//    Vector3 h = dir_norm.cross(e1);
//    const float a = e0.dot(h);
//
//    if (a > -EPSILON && a < EPSILON) {
//        return false;
//    }
//
//    Vector3 s = line.p1 - tri.p1;
//    const float f = 1.0f / a;
//    const float u = f * s.dot(h);
//
//    if (u < 0.0f || u > 1.0f) {
//        return false;
//    }
//
//    Vector3 q = s.cross(e0);
//    const float v = f * dir_norm.dot(q);
//
//    if (v < 0.0f || u + v > 1.0f) {
//        return false;
//    }
//
//
//    const float t = f * e1.dot(q);
//    if (t > EPSILON && t < sqrtf(dir.dot(dir))) // segment intersection
//    {
//        point = line.p1 + dir_norm * t;
//        return true;
//    }
//    return false;
//}
//
//bool ComputeLineWithTwoTriangle(const Triangle3d& tri1, const Triangle3d& tri2, Line3d& lineResult)
//{
//    Vector3 point;
//    Line3d line;
//    vector<Vector3> pointArray;
//    pointArray.clear();
//
//    line = Line3d(tri1.p2, tri1.p1);
//    if (ComputePointWithLineAndTriangle(line, tri2, point)) pointArray.push_back(point);
//
//    line = Line3d(tri1.p3, tri1.p2);
//    if (ComputePointWithLineAndTriangle(line, tri2, point)) pointArray.push_back(point);
//
//    line = Line3d(tri1.p1, tri1.p3);
//    if (ComputePointWithLineAndTriangle(line, tri2, point)) pointArray.push_back(point);
//
//    line = Line3d(tri2.p1, tri2.p3);
//    if (ComputePointWithLineAndTriangle(line, tri1, point)) pointArray.push_back(point);
//
//    line = Line3d(tri2.p1, tri2.p3);
//    if (ComputePointWithLineAndTriangle(line, tri1, point)) pointArray.push_back(point);
//
//    line = Line3d(tri2.p1, tri2.p3);
//    if (ComputePointWithLineAndTriangle(line, tri1, point)) pointArray.push_back(point);
//
//    if (pointArray.size() == 2)
//    {
//        if (abs(pointArray[0].distance(pointArray[1])) < EPSILON)
//            return false;
//        else
//        {
//            lineResult.p1 = pointArray[0];
//            lineResult.p2 = pointArray[1];
//            return true;
//        }
//    }
//    return false;
//}