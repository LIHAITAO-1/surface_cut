//
// Created by 22624 on 2024/5/17.
//


#include "TriIntersectionTri.h"

#define EPSION 1e-7

bool IsZero(double value, double epsion = EPSION)
{
    return std::abs(value) < epsion;
}

bool IsEqual(double v1, double v2, double epsion = EPSION)
{
    return IsZero(v1-v2, epsion);
}

bool IsPositive(double value, double epsion = EPSION)
{
    return value - epsion > 0;
}

bool IsNegative(double value, double epsion = EPSION)
{
    return value + epsion < 0;
}

int GetSignType(double value)
{
    if (IsZero(value)) return 0;
    if (IsPositive(value)) return 1;
    return -1;
}

//int InTriangle(const Triangle& tri, const Vector3& pt)
//{
//    Vector3 AB = tri.m_pt[1] - tri.m_pt[0];
//    Vector3 BC = tri.m_pt[2] - tri.m_pt[1];
//    Vector3 CA = tri.m_pt[0] - tri.m_pt[2];
//
//    Vector3 AP = pt - tri.m_pt[0];
//    Vector3 BP = pt - tri.m_pt[1];
//    Vector3 CP = pt - tri.m_pt[2];
//
//    Vector3 v0 = AB.cross(AP);
//    Vector3 v1 = BC.cross(BP);
//    Vector3 v2 = CA.cross(CP);
//}

//int InTriangle(const Triangle& tri, const Vector3& pt)
//{
//    if (GetPositionType(tri, pt) == OUT)
//        return -1;
//    if (GetPositionType(tri, pt) == ON)
//        return 0;
//    else
//        return 1;
//}

//PositionType GetPositionType(const Triangle& tri, const Vector3& pt)
//{
//    Vector3 ap = tri.m_pt[0] - pt;
//    Vector3 apUnit = ap.normalise();
//    Vector3 normal = (tri.m_pt[1]-tri.m_pt[0]).cross(tri.m_pt[2]-tri.m_pt[0]);
//    Vector3 normalUnit = normal.normalise();
//    double inPlane = std::fabs(normalUnit.dot(apUnit));
//    if (inPlane > EPSION) {
//        return OUT; // 点p不在三角形所在的平面上
//    }
//    Eigen::Matrix3d oriented;
//    oriented << tri.m_pt[0].x - pt.x, tri.m_pt[0].y - pt.y, tri.m_pt[0].z - pt.z,
//            tri.m_pt[1].x - pt.x, tri.m_pt[1].y - pt.y, tri.m_pt[1].z - pt.z,
//            tri.m_pt[2].x- pt.x, tri.m_pt[2].y - pt.y, tri.m_pt[2].z - pt.z;
//
////    oriented << 1, 1, 1,
////                0, 1, 1,
////                0, 0, 1;
//    double value = oriented.determinant();
//    if (IsNegative(value))
//    {
//        return OUT;
//    }
//    else if (IsPositive(value))
//    {
//        return IN;
//    }
//
//    return ON;
//}
//
//Vector3 CalInterPoint(const Vector3& planeNormal, const Vector3& ptOnPlane, const Vector3& lineDir, const Vector3& ptOnLine)
//{
//    double t = planeNormal.dot(ptOnPlane - ptOnLine) / planeNormal.dot(lineDir);
//    return ptOnLine + lineDir * t;
//}

bool CheckPtOnTriangle(const Triangle& tri, const Vector3& pt)
{
    Vector3 v0 = tri.m_pt[1] - tri.m_pt[0], v1 = tri.m_pt[2] - tri.m_pt[0], v2 = pt - tri.m_pt[0];
    double d00 = v0.dot(v0);
    double d01 = v0.dot(v1);
    double d11 = v1.dot(v1);
    double d20 = v2.dot(v1);
    double d21 = v2.dot(v1);
    double denom = d00 * d11 - d01 * d01;
    double v = (d11 * d20 - d01 * d21) / denom;
    double w = (d00 * d21 - d01 * d20) / denom;
    if (v >= 0 && v <= 1 && w >= 0 && w <= 1)
    {
        return true;
    }
    return false;
}

//IntersectionType GetIntersectionPoints(const Triangle& triPlane, const Triangle& triPoints, std::vector<Vector3>& pts)
//{
//    std::vector<Vector3> ptResult;
//
//    PositionType relateToTriPlane[3] = {
//            GetPositionType(triPlane, triPoints.m_pt[0]),
//            GetPositionType(triPlane, triPoints.m_pt[1]),
//            GetPositionType(triPlane, triPoints.m_pt[2])
//    };
//
//    // 平面和直线相交计算
//    // 直线 P = V0 + dir*t
//    // 平面 Normal \cdot (P - POn) = 0
//    // =>
//    // t = N \cdot (POn - V0) / N \cdot dir
//    if (relateToTriPlane[0] == relateToTriPlane[1] && relateToTriPlane[1] == relateToTriPlane[2])
//    {
//        if (relateToTriPlane[0] == ON)
//        {
//            return COPLANE;
//        }
//        else
//        {
//            return DISJOINT;
//        }
//    }
//    else if (relateToTriPlane[0] == relateToTriPlane[1])
//    {
////        if (relateToTriPlane[0] == OUT)
////        {
////            std::cout << "Test" << std::endl;
////        }
////        else
//        {
//            Vector3 inter1 = CalInterPoint(triPlane.m_vecNormal, triPlane.m_pt[0], triPoints.m_pt[2] - triPoints.m_pt[0], triPoints.m_pt[2]);
//            Vector3 inter2 = CalInterPoint(triPlane.m_vecNormal, triPlane.m_pt[0], triPoints.m_pt[2] - triPoints.m_pt[1], triPoints.m_pt[2]);
//
//            if (CheckPtOnTriangle(triPlane, inter1)) ptResult.push_back(inter1);
//            if (CheckPtOnTriangle(triPlane, inter2)) ptResult.push_back(inter2);
//        }
//    }
//    else if (relateToTriPlane[0] == relateToTriPlane[2])
//    {
////        if (relateToTriPlane[0] == OUT)
////        {
////            std::cout << "Test" << std::endl;
////        }
////        else
//        {
//            Vector3 inter1 = CalInterPoint(triPlane.m_vecNormal, triPlane.m_pt[0], triPoints.m_pt[1] - triPoints.m_pt[0], triPoints.m_pt[1]);
//            Vector3 inter2 = CalInterPoint(triPlane.m_vecNormal, triPlane.m_pt[0], triPoints.m_pt[1] - triPoints.m_pt[2], triPoints.m_pt[1]);
//
//            if (CheckPtOnTriangle(triPlane, inter1)) ptResult.push_back(inter1);
//            if (CheckPtOnTriangle(triPlane, inter2)) ptResult.push_back(inter2);
//        }
//    }
//    else if (relateToTriPlane[1] == relateToTriPlane[2])
//    {
////        if (relateToTriPlane[1] == OUT)
////        {
////            std::cout << "Test" << std::endl;
////        }
////        else
//        {
//            Vector3 inter1 = CalInterPoint(triPlane.m_vecNormal, triPlane.m_pt[0], triPoints.m_pt[0] - triPoints.m_pt[1], triPoints.m_pt[0]);
//            Vector3 inter2 = CalInterPoint(triPlane.m_vecNormal, triPlane.m_pt[0], triPoints.m_pt[0] - triPoints.m_pt[2], triPoints.m_pt[0]);
//
//            if (CheckPtOnTriangle(triPlane, inter1)) ptResult.push_back(inter1);
//            if (CheckPtOnTriangle(triPlane, inter2)) ptResult.push_back(inter2);
//        }
//    }
//    else // 有一个点位于三角平面上，另外两个点分别位于两边
//    {
//        if (relateToTriPlane[0] == ON && CheckPtOnTriangle(triPlane, triPoints.m_pt[0])) ptResult.push_back(triPoints.m_pt[0]);
//        else if (relateToTriPlane[1] == ON && CheckPtOnTriangle(triPlane, triPoints.m_pt[1])) ptResult.push_back(triPoints.m_pt[0]);
//        else if (relateToTriPlane[2] == ON && CheckPtOnTriangle(triPlane, triPoints.m_pt[2])) ptResult.push_back(triPoints.m_pt[0]);
//    }
//
//    if (ptResult.empty())
//    {
//        return DISJOINT;
//    }
//
//    std::move(begin(ptResult), end(ptResult), back_inserter(pts));
//
//    return INTERSECTION;
//}

//void TriIntersectTestCase()
//{
//    {
//        Triangle tr1(Vector3(0, 0, 0), Vector3(1, 0, 1), Vector3(0, 1, 1));
//        Triangle tr2(Vector3(1, 1, 0), Vector3(1, 1, 1), Vector3(0, 0, 1));
//
//        std::vector<Vector3> pts;
//        auto type = GetIntersectionPoints(tr1, tr2, pts);
//
//        assert(type == INTERSECTION);
//        std::cout << "Intersection points: \n";
//        for (int i = 0; i < pts.size(); ++i)
//        {
//            std::cout << "=====" << "\n";
//            std::cout << pts[i].x << "\n";
//        }
//    }
//
//    {
//        Triangle tr1(Vector3(0, 0, 0), Vector3(0, 0, 1), Vector3(1, 1, 0));
//        Triangle tr2(Vector3(1, 1, 0), Vector3(1, 1, 1), Vector3(0, 0, 1));
//
//        std::vector<Vector3> pts;
//        auto type = GetIntersectionPoints(tr1, tr2, pts);
//
//        assert(type == COPLANE);
//        assert(pts.size() == 0);
//    }
//
//    {
//        Triangle tr1(Vector3(0, 0, 0), Vector3(0, 0, 1), Vector3(1, 1, 0));
//        Triangle tr2(Vector3(1, 0, 1), Vector3(0, 1, 1), Vector3(1, 1, 1));
//
//        std::vector<Vector3> pts;
//        auto type = GetIntersectionPoints(tr1, tr2, pts);
//
//        assert(type == DISJOINT);
//        assert(pts.size() == 0);
//    }
//}

const float EPSILON = std::numeric_limits<float>::epsilon();
bool ComputePointWithLineAndTriangle(const Line3d& line, const Triangle& tri, Vector3& point)
{
    if (InTriangle(tri, line.p1) != -1){
        point = line.p1;
        return true;
    }
    if (InTriangle(tri, line.p2) != -1){
        point = line.p2;
        return true;
    }
    Vector3 e0 = tri.m_pt[1] - tri.m_pt[0];
    Vector3 e1 = tri.m_pt[2] - tri.m_pt[0];

    Vector3 dir = line.p2 - line.p1;
    Vector3 dir_norm = dir.normalise();

    Vector3 h = dir_norm.cross(e1);
    const double a = e0.dot(h);

    if (a > -EPSILON && a < EPSILON) {
        return false;
    }

    Vector3 s = line.p1 - tri.m_pt[0];
    const double f = 1.0f / a;
    const double u = f * s.dot(h);

    if (u < 0.0f || u > 1.0f) {
        return false;
    }

    Vector3 q = s.cross(e0);
    const double v = f * dir_norm.dot(q);

    if (v < 0.0f || u + v > 1.0f) {
        return false;
    }


    const double t = f * e1.dot(q);
    if (t > EPSILON && t < sqrtf(dir.dot(dir))) // segment intersection
    {
        point = line.p1 + dir_norm * t;
        return true;
    }
    return false;
}

bool ComputeLineWithTwoTriangle(const Triangle& tri1, const Triangle& tri2, std::vector<Vector3>& pts)
{
    pts.clear();
    Vector3 point;
    Line3d line;
    std::vector<Vector3> pointArray;
    pointArray.clear();

    line = Line3d(tri1.m_pt[1], tri1.m_pt[0]);
    if (ComputePointWithLineAndTriangle(line, tri2, point)) pointArray.push_back(point);

    line = Line3d(tri1.m_pt[2], tri1.m_pt[1]);
    if (ComputePointWithLineAndTriangle(line, tri2, point)) pointArray.push_back(point);

    line = Line3d(tri1.m_pt[0], tri1.m_pt[2]);
    if (ComputePointWithLineAndTriangle(line, tri2, point)) pointArray.push_back(point);

    line = Line3d(tri2.m_pt[1], tri2.m_pt[0]);
    if (ComputePointWithLineAndTriangle(line, tri1, point)) pointArray.push_back(point);

    line = Line3d(tri2.m_pt[2], tri2.m_pt[1]);
    if (ComputePointWithLineAndTriangle(line, tri1, point)) pointArray.push_back(point);

    line = Line3d(tri2.m_pt[0], tri2.m_pt[2]);
    if (ComputePointWithLineAndTriangle(line, tri1, point)) pointArray.push_back(point);

    //去重复值
    if (pointArray.size() > 2)
    {
        Vector3 tmpv1 = pointArray[0];
        Vector3 tmpv2;
        bool flagtmpv2 = false;
        for (int i = 1; i < pointArray.size(); i++)
        {
            if (tmpv1.distance(pointArray[i]) > EPSILON)
            {
                tmpv2 = pointArray[i];
                flagtmpv2 = true;
                break;
            }
        }
        pointArray.clear();
        pointArray.push_back(tmpv1);
        if (flagtmpv2){
            pointArray.push_back(tmpv2);
        }
    }

    if (pointArray.size() == 2)
    {
        if (abs(pointArray[0].distance(pointArray[1])) < EPSILON)
            return false;
        else
        {
            pts.push_back(pointArray[0]);
            pts.push_back(pointArray[1]);
            return true;
        }
    }
    return false;
}

bool ParallelJudgment(const Vector3& v1, const Vector3& v2)
{
    // 如果其中一个向量为零向量，则任何向量都与它平行
    if (v1.x == 0 && v1.y == 0 && v1.z == 0) return true;
    if (v2.x == 0 && v2.y == 0 && v2.z == 0) return true;

    // 计算两个向量的叉积，如果叉积的模长接近于零，则向量平行
    double crossProductX = v1.y * v2.z - v1.z * v2.y;
    double crossProductY = v1.z * v2.x - v1.x * v2.z;
    double crossProductZ = v1.x * v2.y - v1.y * v2.x;
    double crossProductLength = std::sqrt(crossProductX * crossProductX + crossProductY * crossProductY + crossProductZ * crossProductZ);

    // 使用一个小的容差值来处理浮点数的精度问题
    return crossProductLength < 1e-6;
}


 //判断点是否在三角形内部或边界上
//int isPointInOrOnTriangle3D(const Vector3& p, const Vector3& a, const Vector3& b, const Vector3& c)
//int InTriangle(const Triangle& tri, const Vector3& p)
//{
//    Vector3 a = tri.m_pt[0];
//    Vector3 b = tri.m_pt[1];
//    Vector3 c = tri.m_pt[2];
//
//    // 计算三角形的法向量
//    Vector3 ab = b - a;
//    Vector3 ac = c - a;
//    Vector3 normal = ab.cross(ac);
//
//    // 判断点p是否与三角形共面
//    Vector3 ap = p - a;
//    double tolerance = 1e-5; // 设置阈值以处理浮点数精度问题
//    if (std::fabs(normal.dot(ap)) > tolerance) {
//        return -1; // 点p不在三角形所在的平面上
//    }
//
//    // 计算重心坐标
//    double det = ab.x * (ac.y * ap.z - ac.z * ap.y)
//                 - ab.y * (ac.x * ap.z - ac.z * ap.x)
//                 + ab.z * (ac.x * ap.y - ac.y * ap.x);
//
//    double alpha = ap.x * (ac.y * ab.z - ac.z * ab.y)
//                   - ap.y * (ac.x * ab.z - ac.z * ab.x)
//                   + ap.z * (ac.x * ab.y - ac.y * ab.x) / det;
//    double beta = ab.x * (ap.y * ac.z - ap.z * ac.y)
//                  - ab.y * (ap.x * ac.z - ap.z * ac.x)
//                  + ab.z * (ap.x * ac.y - ap.y * ac.x) / det;
//    double gamma = 1.0 - alpha - beta;
//
//    // 检查重心坐标
//    if (alpha >= 0 && alpha <= 1 && beta >= 0 && beta <= 1 && gamma >= 0 && gamma <= 1) {
//        // 如果所有重心坐标都在[0, 1]内，则点p在三角形内部或边界上
//        if (alpha > 0 && beta > 0 && gamma > 0) {
//            return 1;
//        } else {
//            return 0;
//        }
//    } else {
//        // 点p在三角形外部（但由于前面的检查，这种情况不应该发生）
//        return -1;
//    }
//}
/*
 * -1 : 点在三角形外
 * 0  : 点在三角形边上
 * 1  : 点在三角形内
 * 2  : 点在三角形点上
 */
int InTriangle(const Triangle& tri, const Vector3& pt)
{
    Vector3 ap = tri.m_pt[0] - pt;
    Vector3 apUnit = ap.normalise();
    Vector3 normal = (tri.m_pt[1]-tri.m_pt[0]).cross(tri.m_pt[2]-tri.m_pt[0]);
    Vector3 normalUnit = normal.normalise();
    double inPlane = std::fabs(normalUnit.dot(apUnit));
    if (inPlane > EPSION)
        return -1; // 点p不在三角形所在的平面上
    if (tri.m_pt[0].distance(pt) < EPSILON ||
        tri.m_pt[1].distance(pt) < EPSILON ||
        tri.m_pt[2].distance(pt) < EPSILON )
    {
        return 2;
    }
    if (ParallelJudgment(pt-tri.m_pt[0], tri.m_pt[1]-tri.m_pt[0]) ||
        ParallelJudgment(pt-tri.m_pt[1], tri.m_pt[2]-tri.m_pt[1]) ||
        ParallelJudgment(pt-tri.m_pt[2], tri.m_pt[0]-tri.m_pt[2]))
    {
        return 0;
    }

    Vector3 N0 = (pt-tri.m_pt[0]).cross(tri.m_pt[1]-tri.m_pt[0]);
    Vector3 N1 = (pt-tri.m_pt[1]).cross(tri.m_pt[2]-tri.m_pt[1]);
    Vector3 N2 = (pt-tri.m_pt[2]).cross(tri.m_pt[0]-tri.m_pt[2]);

    Vector3 Nor0 = N0.normalise();
    Vector3 Nor1 = N1.normalise();
    Vector3 Nor2 = N2.normalise();

    if (abs(Nor0.dot(Nor1) - 1) < EPSILON && abs(Nor0.dot(Nor2) - 1) < EPSILON)
//    if (Nor0 == Nor1 && Nor1 == Nor2)
    {
        return 1;
    }
    else
        return -1;
}

