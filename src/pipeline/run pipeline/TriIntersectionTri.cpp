//
// Created by 22624 on 2024/5/17.
//

//
// Created by 22624 on 2024/5/17.
//
#include "TriIntersectionTri.h"

//typedef Eigen::Vector3d Point3d;

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

int InTriangle(const Triangle& tri, const Vector3& pt)
{
    if (GetPositionType(tri, pt) == OUT)
        return -1;
    if (GetPositionType(tri, pt) == ON)
        return 0;
    else
        return 1;
}

PositionType GetPositionType(const Triangle& tri, const Vector3& pt)
{
    Eigen::Matrix3d oriented;
    oriented << tri.m_pt[0].x - pt.x, tri.m_pt[0].y - pt.y, tri.m_pt[0].z - pt.z,
            tri.m_pt[1].x - pt.x, tri.m_pt[1].y - pt.y, tri.m_pt[1].z - pt.z,
            tri.m_pt[2].x- pt.x, tri.m_pt[2].y - pt.y, tri.m_pt[2].z - pt.z;

    double value = oriented.determinant();
    if (IsNegative(value))
    {
        return OUT;
    }
    else if (IsPositive(value))
    {
        return IN;
    }

    return ON;
}

Vector3 CalInterPoint(const Vector3& planeNormal, const Vector3& ptOnPlane, const Vector3& lineDir, const Vector3& ptOnLine)
{
    double t = planeNormal.dot(ptOnPlane - ptOnLine) / planeNormal.dot(lineDir);
    return ptOnLine + lineDir * t;
}

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

IntersectionType GetIntersectionPoints(const Triangle& triPlane, const Triangle& triPoints, std::vector<Vector3>& pts)
{
    std::vector<Vector3> ptResult;

    PositionType relateToTriPlane[3] = {
            GetPositionType(triPlane, triPoints.m_pt[0]),
            GetPositionType(triPlane, triPoints.m_pt[1]),
            GetPositionType(triPlane, triPoints.m_pt[2])
    };

    // 平面和直线相交计算
    // 直线 P = V0 + dir*t
    // 平面 Normal \cdot (P - POn) = 0
    // =>
    // t = N \cdot (POn - V0) / N \cdot dir
    if (relateToTriPlane[0] == relateToTriPlane[1] && relateToTriPlane[1] == relateToTriPlane[2])
    {
        if (relateToTriPlane[0] == ON)
        {
            return COPLANE;
        }
        else
        {
            return DISJOINT;
        }
    }
    else if (relateToTriPlane[0] == relateToTriPlane[1])
    {
//        if (relateToTriPlane[0] == OUT)
//        {
//            std::cout << "Test" << std::endl;
//        }
//        else
        {
            Vector3 inter1 = CalInterPoint(triPlane.m_vecNormal, triPlane.m_pt[0], triPoints.m_pt[2] - triPoints.m_pt[0], triPoints.m_pt[2]);
            Vector3 inter2 = CalInterPoint(triPlane.m_vecNormal, triPlane.m_pt[0], triPoints.m_pt[2] - triPoints.m_pt[1], triPoints.m_pt[2]);

            if (CheckPtOnTriangle(triPlane, inter1)) ptResult.push_back(inter1);
            if (CheckPtOnTriangle(triPlane, inter2)) ptResult.push_back(inter2);
        }
    }
    else if (relateToTriPlane[0] == relateToTriPlane[2])
    {
//        if (relateToTriPlane[0] == OUT)
//        {
//            std::cout << "Test" << std::endl;
//        }
//        else
        {
            Vector3 inter1 = CalInterPoint(triPlane.m_vecNormal, triPlane.m_pt[0], triPoints.m_pt[1] - triPoints.m_pt[0], triPoints.m_pt[1]);
            Vector3 inter2 = CalInterPoint(triPlane.m_vecNormal, triPlane.m_pt[0], triPoints.m_pt[1] - triPoints.m_pt[2], triPoints.m_pt[1]);

            if (CheckPtOnTriangle(triPlane, inter1)) ptResult.push_back(inter1);
            if (CheckPtOnTriangle(triPlane, inter2)) ptResult.push_back(inter2);
        }
    }
    else if (relateToTriPlane[1] == relateToTriPlane[2])
    {
//        if (relateToTriPlane[1] == OUT)
//        {
//            std::cout << "Test" << std::endl;
//        }
//        else
        {
            Vector3 inter1 = CalInterPoint(triPlane.m_vecNormal, triPlane.m_pt[0], triPoints.m_pt[0] - triPoints.m_pt[1], triPoints.m_pt[0]);
            Vector3 inter2 = CalInterPoint(triPlane.m_vecNormal, triPlane.m_pt[0], triPoints.m_pt[0] - triPoints.m_pt[2], triPoints.m_pt[0]);

            if (CheckPtOnTriangle(triPlane, inter1)) ptResult.push_back(inter1);
            if (CheckPtOnTriangle(triPlane, inter2)) ptResult.push_back(inter2);
        }
    }
    else // 有一个点位于三角平面上，另外两个点分别位于两边
    {
        if (relateToTriPlane[0] == ON && CheckPtOnTriangle(triPlane, triPoints.m_pt[0])) ptResult.push_back(triPoints.m_pt[0]);
        else if (relateToTriPlane[1] == ON && CheckPtOnTriangle(triPlane, triPoints.m_pt[1])) ptResult.push_back(triPoints.m_pt[0]);
        else if (relateToTriPlane[2] == ON && CheckPtOnTriangle(triPlane, triPoints.m_pt[2])) ptResult.push_back(triPoints.m_pt[0]);
    }

    if (ptResult.empty())
    {
        return DISJOINT;
    }

    std::move(begin(ptResult), end(ptResult), back_inserter(pts));

    return INTERSECTION;
}

void TriIntersectTestCase()
{
    {
        Triangle tr1(Vector3(0, 0, 0), Vector3(1, 0, 1), Vector3(0, 1, 1));
        Triangle tr2(Vector3(1, 1, 0), Vector3(1, 1, 1), Vector3(0, 0, 1));

        std::vector<Vector3> pts;
        auto type = GetIntersectionPoints(tr1, tr2, pts);

        assert(type == INTERSECTION);
        std::cout << "Intersection points: \n";
        for (int i = 0; i < pts.size(); ++i)
        {
            std::cout << "=====" << "\n";
            std::cout << pts[i].x << "\n";
        }
    }

    {
        Triangle tr1(Vector3(0, 0, 0), Vector3(0, 0, 1), Vector3(1, 1, 0));
        Triangle tr2(Vector3(1, 1, 0), Vector3(1, 1, 1), Vector3(0, 0, 1));

        std::vector<Vector3> pts;
        auto type = GetIntersectionPoints(tr1, tr2, pts);

        assert(type == COPLANE);
        assert(pts.size() == 0);
    }

    {
        Triangle tr1(Vector3(0, 0, 0), Vector3(0, 0, 1), Vector3(1, 1, 0));
        Triangle tr2(Vector3(1, 0, 1), Vector3(0, 1, 1), Vector3(1, 1, 1));

        std::vector<Vector3> pts;
        auto type = GetIntersectionPoints(tr1, tr2, pts);

        assert(type == DISJOINT);
        assert(pts.size() == 0);
    }
}
