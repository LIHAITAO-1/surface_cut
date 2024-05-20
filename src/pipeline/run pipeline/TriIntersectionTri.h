//
// Created by 22624 on 2024/5/17.
//

#ifndef TETGEO_V2_TRIINTERSECTIONTRI_H
#define TETGEO_V2_TRIINTERSECTIONTRI_H

#endif //TETGEO_V2_TRIINTERSECTIONTRI_H

#pragma once

#include <iostream>
#include <cmath>
#include <algorithm>
#include <assert.h>
#include <vector>
#include <iterator>

#include "basic/math/vector3.h"
#include "Eigen/Dense"

//typedef Eigen::Vector3d Point3d;

using namespace base_type;

class Triangle
{
public:
    Triangle(Vector3 pt0, Vector3 pt1, Vector3 pt2) : m_pt {pt0, pt1, pt2}
    {
        auto vecPt0TPt1 = pt1 - pt0;
        auto vecPt0TPt2 = pt2 - pt0;
        m_vecNormal = vecPt0TPt1.cross(vecPt0TPt2);
        m_vecNormal.normalise();
    }

    double GetDistanceFromPointToTrianglePlane(Vector3 pt) const
    {
        auto vecPtTPt0 = m_pt[0] - pt;

        return m_vecNormal.dot(vecPtTPt0);
    }

    Vector3 m_pt[3];
    Vector3 m_vecNormal;
};

enum IntersectionType
{
    INTERSECTION,             //< 有相交线段
    DISJOINT,                 //< 不相交
    COPLANE                   //< 共面
};


enum PositionType
{
    IN,
    OUT,
    ON
};

int InTriangle(const Triangle& tri, const Vector3& pt);

PositionType GetPositionType(const Triangle& tri, const Vector3& pt);

Vector3 CalInterPoint(const Eigen::Vector3d& planeNormal, const Vector3& ptOnPlane, const Eigen::Vector3d& lineDir, const Vector3& ptOnLine);

bool CheckPtOnTriangle(const Triangle& tri, const Vector3& pt);

IntersectionType GetIntersectionPoints(const Triangle& triPlane, const Triangle& triPoints, std::vector<Vector3>& pts);

void TriIntersectTestCase();