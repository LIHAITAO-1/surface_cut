//
// Created by xmyci on 20/02/2024.
//

#ifndef TETGEO_PREDICATES_WRAPPER_H
#define TETGEO_PREDICATES_WRAPPER_H

#include <immintrin.h>//AVX family/SVML
#include <mmintrin.h> //MMX
#include <tmmintrin.h>//SSE family
#include "basic/math/vector3.h"


namespace Geometrical_Predicates {
    using namespace base_type;

    struct IntersectionResult3d {
        bool intersect;
        Vector3 intersectionPoint;
    };

    struct Tetrahedra {
        Vector3 p1;
        Vector3 p2;
        Vector3 p3;
        Vector3 p4;
    };

    struct Triangle3d {
        Vector3 p1;
        Vector3 p2;
        Vector3 p3;
    };

//    struct Edge {
//        Vector3 p1;
//        Vector3 p2;
//    };

    struct Line {
        Vector3 p1;
        Vector3 p2;
    };

    struct Line3d {
        Vector3 p1;
        Vector3 p2;
    };

    struct Ray3d {
        Vector3 from;
        Vector3 to;
    };

    struct Plane {
        Vector3 p;
        Vector3 normal;
    };

    inline bool lu_decmp(double lu[4][4], int n, int *ps, double *d, int N) {
        double scales[4];
        double pivot, biggest, mult, tempf;
        int pivotindex = 0;
        int i, j, k;

        *d = 1.0; // No row interchanges yet.

        for (i = N; i < n + N; i++) {
            // For each row.
            // Find the largest element in each row for row equilibration
            biggest = 0.0;
            for (j = N; j < n + N; j++)
                if (biggest < (tempf = fabs(lu[i][j])))
                    biggest = tempf;
            if (biggest != 0.0)
                scales[i] = 1.0 / biggest;
            else {
                scales[i] = 0.0;
                return false; // Zero row: singular matrix.
            }
            ps[i] = i; // Initialize pivot sequence.
        }

        for (k = N; k < n + N - 1; k++) {
            // For each column.
            // Find the largest element in each column to pivot around.
            biggest = 0.0;
            for (i = k; i < n + N; i++) {
                if (biggest < (tempf = fabs(lu[ps[i]][k]) * scales[ps[i]])) {
                    biggest = tempf;
                    pivotindex = i;
                }
            }
            if (biggest == 0.0) {
                return false; // Zero column: singular matrix.
            }
            if (pivotindex != k) {
                // Update pivot sequence.
                j = ps[k];
                ps[k] = ps[pivotindex];
                ps[pivotindex] = j;
                *d = -(*d); // ...and change the parity of d.
            }

            // Pivot, eliminating an extra variable  each time
            pivot = lu[ps[k]][k];
            for (i = k + 1; i < n + N; i++) {
                lu[ps[i]][k] = mult = lu[ps[i]][k] / pivot;
                if (mult != 0.0) {
                    for (j = k + 1; j < n + N; j++)
                        lu[ps[i]][j] -= mult * lu[ps[k]][j];
                }
            }
        }

        // (lu[ps[n + N - 1]][n + N - 1] == 0.0) ==> A is singular.
        return lu[ps[n + N - 1]][n + N - 1] != 0.0;
    }

    inline void lu_solve(double lu[4][4], int n, int *ps, double *b, int N) {
        int i, j;
        double X[4], dot;

        for (i = N; i < n + N; i++) X[i] = 0.0;

        // Vector reduction using U triangular matrix.
        for (i = N; i < n + N; i++) {
            dot = 0.0;
            for (j = N; j < i + N; j++)
                dot += lu[ps[i]][j] * X[j];
            X[i] = b[ps[i]] - dot;
        }

        // Back substitution, in L triangular matrix.
        for (i = n + N - 1; i >= N; i--) {
            dot = 0.0;
            for (j = i + 1; j < n + N; j++)
                dot += lu[ps[i]][j] * X[j];
            X[i] = (X[i] - dot) / lu[ps[i]][i];
        }

        for (i = N; i < n + N; i++) b[i] = X[i];
    }

    inline Vector3 cross(const Vector3 &p1, const Vector3 &p2);

    inline double dot(const Vector3 &p1, const Vector3 &p2);

    inline double dot_avx(const Vector3 &p1, const Vector3 &p2);

    inline bool toleft(const Vector3 &p1, const Vector3 &p2, const Vector3 &s);

    inline bool toleft(const Vector3 &p1, const Vector3 &p2, const Vector3 &p3, const Vector3 &s);

    inline bool colinear(const Vector3 &a, const Vector3 &b, const Vector3 &c);

    inline bool coplanar(const Vector3 &a, const Vector3 &b, const Vector3 &c, const Vector3 &d);

    inline double plane_point_distance(Plane plane, Vector3 point);

    inline double vector_length_sqr(Vector3 a);

    inline double vector_length(Vector3 a);

    inline double vector_length_sqr(Vector3 a, Vector3 b);

    inline double vector_length(Vector3 a, Vector3 b);

    //implement begin
    inline Vector3 cross(const Vector3 &p1, const Vector3 &p2) {
        //(u1, u2, u3) x(v1, v2, v3) = (u2v3 - u3v2; u3v1 - u1v3, u1v2 - u2v1)
        double s1, s2, s3;
        s1 = p1.y * p2.z - p1.z * p2.y;
        s2 = p1.z * p2.x - p1.x * p2.z;
        s3 = p1.x * p2.y - p1.y * p2.x;
        return Vector3(s1, s2, s3);
    }

    inline double dot(double *p1, double *p2) {
        return p1[0] * p2[0] + p1[1] * p2[1] + p1[2] * p2[2];
    }

    inline double dot(const Vector3 &p1, const Vector3 &p2) {
        return p1.x * p2.x + p1.y * p2.y + p1.z * p2.z;
    }

    inline bool toleft(const Vector3 &p1, const Vector3 &p2, const Vector3 &s) {
        double value = p1.x * p2.y - p1.y * p2.x
                       + p2.x * s.y - p2.y * s.x
                       + s.x * p1.y - s.y * p1.x;
        return value > 0;
    }

    inline bool toleft(const Vector3 &p1, const Vector3 &p2, const Vector3 &p3, const Vector3 &s) {
        //ORIENT3D

        double a11, a12, a13;
        double a21, a22, a23;
        double a31, a32, a33;

        a11 = p1.x - s.x;
        a12 = p1.y - s.y;
        a13 = p1.z - s.z;
        a21 = p2.x - s.x;
        a22 = p2.y - s.y;
        a23 = p2.z - s.z;
        a31 = p3.x - s.x;
        a32 = p3.y - s.y;
        a33 = p3.z - s.z;

        double value = a11 * a22 * a33 + a12 * a23 * a31 + a13 * a21 * a32 - a31 * a22 * a13 - a32 * a23 * a11 - a21 * a12 * a33;

        return value > 1e-8;
    }

    inline bool colinear(const Vector3 &a, const Vector3 &b, const Vector3 &c) {
        return ((c.z - a.z) * (b.y - a.y) -
                (b.z - a.z) * (c.y - a.y)) == 0 && \
         ((b.z - a.z) * (c.x - a.x) -
          (b.x - a.x) * (c.z - a.z)) == 0 && \
         ((b.x - a.x) * (c.y - a.y) -
          (b.y - a.y) * (c.x - a.x)) == 0;
    }

    inline bool coplanar(const Vector3 &a, const Vector3 &b, const Vector3 &c, const Vector3 &d) {
        assert(colinear(a, b, c) == false);
        double disance = plane_point_distance({a, (b - a).cross(c - a).normalise()}, d);
        return disance < 1e-4;
    }

    inline double plane_point_distance(Plane plane, Vector3 point) {
        Vector3 Q = point - plane.p;
        return std::fabs(dot(/*VectorNormal*/(plane.normal), Q));
    }

    inline double vector_length_sqr(Vector3 a) {
        return (a.x) * (a.x) + (a.y) * (a.y) + (a.z) * (a.z);
    }

    inline double vector_length(Vector3 a) {
        return std::sqrt((a.x) * (a.x) + (a.y) * (a.y) + (a.z) * (a.z));
    }

    inline double vector_length_sqr(Vector3 a, Vector3 b) {
        return (a.x - b.x) * (a.x - b.x) + (a.y - b.y) * (a.y - b.y) + (a.z - b.z) * (a.z - b.z);
    }

    inline base_type::Vector3 vector_normal(base_type::Vector3 a) {
        return a / std::sqrt(vector_length_sqr(a));
    }

    inline double vector_length_subt(Vector3 a, Vector3 b) {
        double d_x = std::abs(a.x - b.x);
        double d_y = std::abs(a.y - b.y);
        double d_z = std::abs(a.z - b.z);

        return d_x * d_x * d_x + d_y * d_y * d_y + d_z * d_z * d_z;
    }

    inline double vector_length(Vector3 a, Vector3 b) {
        return std::sqrt((a.x - b.x) * (a.x - b.x) + (a.y - b.y) * (a.y - b.y) + (a.z - b.z) * (a.z - b.z));
    }

    double dot_avx(const Vector3 &p1, const Vector3 &p2) {
        return 0;
    }

    inline double get_triangle_area(Triangle3d t) {
        auto A = t.p1;
        auto B = t.p2;
        auto C = t.p3;

        double S = vector_length(cross(B - A, C - A)) / 2.0;
        return S;
    };

    inline double tetrahedra_volume(Tetrahedra t) {
        //https://math.stackexchange.com/questions/1603651/volume-of-tetrahedron-using-cross-and-dot-product
        auto a = t.p2 - t.p3;
        auto b = t.p4 - t.p3;
        auto c = t.p1 - t.p3;
        return fabs(dot(cross(a, b), c) / 6.0);
    }
    inline bool is_point_inside_triangle(Triangle3d tri, base_type::Vector3 p) {
        using namespace Geometrical_Predicates;

        if (tri.p1 == p || tri.p2 == p || tri.p3 == p)
            return true;

        auto get_triangle_S = [](Triangle3d t) -> double {
            auto A = t.p1;
            auto B = t.p2;
            auto C = t.p3;

            double S = vector_length(cross(B - A, C - A));
            return S;
        };

        double S_ABC, S_ABP, S_ACP, S_BCP;
        S_ABC = get_triangle_S(tri);
        S_ABP = get_triangle_S({tri.p1, tri.p2, p});
        S_ACP = get_triangle_S({tri.p1, tri.p3, p});
        S_BCP = get_triangle_S({tri.p2, tri.p3, p});

        return fabs(S_ABC - (S_ABP + S_ACP + S_BCP)) < 1e-8;
    }

    inline IntersectionResult3d ray_intersection_calulate_triangle(Triangle3d tri, Ray3d ray, bool include_border) {
        using namespace Geometrical_Predicates;

        auto ray_dir = (ray.to - ray.from);
        //auto ray_dir_normal = VectorNormal(ray_dir);
        auto ray_origin = ray.from;

        auto u = tri.p2 - tri.p1;
        auto v = tri.p3 - tri.p1;
        auto Q = tri.p1;

        base_type::Vector3 cvu = cross(u, v);
        base_type::Vector3 plane_normal = vector_normal(cvu);

        auto denom = dot(plane_normal, ray_dir);
        if (fabs(denom) < 1e-8) {
            return {false};
        }

        double D = dot(plane_normal, Q);
        double t = (D - dot(plane_normal, ray_origin)) / denom;
        if (t < 0) {
            return {false};
        }

        base_type::Vector3 intersection = ray_origin + ray_dir * t;

        if (include_border) {
            bool b = is_point_inside_triangle(tri, intersection);//include border and non-border

            if (b)
                return {true, intersection};
        }
        else {  //include non-border
            base_type::Vector3 planar_hitpt_vector = intersection - Q;
            auto w = cvu / dot(cvu, cvu);
            auto alpha = dot(w, cross(planar_hitpt_vector, v));
            auto beta = dot(w, cross(u, planar_hitpt_vector));

            //auto aa = alpha * u + beta * v + Q;
            //ASSERT(VectorLengthSqr(aa - intersection) < 1e-8);

            if (fabs(alpha) < 1e-8 || fabs(beta) < 1e-8) {
                return {false};
            }

            if ((alpha + beta) < 0.99999999999 && alpha > 1e-8 && beta > 1e-8) {
                return {true, intersection};
            }
        }
        return {false};
    }

    template<typename PointT>
    bool calculate_bounding_sphere(const PointT &pa, const PointT &pb, const PointT &pc, const PointT &pd, PointT &center, double &radius, double &radius2) {

        auto lu_solve = [](double lu[4][4], int n, int *ps, double *b, int N) {
            int i, j;
            double X[4], dot;

            for (i = N; i < n + N; i++) X[i] = 0.0;

            // Vector reduction using U triangular matrix.
            for (i = N; i < n + N; i++) {
                dot = 0.0;
                for (j = N; j < i + N; j++)
                    dot += lu[ps[i]][j] * X[j];
                X[i] = b[ps[i]] - dot;
            }

            // Back substitution, in L triangular matrix.
            for (i = n + N - 1; i >= N; i--) {
                dot = 0.0;
                for (j = i + 1; j < n + N; j++)
                    dot += lu[ps[i]][j] * X[j];
                X[i] = (X[i] - dot) / lu[ps[i]][i];
            }

            for (i = N; i < n + N; i++) b[i] = X[i];
        };

        double A[4][4], b[4], D;
        int indx[4];

        // Compute the coefficient matrix A (3x3).
        A[0][0] = pb.x - pa.x;
        A[0][1] = pb.y - pa.y;
        A[0][2] = pb.z - pa.z;

        A[1][0] = pc.x - pa.x;
        A[1][1] = pc.y - pa.y;
        A[1][2] = pc.z - pa.z;

        A[2][0] = pd.x - pa.x;
        A[2][1] = pd.y - pa.y;
        A[2][2] = pd.z - pa.z;

        // Compute the matrix b (3).
        b[0] = 0.5 * dot(A[0], A[0]);
        b[1] = 0.5 * dot(A[1], A[1]);
        b[2] = 0.5 * dot(A[2], A[2]);

        if (!lu_decmp(A, 3, indx, &D, 0)) {
            radius = 0.0;
            return false;
        }
        lu_solve(A, 3, indx, b, 0);


        center.x = pa.x + b[0];
        center.y = pa.y + b[1];
        center.z = pa.z + b[2];

        radius2 = b[0] * b[0] + b[1] * b[1] + b[2] * b[2];
        radius = std::sqrt(radius2);

        return true;
    }

    template<typename PointT>
    void calculate_bounding_circle(const PointT &p1, const PointT &p2, const PointT &p3, PointT &center, double &radius) {
        // Calculating lengths of the sides of the triangle formed by the coordinates
        double x1 = p1.x, y1 = p1.y, x2 = p2.x, y2 = p2.y, x3 = p3.x, y3 = p3.y;

        double a = std::sqrt((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1));
        double b = std::sqrt((x3 - x1) * (x3 - x1) + (y3 - y1) * (y3 - y1));
        double c = std::sqrt((x3 - x2) * (x3 - x2) + (y3 - y2) * (y3 - y2));

        // Calculating the radius of the circumscribed circle using triangle sides
        radius = (a * b * c) / (sqrt((a + b + c) * (b + c - a) * (c + a - b) * (a + b - c)));

        // Calculating the coordinates of the center of the circumscribed circle (x, y)
        double d = 2 * (x1 * (y2 - y3) + x2 * (y3 - y1) + x3 * (y1 - y2));
        center.x = ((x1 * x1 + y1 * y1) * (y2 - y3) + (x2 * x2 + y2 * y2) * (y3 - y1) + (x3 * x3 + y3 * y3) * (y1 - y2)) / d;
        center.y = ((x1 * x1 + y1 * y1) * (x3 - x2) + (x2 * x2 + y2 * y2) * (x1 - x3) + (x3 * x3 + y3 * y3) * (x2 - x1)) / d;
    }

}


#endif //TETGEO_PREDICATES_WRAPPER_H
