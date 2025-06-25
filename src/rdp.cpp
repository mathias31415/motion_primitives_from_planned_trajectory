// Copyright (c) 2025, b»robotized
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// Authors: Mathias Fuhrer

#include "motion_primitives_from_planned_trajectory/rdp.hpp"
#include <cmath>
#include <cstddef>

namespace rdp {

// Computes the dot product of two vectors
double dot(const Point& a, const Point& b) {
    double result = 0.0;
    for (std::size_t i = 0; i < a.size(); ++i)
        result += a[i] * b[i];
    return result;
}

// Computes the Euclidean norm (length) of a vector
double norm(const Point& v) {
    return std::sqrt(dot(v, v));
}

// Subtracts two vectors
Point operator-(const Point& a, const Point& b) {
    Point result(a.size());
    for (std::size_t i = 0; i < a.size(); ++i)
        result[i] = a[i] - b[i];
    return result;
}

// Adds two vectors
Point operator+(const Point& a, const Point& b) {
    Point result(a.size());
    for (std::size_t i = 0; i < a.size(); ++i)
        result[i] = a[i] + b[i];
    return result;
}

// Multiplies a vector by a scalar
Point operator*(double scalar, const Point& v) {
    Point result(v.size());
    for (std::size_t i = 0; i < v.size(); ++i)
        result[i] = scalar * v[i];
    return result;
}

// Computes the perpendicular distance from a point to a line in N-dimensional space
double pointLineDistanceND(const Point& point, const Point& start, const Point& end) {
    Point lineVec = end - start;
    Point pointVec = point - start;
    double len = norm(lineVec);

    if (len == 0.0)
        return norm(point - start);

    double projection = dot(pointVec, lineVec) / len;
    Point closest = start + (projection / len) * lineVec;
    return norm(point - closest);
}

// Recursive implementation of the Ramer-Douglas-Peucker algorithm
PointList rdpRecursive(const PointList& points, double epsilon) {
    if (points.size() < 2)
        return points;

    double dmax = 0.0;
    std::size_t index = 0;

    // Find the point with the maximum distance from the line segment
    for (std::size_t i = 1; i < points.size() - 1; ++i) {
        double d = pointLineDistanceND(points[i], points.front(), points.back());
        if (d > dmax) {
            index = i;
            dmax = d;
        }
    }

    // If the max distance is greater than epsilon, recursively simplify
    if (dmax > epsilon) {
        PointList rec1(points.begin(), points.begin() + index + 1);
        PointList rec2(points.begin() + index, points.end());

        PointList res1 = rdpRecursive(rec1, epsilon);
        PointList res2 = rdpRecursive(rec2, epsilon);

        // Combine results, avoiding duplicate of the middle point
        res1.pop_back();
        res1.insert(res1.end(), res2.begin(), res2.end());
        return res1;
    } else {
        // No point is far enough — keep only endpoints
        return {points.front(), points.back()};
    }
}

} // namespace rdp
