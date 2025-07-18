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

#ifndef RDP_HPP
#define RDP_HPP

#include <vector>

namespace rdp {

using Point = std::vector<double>;
using PointList = std::vector<Point>;

/**
 * Computes the shortest distance from a point to a line in N-dimensional space.
 *
 * @param point The point to measure from.
 * @param start The start of the line segment.
 * @param end The end of the line segment.
 * @return Distance from the point to the line segment.
 */
double pointLineDistanceND(const Point& point, const Point& start, const Point& end);

/**
 * Recursive implementation of the Ramer-Douglas-Peucker algorithm
 * for simplifying a series of N-dimensional points.
 *
 * @param points The list of input points.
 * @param epsilon Tolerance value: points closer than this distance to the line will be removed.
 * @param offset Offset for original indices (used in recursion).
 * @return A pair consisting of:
 *         - The simplified list of points.
 *         - The list of indices (from the original input) corresponding to the retained points.
 */
std::pair<PointList, std::vector<std::size_t>> rdpRecursive(const PointList& points, double epsilon, std::size_t offset = 0);

} // namespace rdp

#endif // RDP_HPP