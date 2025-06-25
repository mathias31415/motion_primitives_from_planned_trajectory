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
#include <iostream>

void printPoints(const std::string& title, const rdp::PointList& points) {
    std::cout << title << " (" << points.size() << " points):\n";
    for (const auto& point : points) {
        for (double val : point)
            std::cout << val << " ";
        std::cout << "\n";
    }
    std::cout << std::endl;
}

int main() {
    std::cout << "=== 3D Example ===\n";
    rdp::PointList input3D = {
        {0.0, 0.0, 0.0},
        {1.0, 0.1, 0.2},
        {2.0, -0.1, 0.1},
        {3.0, 0.0, 0.0}
    };

    double epsilon3D = 0.15;
    rdp::PointList simplified3D = rdp::rdpRecursive(input3D, epsilon3D);

    printPoints("Input 3D Points", input3D);
    printPoints("Simplified 3D Points", simplified3D);

    std::cout << "=== 6D Example ===\n";
    rdp::PointList input6D = {
        {1.0, 2.0, 3.0, 4.0, 5.0, 6.0},
        {1.1, 2.1, 3.1, 4.1, 5.1, 6.1},
        {1.2, 2.0, 3.0, 4.2, 5.2, 6.2},
        {1.3, 2.1, 3.0, 4.3, 5.3, 6.3},
        {2.0, 3.0, 4.0, 5.0, 6.0, 7.0}
    };

    double epsilon6D = 0.25;
    rdp::PointList simplified6D = rdp::rdpRecursive(input6D, epsilon6D);

    printPoints("Input 6D Points", input6D);
    printPoints("Simplified 6D Points", simplified6D);

    return 0;
}
