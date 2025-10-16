#include <iostream>
#include "include/arc_line_intersection.h"
// TIP To <b>Run</b> code, press <shortcut actionId="Run"/> or click the <icon src="AllIcons.Actions.Execute"/> icon in the gutter.
int main()
{
    Path path1{{0, 0}, 15, 0, 90, 25, 15};
    Path path2{{10, 10}, 5, -45, 225, 18, 30};
    Path path3{{10, -10}, 15, -15, 15, 60, 10};

    Obstacle obstacle1{std::vector<Eigen::Vector2d>({{10, 10}, {10, 0}, {0, 10}}), 30};
    Obstacle obstacle2{std::vector<Eigen::Vector2d>({{10, 0}, {10, 10}, {20, 10}, {20,
0}}), 20};
    Obstacle obstacle3{std::vector<Eigen::Vector2d>({{0, 10}, {0, 20}, {10, 20}, {10,
10}}), 20};

    auto path1Obstacle1Result = ArcLineIntersection::Intersect(obstacle1, path1);
    std::cout << "path1 intersects with obstacle1? " << path1Obstacle1Result << "\n"; // False
    auto path1Obstacle2Result = ArcLineIntersection::Intersect(obstacle2, path1);
    std::cout << "path1 intersects with obstacle2? " << path1Obstacle2Result <<  "\n"; // True
    auto path1Obstacle3Result = ArcLineIntersection::Intersect(obstacle3, path1);
    std::cout << "path1 intersects with obstacle3? " << path1Obstacle3Result <<  "\n"; // False

    auto path2Obstacle1Result = ArcLineIntersection::Intersect(obstacle1, path2);
    std::cout << "path2 intersects with obstacle1? " << path2Obstacle1Result <<  "\n"; // True
    auto path2Obstacle2Result = ArcLineIntersection::Intersect(obstacle2, path2);
    std::cout << "path2 intersects with obstacle2? " << path2Obstacle2Result <<  "\n"; // False
    auto path2Obstacle3Result = ArcLineIntersection::Intersect(obstacle3, path2);
    std::cout << "path2 intersects with obstacle3? " << path2Obstacle3Result <<  "\n"; // True

    auto path3Obstacle1Result = ArcLineIntersection::Intersect(obstacle1, path3);
    std::cout << "path3 intersects with obstacle1? " << path3Obstacle1Result <<  "\n"; // False
    auto path3Obstacle2Result = ArcLineIntersection::Intersect(obstacle2, path3);
    std::cout << "path3 intersects with obstacle2? " << path3Obstacle2Result <<  "\n"; // True
    auto path3Obstacle3Result = ArcLineIntersection::Intersect(obstacle3, path3);
    std::cout << "path3 intersects with obstacle3? " << path3Obstacle3Result <<  "\n"; // False

    return 0;
}