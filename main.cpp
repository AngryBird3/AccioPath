#include <iostream>
#include "include/arc_line_intersection.h"
// TIP To <b>Run</b> code, press <shortcut actionId="Run"/> or click the <icon src="AllIcons.Actions.Execute"/> icon in the gutter.
int main()
{
    Path path1{{0, 0}, 15, 0, 90, 25, 15};

    Obstacle obstacle1{std::vector<Eigen::Vector2d>({{10, 10}, {10, 0}, {0, 10}}), 30};
    Obstacle obstacle{}

    auto path1Obstacle1Result = ArcLineIntersection::Intersect(obstacle1, path1);
    std::cout << path1Obstacle1Result; // False

    return 0;
}