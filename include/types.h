#pragma once

#include "Eigen/Dense"

#include "vector"

using Point2d = Eigen::Vector2d;

struct LineSegment
{
    Point2d p0;
    Point2d p1;
};

/// \brief The obstacle is a convex polygon with a fixed height. It is encoded as an object consisting of an
///        std::vector of the polygon vertices (in clockwise order) in the XY plane, and a height.
struct Obstacle
{
    Obstacle(const std::vector<Eigen::Vector2d>& vertices, float height)
        : vertices(vertices), height(height) {};
    const std::vector<Point2d> vertices{};
    const float height{0.0f};
};

/// \brief A path is a circular arc
struct Path
{
public:
    Path(const Point2d& center,
         float radius,
         float start_angle,
         float end_angle,
         float start_height,
         float end_height)
        : center_(center)
        , radius_(radius)
        , startAngle_(start_angle)
        , endAngle_(end_angle)
        , startHeight_(start_height)
        , endHeight_(end_height){};
    const Point2d center_;
    const float radius_;
    const float startAngle_;
    const float endAngle_;
    const float startHeight_;
    const float endHeight_;
};