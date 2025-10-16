#include "arc_line_intersection.h"
#include "../include/arc_line_intersection.h"

#include <iostream>
#include <optional>

#include "../include/types.h"

double ArcLineIntersection::perpendicularDistance(const LineSegment& segment, const Path& path)
{
    /**
     * This uses the formula for the distance from a point to a line.
     * d = | D x V | / |D|
     * where D is the direction vector of the line segment, and V is the vector from
     * one endpoint of the segment to the center of the arc.
     */
    const Eigen::Vector2d D = segment.p1 - segment.p0;
    const Eigen::Vector2d V = path.center_ - segment.p0;
    double dNorm = D.norm();
    if (dNorm == 0)
    {
        return std::numeric_limits<double>::infinity();
    }
    auto cross = D.x() * V.y() - D.y() * V.x();
    return std::abs(cross) / dNorm;
}

std::optional<Point2d> ArcLineIntersection::intersectionPoint(const LineSegment& segment, const Path& path)
{
    /**
     * This is using discriminant method to find intersection points between a line and an arc.
     * Putting my notes here for reference:
     * Line segment point where line intersects circle:
     * Q(t) = P0 + t * D, t in [0, 1]
     * where P0 is segment.p0, D = segment.p1 - segment.p0
     * Now, for that point to be on the circle:
     * |Q(t) - C|^2 = a_r^2. C is center of circle, a_r is arc radius
     * Expanding:
     * (P0 + t * D - C) . (P0 + t * D - C) = a_r^2
     * (D . D) t^2 + 2 D . (P0 - C) t + (P0 - C) . (P0 - C) - a_r^2 = 0
     * This is a quadratic equation in t, with:
     * A = D . D
     * B = 2 D . (P0 - C)
     * C = (P0 - C) . (P0 - C) - a_r^2
     * Discriminant = B^2 - 4AC
     * If Discriminant < 0, no intersection
     */
    auto D = segment.p1 - segment.p0; // Delta vector of line segment
    auto a = segment.p0 - path.center_; // Vector from circle center to line segment start
    double A = D.dot(D);
    double B = 2 * D.dot(a);
    double C = a.dot(a) - path.radius_ * path.radius_;
    double discriminant = B * B - 4 * A * C;
    if (discriminant < 0)
    {
        return std::nullopt; // No intersection
    }
    // One or two intersections
    double sqrtDiscriminant = std::sqrt(discriminant);
    double t1 = (-B + sqrtDiscriminant) / (2 * A);
    double t2 = (-B - sqrtDiscriminant) / (2 * A);
    bool t1Valid = (t1 >= 0.0 && t1 <= 1.0);
    bool t2Valid = (t2 >= 0.0 && t2 <= 1.0);
    if (t1Valid && t2Valid)
    {
        // Two valid intersections, return the one closer to segment.p0
        return (t1 < t2) ? (segment.p0 + t1 * D) : (segment.p0 + t2 * D);
    }
    else if (t1Valid)
    {
        return segment.p0 + t1 * D;
    }
    else if (t2Valid)
    {
        return segment.p0 + t2 * D;
    }
    else
    {
        return std::nullopt; // Both intersections are out of segment bounds
    }
}

bool ArcLineIntersection::isWithinArcAngleAndHeight(const Point2d& point, double segmentHeight,
                                                                        const Path& path)
{
    // Calculate angle of point relative to arc center
    Eigen::Vector2d vec = point - path.center_;
    double tAngle = std::atan2(vec.y(), vec.x()); // Angle in radians
    tAngle = (tAngle * 180.0) / M_PI; // Convert to degrees

    // we want that angle to be against Y axis, clockwise positive cause we are given angles in that way
    tAngle = 90.0 - tAngle;

    // Normalize angles to [0, 2pi]
    auto normalizeAngle = [](double a) {
        a = fmod(a, 2 * M_PI);
        if (a < 0)
            a += 2 * M_PI;
        return a;
    };

    double startAngle = normalizeAngle(path.startAngle_);
    double endAngle = normalizeAngle(path.endAngle_);
    tAngle = normalizeAngle(tAngle);

    auto tAlongStart = normalizeAngle(tAngle - startAngle);
    auto arcSpan = normalizeAngle(endAngle - startAngle);
    if (tAlongStart > arcSpan)
    {
        return false;
    }

    // find height at that point
    double angularProgress = arcSpan > 0 ? (tAlongStart / arcSpan) : 0.0;
    double heightAtPoint = path.startHeight_ + angularProgress * (path.endHeight_ - path.startHeight_);

    // Gate against z - height of line segment (convex polygon)
    return heightAtPoint <= segmentHeight;
}

bool ArcLineIntersection::Intersect(const Obstacle& obstacle, const Path& path)
{
    bool result = false;
    // Iterate over segment to find whether path intersects with any of its edges
    for (auto iIndex = 0; iIndex < obstacle.vertices.size(); iIndex++)
    {
        LineSegment segment;
        segment.p0 = obstacle.vertices[iIndex];
        segment.p1 = obstacle.vertices[(iIndex + 1) % obstacle.vertices.size()];
        segment.height = obstacle.height;

        result = ArcLineIntersection::isPathIntersectingSegment(segment, path);
        if (true == result)
        {
            return result;
        }
    }

    // Lastly check whether whole arc is inside the polygon
    auto [arcStart, arcEnd] = ArcLineIntersection::getStartAndEndPointsOfPath(path);
    if (isPointInPolygon(arcStart, obstacle) && isPointInPolygon(arcEnd, obstacle))
    {
        result = true;
    }

    return result;
}

bool ArcLineIntersection::isPathIntersectingSegment(const LineSegment& segment, const Path& path)
{
    // First do a quick check using perpendicular distance from line to arc center
    double perpDist = perpendicularDistance(segment, path);
    if (perpDist > path.radius_)
    {
        return false; // No intersection possible
    }

    // Now find the actual intersection point(s) between the line segment and the arc's circle
    auto intersectionOpt = intersectionPoint(segment, path);
    if (!intersectionOpt.has_value())
    {
        return false; // No intersection points within segment bounds
    }
    auto intersection = intersectionOpt.value();

    // Check if the intersection point is within the arc's angular span and below the height of the segment
    auto heightCheck = isWithinArcAngleAndHeight(intersection, segment.height, path);
    return heightCheck;
}

bool ArcLineIntersection::isPointInPolygon(const Point2d& point, const Obstacle& polygon)
{
    // Use cross-product method to determine if point is inside polygon
    // we are given that polygon is convex and vertices are in clockwise order
    // So we will use cross-product from each edge and edge to point vector to determine if point is inside
    // If all cross products have the same sign, point is inside
    bool inside = false;
    auto n = polygon.vertices.size();
    for (auto iIndex = 0; iIndex < n; iIndex++)
    {
        auto v0 = polygon.vertices[iIndex];
        auto v1 = polygon.vertices[(iIndex + 1) % n];
        Eigen::Vector2d edge = v1 - v0;
        Eigen::Vector2d toPoint = point - v0;
        double cross = edge.x() * toPoint.y() - edge.y() * toPoint.x();
        // if cross is zero, point is on the edge, we consider that inside
        if (iIndex == 0)
        {
            inside = cross >= 0; // Initialize inside based on first cross product. we will match this sign for all others
        }
        else
        {
            if ((cross >= 0) != inside)
            {
                return false; // Point is outside
            }
        }
    }
    return inside;
}

ArcLineIntersection::StartAndEndPointsOfArc ArcLineIntersection::getStartAndEndPointsOfPath(const Path& path)
{
    // sine/cosine expects angle in radia
    // n from X axis
    auto thetaFromXAxis = [](double angleInDegrees) {
        double angleInRadians = (90.0 - angleInDegrees) * (M_PI / 180.0);
        return angleInRadians;
    };
    Eigen::Vector2d startVec(std::cos(thetaFromXAxis(path.startAngle_)), std::sin(thetaFromXAxis(path.startAngle_)));
    Eigen::Vector2d endVec(std::cos(thetaFromXAxis(path.endAngle_)), std::sin(thetaFromXAxis(path.endAngle_)));
    StartPointOfArc startPoint = path.center_ + path.radius_ * startVec;
    EndPointOfArc endPoint = path.center_ + path.radius_ * endVec;
    return {startPoint, endPoint};
}