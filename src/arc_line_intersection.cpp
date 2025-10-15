#include "arc_line_intersection.h"

#include <optional>

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

std::optional<Point2d> intersectionPoint(const LineSegment& segment, const Path& path)
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
    else
    {
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

}