#pragma once

#include "types.h"

class ArcLineIntersection
{
public:
    static bool Intersect(const LineSegment& segment,
                          const Path& path);
private:
    /**
     * Calculate the perpendicular distance from a line segment to a circular arc path.
     * @param segment
     * @param path
     * @return
     */
    static double perpendicularDistance(const LineSegment& segment, const Path& path);

    /**
     * Find the intersection point between a line segment and a circular arc path, if it exists.
     * @param segment
     * @param path
     * @return
     */
    static std::optional<Point2d> intersectionPoint(const LineSegment& segment, const Path& path);

    /**
     * Check if a point is within the angular span of the arc.
     * @param point
     * @param path
     * @return interpolated height at that point if point is within arc angle, std::nullopt otherwise
     */
    static std::optional<double> isWithinArcAngle(const Point2d& point, const Path& path);

};
