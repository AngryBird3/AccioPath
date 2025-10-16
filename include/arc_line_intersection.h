#pragma once

#include "types.h"

class ArcLineIntersection
{
public:
    /**
     * @brief Returns true if the line segment intersects the circular arc path
     * @param segment
     * @param path
     * @return
     */
    static bool Intersect(const Obstacle& segment,
                          const Path& path);
private:
    using StartPointOfArc = Point2d;
    using EndPointOfArc = Point2d;
    using StartAndEndPointsOfArc = std::pair<StartPointOfArc, EndPointOfArc>;
    /**
     * @brief Calculate the perpendicular distance from a line segment to a circular arc path.
     * @param segment
     * @param path
     * @return
     */
    static double perpendicularDistance(const LineSegment& segment, const Path& path);

    /**
     * @brief Find the intersection point between a line segment and a circular arc path, if it exists.
     * @param segment
     * @param path
     * @return
     */
    static std::optional<Point2d> intersectionPoint(const LineSegment& segment, const Path& path);

    /**
     * @brief Check if a point (on arc) is within the angular span of the arc.
     * @details gate against 1) within arc angle 2) height at that point is below the height of the line segment
     * @param point point (on arc) to check
     * @param segmentHeight height the line segment
     * @param path arc
     * @return interpolated height at that point if point is within arc angle, std::nullopt otherwise
     */
    static bool isWithinArcAngleAndHeight(const Point2d& point, double segmentHeight, const Path& path);

    /**
     * @brief Returns true if the point is inside the polygon.
     * @param point the point to check
     * @param polygon obstacle (convex polygon)
     * @return
     */
    static bool isPointInPolygon(const Point2d& point, const Obstacle& polygon);

    /**
     * @brief Returns the start and end points of the arc defined by the path.
     * @param path the arc path
     * @return
     */
    static StartAndEndPointsOfArc getStartAndEndPointsOfPath(const Path& path);

    /**
     * @brief Returns true if the path intersects the line segment.
     * @param point
     * @param segment
     * @return
     */
    static bool isPathIntersectingSegment(const LineSegment& segment, const Path& path);

};
