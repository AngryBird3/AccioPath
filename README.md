# AccioPath
Determine if a path intersects with an obstacle

## Problem Statement
Given a set of obstacles and a path, determine whether the path intersects with any of the obstacles.

## Some highlight
* Discriminant method - Finds intersection points between a line and a circle
* Cross-product method - Determines whether a point is inside a polygon

## Approach in Details
1. First apprached it with how to find intersection of a line and a circle.
* I had solved whether a point lies on the circle or not in my previous project.
* I used the similar intuation to find intersection of a line and a circle.
* But this time I came across discriminant method. I knew about it but never used it. Learnt in school but of course forgot it.
* Then, I put gate1 - hey whether point is within arc angle or not. I did stumbled upon how Y axis theta is calculated, and
* why should I normalize by just drawing few things.
* Then, I put gate2 - hey whether height of the intersection point is within segment window or not. Since interpolation
* was mentioned in the problem statement, I ended up using that method. I had to study up a bit on interpolation.
* Lastly, to put together, how to use what I built to convex polygon. I used this mental model
* In order for arc (path) to intersect with convex polygon (edge), one of these two things needs to happen:
* 1. Arc intersects with one or more edges of the polygon.
* 2. The whole path is inside the polygon. Right? Cause if only part is inside then it must intersect with one of the edges.
* So, (1) is easy. I loop through all the edges and check intersection with arc.
* (2) I had to check whether my start and end point of my arc (path) is inside the polygon or not.
* For that, I had to look up hey how do I check whether a point is inside a polygon or not. This required cross-product
* and right-hand rule. That was fun to learn. I had to draw few things to understand it. I used (0,0), (0, 2), (3, 0) triangle
* as convex poDetermine if a path intersects with an obstacleilygon and checked whether (1,1) and point(2, 2) is inside or not. 