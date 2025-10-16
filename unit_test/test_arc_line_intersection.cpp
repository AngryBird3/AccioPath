#include "arc_line_intersection.h"

#include "gtest/gtest.h"

class ArcLineIntersectionTest : public ::testing::Test
{
public:
    void SetUp() override
    {

    }

    void TearDown() override
    {
    }

    static double perpendicularDistanceTest(const LineSegment& segment, const Path& path)
    {
         return ArcLineIntersection::perpendicularDistance(segment, path);
    }

};

TEST_F(ArcLineIntersectionTest, PerpendicularDistanceTest)
{
    LineSegment segment{{0, 0}, {10, 0}, 10};
    Path path{{5, 5}, 5, 0, 90, 0, 10};
    double dist = perpendicularDistanceTest(segment, path);
    EXPECT_NEAR(dist, 5.0, 1e-9);
}