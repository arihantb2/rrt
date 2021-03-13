#include <gtest/gtest.h>
#include <obstacle.h>

class CircleObstacleTest : public ::testing::Test
{
protected:
    void SetUp() {}
    void TearDown() {}

public:
    const world::Circle circle = world::Circle(world::Vector2(0.0, 0.0), 5.0);
    const world::Vector2 point1 = world::Vector2(0.0, 0.0);
    const world::Vector2 point2 = world::Vector2(10.0, 10.0);
    const world::Vector2 point3 = world::Vector2(10.0, -10.0);
    const world::Vector2 point4 = world::Vector2(-10.0, 10.0);
    const world::Vector2 point5 = world::Vector2(-10.0, -10.0);
    const world::Vector2 point6 = world::Vector2(5.0, 5.0);
    const world::Vector2 point7 = world::Vector2(5.0, -5.0);
    const world::Vector2 point8 = world::Vector2(-5.0, 5.0);
    const world::Vector2 point9 = world::Vector2(-5.0, -5.0);
};

TEST_F(CircleObstacleTest, NoCollision)
{
    ASSERT_FALSE(circle.isColliding(point2, point3));
    ASSERT_FALSE(circle.isColliding(point4, point5));
    ASSERT_FALSE(circle.isColliding(point2, point4));
    ASSERT_FALSE(circle.isColliding(point3, point5));
}

TEST_F(CircleObstacleTest, Collision)
{
    ASSERT_TRUE(circle.isColliding(point1, point2));
    ASSERT_TRUE(circle.isColliding(point1, point3));
    ASSERT_TRUE(circle.isColliding(point1, point4));
    ASSERT_TRUE(circle.isColliding(point1, point5));
    ASSERT_TRUE(circle.isColliding(point2, point5));
    ASSERT_TRUE(circle.isColliding(point3, point4));
}

int main(int argc, char *argv[])
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
