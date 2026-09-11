#include <gtest/gtest.h>
#include <explore/boundary_goal.h>
#include <explore/frontier_search.h>
#include <explore/known_space.h>
#include <costmap_2d/cost_values.h>

TEST(BoundaryGoal, RingCenterCannotBeReturnedAsGoal) {
  geometry_msgs::Point robot;
  std::vector<geometry_msgs::Point> points;
  for (int i=0;i<24;++i) {
    geometry_msgs::Point p;
    p.x=.225*std::cos(i*2*M_PI/24);
    p.y=.225*std::sin(i*2*M_PI/24);
    points.push_back(p);
  }
  const auto result=explore::boundaryCandidates(points,robot,robot,0,.20);
  ASSERT_FALSE(result.empty());
  EXPECT_NEAR(result.front().x,.225,1e-9);
  EXPECT_NEAR(result.front().y,0,1e-9);
  for (const auto& p:result) EXPECT_GE(std::hypot(p.x,p.y),.20);
}

TEST(BoundaryGoal, TinyFrontierWaitsInsteadOfReportingArrival) {
  geometry_msgs::Point robot,p;
  p.x=.10;
  EXPECT_TRUE(explore::boundaryCandidates({p},robot,robot,0,.20).empty());
}

TEST(FrontierGeometry, SeedIsIncludedAtTranslatedOrigin) {
  costmap_2d::Costmap2D map(21,21,.05,10,20,costmap_2d::NO_INFORMATION);
  for (unsigned y=8;y<=12;++y)
    for (unsigned x=8;x<=12;++x) map.setCost(x,y,costmap_2d::FREE_SPACE);
  geometry_msgs::Point robot;
  map.mapToWorld(10,10,robot.x,robot.y);
  frontier_exploration::FrontierSearch search(&map,.001,1,.5);
  const auto found=search.searchFrom(robot);
  ASSERT_EQ(found.size(),1u);
  EXPECT_EQ(found.front().size,found.front().points.size());
  EXPECT_NEAR(found.front().centroid.x,robot.x,1e-9);
  EXPECT_NEAR(found.front().centroid.y,robot.y,1e-9);
}

TEST(KnownApproach, ChassisMustFitInsideObservedArea) {
  costmap_2d::Costmap2D map(60,60,.05,-1.5,-1.5,costmap_2d::NO_INFORMATION);
  for (unsigned y=10;y<50;++y) for (unsigned x=10;x<50;++x)
    map.setCost(x,y,costmap_2d::FREE_SPACE);
  EXPECT_TRUE(explore::knownFootprint(map,0,0,0,.43));
  EXPECT_FALSE(explore::knownFootprint(map,.9,0,0,.43));
  geometry_msgs::Point p;
  map.mapToWorld(50,30,p.x,p.y);
  const auto goals=explore::knownApproaches(map,{p});
  ASSERT_FALSE(goals.empty());
  for (const auto& g:goals) {
    unsigned x,y;
    ASSERT_TRUE(map.worldToMap(g.x,g.y,x,y));
    EXPECT_LT(map.getCost(x,y),costmap_2d::INSCRIBED_INFLATED_OBSTACLE);
  }
}

TEST(KnownApproach, InteriorObstacleAndUnknownAreRejectedButSoftCostsPreserved) {
  costmap_2d::Costmap2D map(60,60,.05,-1.5,-1.5,100);
  EXPECT_TRUE(explore::knownFootprint(map,0,0,0));
  map.setCost(31,31,costmap_2d::LETHAL_OBSTACLE);
  EXPECT_FALSE(explore::knownFootprint(map,0,0,0));
  map.setCost(31,31,costmap_2d::NO_INFORMATION);
  EXPECT_FALSE(explore::knownFootprint(map,0,0,0));
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc,argv);
  return RUN_ALL_TESTS();
}
