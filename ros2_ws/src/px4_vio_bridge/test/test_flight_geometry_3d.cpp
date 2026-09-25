#include <gtest/gtest.h>
#include <octomap/OcTree.h>
#include "px4_vio_bridge/flight_geometry_3d.hpp"
namespace p=px4_vio_bridge;
TEST(Flight3D, ConvertsAllAxesThroughFullCorrectionAndYawOffset)
{
  const auto q=p::normalize({.2,-.3,.1,.9});
  const p::Point3 v{.4,-.5,.6};
  for (double yaw : {0.,.8,-1.2}) {
    const auto n=p::map_delta_to_ned(v,q,yaw),m=p::ned_delta_to_map(n,q,yaw);
    EXPECT_NEAR(p::distance_3d(v,m),0,1e-12);
  }
  const auto n=p::map_delta_to_ned(v,{},0);
  EXPECT_DOUBLE_EQ(n.x,-.5);EXPECT_DOUBLE_EQ(n.y,.4);EXPECT_DOUBLE_EQ(n.z,-.6);
}
TEST(Flight3D, ReframesMapPointsThroughTheirContinuousOdomCoordinates)
{
  const p::Point3 odom{.4,-.5,.6};
  const p::Pose3 old_frame{{1.,-2.,.3},p::normalize({.2,-.3,.1,.9})};
  const p::Pose3 new_frame{{-.7,.8,-.2},p::normalize({-.1,.25,.3,.91})};
  const auto old_map=p::odom_point_to_map_3d(odom,old_frame);
  const auto recovered=p::map_point_to_odom_3d(old_map,old_frame);
  EXPECT_NEAR(p::distance_3d(odom,recovered),0,1e-12);
  const auto reframed=p::reframe_map_point_3d(old_map,old_frame,new_frame);
  const auto expected=p::odom_point_to_map_3d(odom,new_frame);
  EXPECT_NEAR(p::distance_3d(reframed,expected),0,1e-12);
}
TEST(Flight3D, UnknownRequiresExplicitPolicyAndOccupiedStillBlocks)
{
  octomap::OcTree tree(.05);
  tree.updateNode(octomap::point3d(2,2,2),true);tree.updateInnerOccupancy();
  const p::Point3 a{0,0,0},b{.5,0,0};
  EXPECT_FALSE(p::flight_chord_clear(tree,a,b,.35,false));
  EXPECT_TRUE(p::flight_chord_clear(tree,a,b,.35,true));
  tree.updateNode(octomap::point3d(.25,.2,0),true);tree.updateInnerOccupancy();
  EXPECT_FALSE(p::flight_chord_clear(tree,a,b,.35,true));
  EXPECT_FALSE(p::flight_chord_clear(tree,{NAN,0,0},b,.35,true));
}
TEST(Flight3D, BoundsAreRelativeToTakeoffAndIncludeVerticalMotion)
{
  const p::Point3 origin{10,-5,3};
  EXPECT_TRUE(p::within_flight_bounds({10.5,-5,2.5},origin,2.65,.15,1.25));
  EXPECT_FALSE(p::within_flight_bounds({13,-5,2.5},origin,2.65,.15,1.25));
  EXPECT_FALSE(p::within_flight_bounds({10,-5,3},origin,2.65,.15,1.25));
  EXPECT_FALSE(p::within_flight_bounds({10,-5,1.5},origin,2.65,.15,1.25));
}
TEST(Flight3D, FinalNedLimiterSurvivesRebaseNoiseAndRouteReversal)
{
  p::Follower3DConfig c;c.stop_at_vertices=true;p::FlightPositionLimiter3D limiter(c);limiter.reset({10,-4,3});
  auto previous=limiter.position();p::Point3 velocity{};
  for (int i=0;i<1200;++i) {
    const double dt=i%2?.02:.035;
    const p::Point3 target=i<300?p::Point3{11,-3,2.5}:(i<600?
      p::Point3{previous.x+(i%2?.0001:-.0001),previous.y+.0001,previous.z-.0001}:p::Point3{9,-5,3});
    const auto next=limiter.update(target,dt),v=p::scale_3d(p::subtract_3d(next,previous),1/dt);
    const auto a=p::scale_3d(p::subtract_3d(v,velocity),1/dt);
    EXPECT_LE(std::hypot(v.x,v.y),c.max_horizontal_speed+1e-9);
    EXPECT_LE(std::abs(v.z),c.max_vertical_speed+1e-9);
    EXPECT_LE(std::hypot(a.x,a.y),c.max_horizontal_acceleration+1e-9);
    EXPECT_LE(std::abs(a.z),c.max_vertical_acceleration+1e-9);
    previous=next;velocity=v;
  }
}

TEST(Flight3D, SteepRouteKeepsDirectionAndBrakesAtCorners)
{
  p::Follower3DConfig cfg;cfg.stop_at_vertices=true;
  cfg.max_cross_track=.15;cfg.max_vertical_track=.15;
  const std::vector<p::Point3> route{{0,0,0},{.006,.001,.014},
    {.6,-2,1.1},{.8,-2.35,1.1},{1.4,-2.7,.2}};
  const p::Polyline3D polyline(route);
  const auto q=p::normalize({.0033,-.0007,.0141,.9999});
  const auto up=p::rotate(q,{0,0,1});
  p::RouteFollower3D follower(p::limits_for_map_tilt(cfg,up.z));
  p::FlightPositionLimiter3D limiter(cfg);limiter.reset({});
  p::Point3 body{},previous{},velocity{};
  ASSERT_TRUE(follower.set_path(route,body));
  bool reached=false;
  for(int i=0;i<15000;++i) {
    const auto result=follower.update(body,.02,[](const auto&,const auto&){return true;});
    ASSERT_TRUE(result.valid)<<result.reason;
    const auto current=p::map_delta_to_ned(body,q,0);
    const auto target=p::add_3d(current,p::map_delta_to_ned(result.displacement,q,0));
    const auto command=limiter.update(target,.02);
    body=p::ned_delta_to_map(command,q,0);
    const auto v=p::scale_3d(p::subtract_3d(command,previous),50);
    const auto a=p::scale_3d(p::subtract_3d(v,velocity),50);
    ASSERT_LE(polyline.project(body).distance,.005);
    ASSERT_LE(std::hypot(v.x,v.y),cfg.max_horizontal_speed+1e-8);
    ASSERT_LE(std::abs(v.z),cfg.max_vertical_speed+1e-8);
    ASSERT_LE(std::hypot(a.x,a.y),cfg.max_horizontal_acceleration+1e-8);
    ASSERT_LE(std::abs(a.z),cfg.max_vertical_acceleration+1e-8);
    previous=command;velocity=v;
    if(p::distance_3d(body,route.back())<.05){reached=true;break;}
  }
  EXPECT_TRUE(reached);
}

TEST(Flight3D, CornersDoNotRequireMillimetrePerfectVehicleTracking)
{
  p::Follower3DConfig cfg;cfg.stop_at_vertices=true;
  cfg.max_cross_track=.15;cfg.max_vertical_track=.15;
  const std::vector<p::Point3> route{{0,0,.5},{.5,0,.5},{.5,.5,.5}};
  p::RouteFollower3D follower(cfg);p::Point3 body=route.front();
  ASSERT_TRUE(follower.set_path(route,body));
  bool reached=false;
  for(int i=0;i<6000;++i) {
    const auto result=follower.update(body,.02,[](const auto&,const auto&){return true;});
    ASSERT_TRUE(result.valid)<<result.reason;
    // A persistent 1 cm position error must not strand the vehicle at a corner.
    body=p::add_3d(result.carrot,{.01,0,0});
    if(result.reached){reached=true;break;}
  }
  EXPECT_TRUE(reached);
}
