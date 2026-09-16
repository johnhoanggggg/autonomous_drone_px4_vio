#include <cmath>
#include <gtest/gtest.h>

#include "px4_vio_bridge/localization_pair.hpp"
#include "px4_vio_bridge/path_geometry.hpp"
#include "px4_vio_bridge/command_limiter.hpp"

using px4_vio_bridge::LocalizationPairBuffer;
using px4_vio_bridge::Point2;
using px4_vio_bridge::map_displacement_to_vio;
using px4_vio_bridge::vio_point_to_map;

TEST(LocalizationPair, EitherDeliveryOrderCommitsOnlyACompleteSourceSample)
{
  for (const bool correction_first : {false, true}) {
    LocalizationPairBuffer pairs;
    const LocalizationPairBuffer::Pose pose{1.0, 2.0, 0.3};
    const LocalizationPairBuffer::Correction correction{0.15, -0.02, 0.0, -0.04};
    std::optional<LocalizationPairBuffer::Pair> result;
    if (correction_first) {
      EXPECT_FALSE(pairs.correction(10, correction, 1.0));
      result = pairs.pose(10, pose, 1.03);
    } else {
      EXPECT_FALSE(pairs.pose(10, pose, 1.0));
      result = pairs.correction(10, correction, 1.03);
    }
    ASSERT_TRUE(result);
    EXPECT_EQ(result->pose, pose);
    EXPECT_EQ(result->correction, correction);
    EXPECT_EQ(result->stamp, 10);
    EXPECT_DOUBLE_EQ(result->pose_received, correction_first ? 1.03 : 1.0);
    EXPECT_DOUBLE_EQ(result->correction_received, correction_first ? 1.0 : 1.03);
    EXPECT_FALSE(pairs.pending());
  }
}

TEST(LocalizationPair, InterleavedReceiptsNeverMixSamplesOrRegress)
{
  LocalizationPairBuffer pairs;
  EXPECT_FALSE(pairs.pose(10, {1, 0, 0}, 1));
  EXPECT_FALSE(pairs.pose(20, {2, 0, 0}, 2));
  auto first = pairs.correction(10, {0.1, 0, 0, 0}, 3);
  ASSERT_TRUE(first);
  EXPECT_DOUBLE_EQ(first->pose[0], 1);
  EXPECT_DOUBLE_EQ(first->pose_received, 1);
  EXPECT_TRUE(pairs.pending());
  auto second = pairs.correction(20, {0.2, 0, 0, 0}, 4);
  ASSERT_TRUE(second);
  EXPECT_DOUBLE_EQ(second->pose[0], 2);
  EXPECT_FALSE(pairs.pose(10, {9, 0, 0}, 5));
  EXPECT_FALSE(pairs.correction(10, {9, 0, 0, 0}, 5));
  EXPECT_FALSE(pairs.pending());
}

TEST(LocalizationPair, DuplicateTrafficCannotRefreshTheOriginalReceiptAge)
{
  LocalizationPairBuffer pairs;
  EXPECT_FALSE(pairs.pose(10, {1, 0, 0}, 1));
  EXPECT_FALSE(pairs.pose(10, {1, 0, 0}, 100));
  const auto result = pairs.correction(10, {0, 0, 0, 0}, 101);
  ASSERT_TRUE(result);
  EXPECT_DOUBLE_EQ(result->pose_received, 1);
  EXPECT_FALSE(pairs.correction(10, {0, 0, 0, 0}, 102));
  EXPECT_FALSE(pairs.pose(10, {1, 0, 0}, 102));
}

TEST(LocalizationPair, MissingCounterpartCannotGrowMemoryWithoutBound)
{
  LocalizationPairBuffer pairs;
  for (int i = 1; i <= 100; ++i) {
    EXPECT_FALSE(pairs.pose(i, {1, 0, 0}, i));
  }
  EXPECT_FALSE(pairs.correction(1, {0, 0, 0, 0}, 101));
  EXPECT_TRUE(pairs.correction(100, {0, 0, 0, 0}, 102));
  EXPECT_FALSE(pairs.pending());
}

TEST(LocalizationPair, RejectionCannotBeUndoneByAnOlderPendingPair)
{
  LocalizationPairBuffer pairs;
  EXPECT_FALSE(pairs.pose(10, {1, 0, 0}, 1));
  pairs.discard_through(20);
  EXPECT_FALSE(pairs.correction(10, {0, 0, 0, 0}, 2));
  EXPECT_FALSE(pairs.pose(20, {1, 0, 0}, 2));
  EXPECT_FALSE(pairs.correction(20, {0, 0, 0, 0}, 2));
  EXPECT_FALSE(pairs.pose(21, {1, 0, 0}, 3));
  EXPECT_TRUE(pairs.correction(21, {0, 0, 0, 0}, 3));
}

TEST(LocalizationPair, ZeroOrNegativeSourceStampsNeverEstablishLocalization)
{
  LocalizationPairBuffer pairs;
  for (const auto stamp : {-1, 0}) {
    EXPECT_FALSE(pairs.pose(stamp, {1, 0, 0}, 1));
    EXPECT_FALSE(pairs.correction(stamp, {0, 0, 0, 0}, 1));
  }
  EXPECT_FALSE(pairs.pending());
}

TEST(LocalizationPair, PathProvenanceRequiresTheExactPairAndSurvivesNewerSamples)
{
  LocalizationPairBuffer pairs;
  const LocalizationPairBuffer::Correction old_c{0, 0, 0, 0};
  const LocalizationPairBuffer::Correction new_c{0.15, 0, 0, 0.04};
  EXPECT_FALSE(pairs.at(10));  // A path arrived before its localization.
  pairs.correction(10, old_c, 1);
  EXPECT_FALSE(pairs.at(10));  // Still cannot use an unpaired correction.
  ASSERT_TRUE(pairs.pose(10, {1, 0, 0}, 1.02));
  pairs.correction(20, new_c, 2);
  ASSERT_TRUE(pairs.pose(20, {1.15, 0, 0}, 2.02));
  ASSERT_TRUE(pairs.at(10));  // A delayed old path retains its OLD transform.
  EXPECT_EQ(pairs.at(10)->correction, old_c);
  EXPECT_EQ(pairs.at(20)->correction, new_c);
  EXPECT_FALSE(pairs.at(15));  // No nearest-stamp guessing.
  pairs.reject(20);
  EXPECT_FALSE(pairs.at(20));  // Rejected duplicate cannot supply provenance.
  EXPECT_TRUE(pairs.at(10));
}

TEST(LocalizationPair, ReframedStoredPathDoesNotReinstallOldMapGeometry)
{
  using namespace px4_vio_bridge;
  const Correction4 old_c{0, 0, 0, 0}, new_c{0.15, -0.03, 0, 0.05};
  const std::vector<Point2> source_path{{0, 0}, {2, 0}};
  PathCommandLimiter limiter(.2, .3, .05, .05, .3, .2, .01, true, .05, true);
  ASSERT_TRUE(limiter.set_path(source_path, {0, 0}));
  for (int i = 0; i < 10; ++i) {limiter.update({1, 0}, .05, true, Point2{0, 0});}
  const auto before = *limiter.position();
  limiter.reexpress(old_c, new_c);
  std::vector<Point2> current_path;
  for (const auto & point : source_path) {
    current_path.push_back(reexpress_point(point, old_c, new_c));
  }
  EXPECT_FALSE(limiter.set_path(current_path, reexpress_point({0, 0}, old_c, new_c)));
  const auto after = reexpress_point(*limiter.position(), new_c, old_c);
  EXPECT_NEAR(after.first, before.first, 1e-12);
  EXPECT_NEAR(after.second, before.second, 1e-12);
}

TEST(LocalizationPair, CorrectionBetweenTicksCannotCreateALateralPositionPulse)
{
  // September 16 mechanism: new transform, timer tick, then matching body pose.
  // Keep both physical pose and physical command fixed while the map changes.
  const Point2 body_vio{1.34, -0.64}, command_vio{1.25, -0.8};
  const LocalizationPairBuffer::Correction old_c{-0.01, -0.092, 0, 0.0};
  const LocalizationPairBuffer::Correction new_c{0.17, -0.023, 0, -0.04628};
  const auto old_pose = vio_point_to_map(body_vio, old_c);
  const auto new_pose = vio_point_to_map(body_vio, new_c);
  const auto displacement = [&](const auto & pose, const auto & correction) {
      const auto command = vio_point_to_map(command_vio, correction);
      return map_displacement_to_vio(
        {command.first - pose[0], command.second - pose[1]}, correction[3]);
    };
  const LocalizationPairBuffer::Pose old_p{old_pose.first, old_pose.second, 0.3};
  const LocalizationPairBuffer::Pose new_p{new_pose.first, new_pose.second, 0.3};
  const auto baseline = displacement(old_p, old_c);
  // Demonstrate that the old independent-callback combination really fails.
  const auto mixed = displacement(old_p, new_c);
  EXPECT_GT(std::hypot(mixed.first-baseline.first, mixed.second-baseline.second), 0.14);
  for (const bool correction_first : {false, true}) {
    LocalizationPairBuffer pairs;
    pairs.pose(1, old_p, 0);
    auto active = pairs.correction(1, old_c, 0);
    ASSERT_TRUE(active);
    if (correction_first) {EXPECT_FALSE(pairs.correction(2, new_c, .05));}
    else {EXPECT_FALSE(pairs.pose(2, new_p, .05));}
    const auto during = displacement(active->pose, active->correction);
    EXPECT_NEAR(during.first, baseline.first, 1e-12);
    EXPECT_NEAR(during.second, baseline.second, 1e-12);
    auto next = correction_first ? pairs.pose(2, new_p, .08) :
      pairs.correction(2, new_c, .08);
    ASSERT_TRUE(next);
    const auto after = displacement(next->pose, next->correction);
    EXPECT_NEAR(after.first, baseline.first, 1e-12);
    EXPECT_NEAR(after.second, baseline.second, 1e-12);
  }
}
