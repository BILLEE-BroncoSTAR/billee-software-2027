#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <vector>

#include "zed_openpose/skeleton_3d.hpp"

using zed_openpose::Xyz;

namespace
{

constexpr int kWidth = 100;
constexpr int kHeight = 100;

// A fake ZED XYZ measure: pixel (i, j) -> (depth, i / 100, j / 100), NaN where `valid` says so.
template<typename Valid>
auto measure(float depth, Valid valid)
{
  return [depth, valid](int i, int j) {
           if (i < 0 || j < 0 || i >= kWidth || j >= kHeight || !valid(i, j)) {
             return zed_openpose::nan_xyz();
           }
           return Xyz{depth, i / 100.f, j / 100.f};
         };
}

std::vector<std::array<float, 3>> body_25_seen_at(int i, int j, float score)
{
  return std::vector<std::array<float, 3>>(25, {static_cast<float>(i), static_cast<float>(j), score});
}

}  // namespace

TEST(PatchXyz, UsesThePixelWhenItHasDepth)
{
  const Xyz p = zed_openpose::patch_xyz(10, 20, measure(2.f, [](int, int) {return true;}));
  EXPECT_FLOAT_EQ(p[1], 0.10f);
  EXPECT_FLOAT_EQ(p[2], 0.20f);
}

TEST(PatchXyz, SearchesOutwardForTheNearestValidDepth)
{
  // Only column 13 has depth: 3 px right of the keypoint, inside R_max = 10.
  const Xyz p = zed_openpose::patch_xyz(10, 20, measure(2.f, [](int i, int) {return i == 13;}));
  EXPECT_FLOAT_EQ(p[1], 0.13f);
}

TEST(PatchXyz, GivesUpBeyondTheMaximumRadius)
{
  const Xyz p = zed_openpose::patch_xyz(10, 20, measure(2.f, [](int i, int) {return i == 25;}));
  EXPECT_TRUE(std::isnan(p[2]));
}

TEST(LiftPerson, LowScoresAreUnknownAndTheCentroidIsTheMeanOfTheRest)
{
  auto keypoints = body_25_seen_at(50, 50, 0.9f);
  keypoints[4] = {10.f, 10.f, 0.05f};  // right wrist below the threshold
  const auto person =
    zed_openpose::lift_person(keypoints, 0.12f, measure(3.f, [](int, int) {return true;}));
  ASSERT_EQ(person.joints.size(), 25u);
  EXPECT_TRUE(std::isnan(person.joints[4][2]));
  EXPECT_FLOAT_EQ(person.centroid[0], 3.f);
  EXPECT_FLOAT_EQ(person.centroid[1], 0.5f);
}

TEST(LiftPerson, DropsLimbsLongerThanAMetre)
{
  // Every joint at the same pixel except the right wrist, which lands 1.5 m deeper.
  auto keypoints = body_25_seen_at(50, 50, 0.9f);
  keypoints[4] = {80.f, 50.f, 0.9f};
  const auto person = zed_openpose::lift_person(
    keypoints, 0.12f, [](int i, int j) {
      return Xyz{i == 80 ? 4.5f : 3.f, i / 100.f, j / 100.f};
    });
  for (const auto & [a, b] : person.limbs) {
    EXPECT_FALSE(a == 4 || b == 4) << "limb " << a << "-" << b << " kept";
  }
  EXPECT_FALSE(person.limbs.empty());
}
