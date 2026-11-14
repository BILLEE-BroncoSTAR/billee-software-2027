// 3D skeletons from OpenPose 2D keypoints + the ZED's XYZ measure.
//
// The logic of stereolabs/zed-openpose src/main.cpp (getPatchIdx, fill_people_ogl), unchanged:
// each keypoint takes the XYZ under its pixel, else the first valid XYZ in growing squares around
// it; a limb is kept only if it is shorter than MAX_DISTANCE_LIMB and its middle is within
// MAX_DISTANCE_CENTER of the person's centre of gravity. Free of ZED/OpenPose types so it can be
// unit-tested; the node passes a lookup into the ZED's sl::Mat.
//
// Copyright (c) 2019 Stereolabs, MIT License (LICENSE.zed-openpose).

#pragma once

#include <array>
#include <cmath>
#include <limits>
#include <string>
#include <utility>
#include <vector>

namespace zed_openpose
{

using Xyz = std::array<float, 3>;

constexpr int kPatchRadiusMax = 10;         // getPatchIdx: R_max
constexpr float kMaxDistanceLimb = 1.0f;    // MAX_DISTANCE_LIMB (m)
constexpr float kMaxDistanceCenter = 1.8f;  // MAX_DISTANCE_CENTER (m)

inline Xyz nan_xyz()
{
  const float nan = std::numeric_limits<float>::quiet_NaN();
  return {nan, nan, nan};
}

inline bool is_valid(const Xyz & p) {return std::isfinite(p[2]);}

// Limb pairs per model, as drawn by zed-openpose (fill_people_ogl's partsLink), by keypoint count.
inline const std::vector<int> & parts_link(int keypoint_count)
{
  // https://github.com/CMU-Perceptual-Computing-Lab/openpose/blob/master/doc/02_output.md
  static const std::vector<int> mpi = {
    0, 1, 1, 2, 2, 3, 3, 4, 1, 5, 5, 6, 6, 7, 1, 14, 14, 8, 8, 9, 9, 10, 14, 11, 11, 12, 12, 13};
  static const std::vector<int> coco = {
    2, 1, 1, 5, 8, 11, 1, 8, 11, 1, 8, 9, 9, 10, 11, 12, 12, 13, 2, 3, 3, 4, 5, 6, 6, 7,
    16, 1, 17, 1, 16, 17};
  static const std::vector<int> body_25 = {
    0, 1, 1, 2, 2, 3, 3, 4, 1, 5, 5, 6, 6, 7, 1, 8, 8, 9, 8, 12, 12, 13, 13, 14, 14, 19, 19, 20,
    14, 21, 8, 9, 9, 10, 10, 11, 11, 24, 11, 22, 22, 23, 0, 16, 0, 15, 15, 17, 16, 18};
  static const std::vector<int> none;
  switch (keypoint_count) {
    case 15: return mpi;
    case 18: return coco;
    case 25: return body_25;
    default: return none;
  }
}

// getPatchIdx: XYZ at (i, j), else the first valid one in squares of growing radius around it.
// `xyz(i, j)` returns the measure at column i, row j (NaN outside the image or where unknown).
template<typename Lookup>
Xyz patch_xyz(int center_i, int center_j, Lookup && xyz)
{
  for (int r = 0; r < kPatchRadiusMax; ++r) {
    for (int y = -r; y <= r; ++y) {
      for (int x = -r; x <= r; ++x) {
        const Xyz p = xyz(center_i + x, center_j + y);
        if (is_valid(p)) {
          return p;
        }
      }
    }
  }
  return nan_xyz();
}

struct Person3D
{
  std::vector<Xyz> joints;                  // per keypoint; NaN = unknown
  Xyz centroid;                             // centre of gravity of the known joints
  std::vector<std::pair<int, int>> limbs;   // keypoint pairs that passed the limb filters
};

// fill_people_ogl for one person. `keypoints` is that person's OpenPose row: (u, v, score) per part.
template<typename Lookup>
Person3D lift_person(
  const std::vector<std::array<float, 3>> & keypoints, float score_threshold, Lookup && xyz)
{
  Person3D person;
  person.joints.assign(keypoints.size(), nan_xyz());
  Xyz sum{0.f, 0.f, 0.f};
  int count = 0;
  for (size_t k = 0; k < keypoints.size(); ++k) {
    const auto & [u, v, score] = keypoints[k];
    if (score < score_threshold) {
      continue;
    }
    const int i = static_cast<int>(std::round(u));
    const int j = static_cast<int>(std::round(v));
    Xyz p = xyz(i, j);
    if (!is_valid(p)) {
      p = patch_xyz(i, j, xyz);
    }
    person.joints[k] = p;
    if (is_valid(p)) {
      for (int a = 0; a < 3; ++a) {
        sum[a] += p[a];
      }
      ++count;
    }
  }
  person.centroid = nan_xyz();
  if (count > 0) {
    person.centroid = {sum[0] / count, sum[1] / count, sum[2] / count};
  }

  const auto & link = parts_link(static_cast<int>(keypoints.size()));
  for (size_t part = 0; part + 1 < link.size(); part += 2) {
    const Xyz & a = person.joints[link[part]];
    const Xyz & b = person.joints[link[part + 1]];
    const float length = std::hypot(a[0] - b[0], a[1] - b[1], a[2] - b[2]);
    const float to_center = std::hypot(
      (a[0] + b[0]) * 0.5f - person.centroid[0], (a[1] + b[1]) * 0.5f - person.centroid[1],
      (a[2] + b[2]) * 0.5f - person.centroid[2]);
    if (std::isfinite(to_center) && to_center < kMaxDistanceCenter && length < kMaxDistanceLimb) {
      person.limbs.emplace_back(link[part], link[part + 1]);
    }
  }
  return person;
}

}  // namespace zed_openpose
