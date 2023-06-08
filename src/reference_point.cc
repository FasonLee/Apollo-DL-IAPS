/******************************************************************************
 * Copyright 2017 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

/**
 * @file reference_point.cc
 **/

#include "include/reference_point.h"

namespace apollo {
namespace planning {

namespace {
// Minimum distance to remove duplicated points.
const double kDuplicatedPointsEpsilon = 1e-7;
}  // namespace

ReferencePoint::ReferencePoint(const double x, const double y) : x_(x), y_(y) {}

ReferencePoint::ReferencePoint(double x, double y, double heading, 
                  double kappa, double dkappa)
                  : x_(x), y_(y), heading_(heading), kappa_(kappa), dkappa_(dkappa) {}

double ReferencePoint::x() const { return x_; }
double ReferencePoint::y() const { return y_; }
double ReferencePoint::heading() const { return heading_; }
double ReferencePoint::kappa() const { return kappa_; }
double ReferencePoint::dkappa() const { return dkappa_; }

void ReferencePoint::set_x(double x) { x_ = x; }
void ReferencePoint::set_y(double y) { y_ = y; }
void ReferencePoint::set_heading(double heading) { heading_ = heading; }
void ReferencePoint::set_kappa(double kappa) { kappa_ = kappa; }
void ReferencePoint::set_dkappa(double dkappa) { dkappa_ = dkappa; }


void ReferencePoint::RemoveDuplicates(std::vector<ReferencePoint>* points) {
  int count = 0;
  const double limit = kDuplicatedPointsEpsilon * kDuplicatedPointsEpsilon;
  for (size_t i = 0; i < points->size(); ++i) {
    if (count == 0 ||
        (*points)[i].DistanceSquareTo((*points)[count - 1]) > limit) {
      (*points)[count++] = (*points)[i];
    } else {
      (*points).insert((*points).begin() + count, (*points)[i]);
    }
  }
  points->resize(count);
}

double ReferencePoint::DistanceSquareTo(const ReferencePoint &other) const {
  const double dx = x_ - other.x_;
  const double dy = y_ - other.y_;
  return dx * dx + dy * dy;
}

}  // namespace planning
}  // namespace apollo
