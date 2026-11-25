#include "planner_snapshot.h"
#include <math.h>

extern "C" bool PlannerSnapshot_Build(const PlannerSnapshotSource *s,
    const PlannerSnapshotLimits *limits, OneRunInput *output) {
  if (!output) return false;
  *output = {};
  if (!s || !limits || !limits->pose_age_ms || !limits->range_age_ms ||
      limits->pose_age_ms > 0x7fffffffU || limits->range_age_ms > 0x7fffffffU ||
      limits->frame_skew_ms > limits->range_age_ms ||
      !isfinite(s->x_cm) || !isfinite(s->y_cm) || !isfinite(s->hybrid_yaw_rad) ||
      !isfinite(s->speed_cm_s) || !isfinite(s->dt_s) || s->dt_s <= 0 || s->dt_s > .25f ||
      !s->pose_verified || !s->encoder_valid || !s->gyro_valid) return false;
  const uint32_t encoderAge = s->now_ms - s->encoder_ms;
  const uint32_t gyroAge = s->now_ms - s->gyro_ms;
  const uint32_t poseAge = encoderAge > gyroAge ? encoderAge : gyroAge;
  if (poseAge > limits->pose_age_ms) return false;
  uint32_t oldest = 0, newest = UINT32_MAX;
  for (unsigned i = 0; i < 4; ++i) {
    const auto &range = s->ranges[i];
    const uint32_t age = s->now_ms - range.at_ms;
    if (!range.valid || range.mm == 0 || age > limits->range_age_ms) return false;
    if (age > oldest) oldest = age;
    if (age < newest) newest = age;
    if (s->ranges_have_origins && (!isfinite(range.x_cm) ||
        !isfinite(range.y_cm) || !isfinite(range.hybrid_yaw_rad))) return false;
  }
  if (oldest - newest > limits->frame_skew_ms) return false;
  output->pose = {s->x_cm, s->y_cm,
    remainderf(1.57079632679f - remainderf(s->hybrid_yaw_rad, 6.28318530718f), 6.28318530718f)};
  output->measured_speed_cm_s = s->speed_cm_s;
  output->dt_s = s->dt_s;
  output->safety.now_ms = s->now_ms;
  output->safety.pose_sample_ms = s->now_ms - poseAge;
  output->safety.sensor_sample_ms = s->now_ms - oldest;
  output->safety.pose_valid = 1;
  output->safety.sensor_valid = 1;
  output->safety.control_deadline_ok = 1;
  output->safety.emergency_stop_requested = s->emergency_stop ? 1 : 0;
  output->safety.front_min_mm = s->ranges[1].mm < s->ranges[2].mm ? s->ranges[1].mm : s->ranges[2].mm;
  for (unsigned i = 0; i < 4; ++i)
    output->ranges[i] = {s->ranges[i].mm, uint8_t(s->ranges[i].valid)};
  if (s->ranges_have_origins) {
    for (unsigned i = 0; i < 4; ++i) {
      const PlannerTimedRange &range = s->ranges[i];
      if (!isfinite(range.x_cm) || !isfinite(range.y_cm) ||
          !isfinite(range.hybrid_yaw_rad)) {
        return false;
      }
      output->range_poses[i] = {
        range.x_cm, range.y_cm,
        remainderf(1.57079632679f -
                   remainderf(range.hybrid_yaw_rad, 6.28318530718f),
                   6.28318530718f),
      };
    }
    output->ranges_ready = 1;
    output->range_poses_ready = 1;
  }
  // planner_ok is owned by mission, not the sensor snapshot.
  return true;
}
