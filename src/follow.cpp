#include "follow.h"

double follow_effort(const FollowInput& in, double dt, double& integrator,
                     const FollowParams& p) {
  const double e = in.gap - (p.d0 + p.h * in.own_speed);

  integrator += p.ki * e * dt;
  if (integrator < 0.0)
    integrator = 0.0;
  else if (integrator > in.max_effort)
    integrator = in.max_effort;

  double u = p.kp * e + integrator + p.kd * in.rel_speed;
  if (u < 0.0)
    u = 0.0;
  else if (u > in.max_effort)
    u = in.max_effort;
  return u;
}

double protect_effort(const FollowInput& in, double dt, double& integrator,
                      const FollowParams& p) {
  // Mirrored error: positive when the ward is closing in (gap under the
  // setpoint) — push; negative when the ward is dropping — ease.
  const double e = (p.d0 + p.h * in.own_speed) - in.gap;

  integrator += p.protect_ki * e * dt;
  if (integrator < 0.0)
    integrator = 0.0;
  else if (integrator > in.max_effort)
    integrator = in.max_effort;

  // rel_speed = ward - own: a faster ward is about to shrink the gap, so the
  // term leads the position error — this is the speed-matching feedforward.
  double u = p.protect_kp * e + integrator + p.protect_kd * in.rel_speed;
  if (u < 0.0)
    u = 0.0;
  else if (u > in.max_effort)
    u = in.max_effort;
  return u;
}

double drift_effort(double v_err, double dt, double& integrator,
                    double max_effort, const FollowParams& p) {
  integrator += p.drift_ki * v_err * dt;
  if (integrator < 0.0)
    integrator = 0.0;
  else if (integrator > max_effort)
    integrator = max_effort;

  double u = p.drift_kp * v_err + integrator;
  if (u < 0.0)
    u = 0.0;
  else if (u > max_effort)
    u = max_effort;
  return u;
}
