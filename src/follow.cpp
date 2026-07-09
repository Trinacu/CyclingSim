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
