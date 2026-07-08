#include "drafting.h"
#include <algorithm>
#include <cmath>
#include <numeric>

namespace {

// Keeps the wake-axis slope finite when a leader is near-stationary.
constexpr double kMinAirSpeed = 0.1;

// Lateral displacement of `follower` from `leader`'s wake axis.  The axis
// leaves the leader along its apparent wind: at lon distance d behind, it
// sits d · crosswind / airspeed to the side.  Zero crosswind → straight
// behind.
double wake_offset(const DraftRiderState& leader,
                   const DraftRiderState& follower) {
  const double lon_sep = leader.lon_pos - follower.lon_pos;
  const double air_speed =
      std::max(leader.speed + leader.headwind, kMinAirSpeed);
  const double axis_lat =
      leader.lat_pos + lon_sep * leader.crosswind / air_speed;
  return follower.lat_pos - axis_lat;
}

// Benefit fraction retained at lateral displacement `off` from the wake
// axis: 1 on-axis, linear to 0 at lat_cutoff_radii * leader radius.
double alignment(double off, double leader_radius, const DraftingParams& p) {
  const double cutoff = p.lat_cutoff_radii * leader_radius;
  if (cutoff <= 0.0)
    return 0.0;
  return std::max(0.0, 1.0 - std::fabs(off) / cutoff);
}

// paceline_table at continuous depth >= 1, linearly interpolated, clamped to
// the last entry.
double table_value(double depth, const DraftingParams& p) {
  constexpr int N =
      sizeof(DraftingParams{}.paceline_table) / sizeof(double);
  if (depth >= N - 1)
    return p.paceline_table[N - 1];
  const int lo = static_cast<int>(depth);
  const double frac = depth - lo;
  return p.paceline_table[lo] * (1.0 - frac) +
         p.paceline_table[lo + 1] * frac;
}

} // namespace

double draft_gap_falloff(double gap, const DraftingParams& p) {
  if (gap <= 0.0)
    return 1.0;
  if (gap < p.gap_knee)
    return 1.0 - (1.0 - p.knee_retention) * gap / p.gap_knee;
  if (gap < p.max_draft_gap)
    return p.knee_retention * (p.max_draft_gap - gap) /
           (p.max_draft_gap - p.gap_knee);
  return 0.0;
}

std::vector<double>
compute_draft_factors(const std::vector<DraftRiderState>& riders,
                      const DraftingParams& p) {
  const int n = static_cast<int>(riders.size());
  std::vector<double> factors(n, 1.0);
  if (n < 2)
    return factors;

  // Front-to-back processing order so a leader's depth and link strength are
  // resolved before its followers read them.
  std::vector<int> order(n);
  std::iota(order.begin(), order.end(), 0);
  std::sort(order.begin(), order.end(), [&riders](int a, int b) {
    return riders[a].lon_pos > riders[b].lon_pos;
  });

  std::vector<int> leader_of(n, -1);
  std::vector<double> link_s(n, 0.0); // own-link strength: falloff · align
  std::vector<double> depth(n, 0.0);  // continuous chain depth, 0 = head

  // Links are built for every rider regardless of role: a Body rider's wheel
  // is still air shelter, and its link strength propagates depth to riders
  // behind it.  Only the *factor* is role-gated below.
  for (int k = 0; k < n; ++k) {
    const int i = order[k];
    const DraftRiderState& me = riders[i];

    // Nearest strictly-ahead rider with a draftable wheel.  order[0..k) are
    // the riders at or ahead of me; N is small, so scan them all.
    int best = -1;
    double best_sep = 0.0;
    for (int k2 = 0; k2 < k; ++k2) {
      const int j = order[k2];
      const double lon_sep = riders[j].lon_pos - me.lon_pos;
      if (lon_sep <= 0.0)
        continue; // co-located: no link
      if (lon_sep - riders[j].bike_len >= p.max_draft_gap)
        continue;
      if (alignment(wake_offset(riders[j], me), riders[j].radius, p) <= 0.0)
        continue;
      if (best < 0 || lon_sep < best_sep) {
        best = j;
        best_sep = lon_sep;
      }
    }
    if (best < 0)
      continue;

    const double gap = best_sep - riders[best].bike_len;
    const double off = wake_offset(riders[best], me);
    leader_of[i] = best;
    link_s[i] =
        draft_gap_falloff(gap, p) * alignment(off, riders[best].radius, p);
    depth[i] = 1.0 + link_s[best] * depth[best];
  }

  constexpr int NB = sizeof(DraftingParams{}.body_curve) / sizeof(double);

  for (int i = 0; i < n; ++i) {
    if (riders[i].role == GroupRole::Body) {
      int n_ahead = 0;
      for (int j = 0; j < n; ++j) {
        if (j == i || riders[j].group_id != riders[i].group_id)
          continue;
        const double d = riders[j].lon_pos - riders[i].lon_pos;
        if (d > 0.0 && d <= p.body_window)
          ++n_ahead;
      }
      factors[i] = p.body_curve[std::min(n_ahead, NB - 1)];
      continue;
    }

    const double s = link_s[i];
    const double benefit_ahead =
        (leader_of[i] >= 0) ? (1.0 - table_value(depth[i], p)) * s : 0.0;

    // table[0]'s ~2% push from a follower on the wheel, scaled by the
    // strongest link any follower has on this rider (max over followers is
    // continuous where "nearest follower" would jump when the closest wheel
    // changes).  Weighted by (1 - s): a tightly linked mid-chain rider gets
    // none — the TTT table already includes it — but it fades in smoothly as
    // the rider's own link ahead fades, so a chain splitting doesn't step
    // the new head's CdA by the push amount.
    double follower_s = 0.0;
    for (int f = 0; f < n; ++f) {
      if (leader_of[f] == i)
        follower_s = std::max(follower_s, link_s[f]);
    }
    const double benefit_behind =
        (1.0 - p.paceline_table[0]) * follower_s * (1.0 - s);

    factors[i] = 1.0 - benefit_ahead - benefit_behind;
  }

  return factors;
}
