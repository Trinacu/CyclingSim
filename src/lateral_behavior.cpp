// lateral_behavior.cpp
#include "lateral_behavior.h"
#include <algorithm>
#include <cmath>
#include <optional>

std::optional<double>
HoldLineBehavior::compute_lat_target(const LateralContext& ctx) const {
  return std::nullopt;
}

OvertakeBehavior::OvertakeBehavior(Side preferred_side, double rider_radius,
                                   double arrival_tol)
    : preferred_side_(preferred_side), rider_radius_(rider_radius),
      arrival_tol_(arrival_tol) {}

std::optional<double>
OvertakeBehavior::compute_lat_target(const LateralContext& ctx) const {

  // Collect riders ahead (positive lon_offset only)
  std::vector<const NearbyRider*> ahead;
  for (const auto& nr : ctx.nearby) {
    if (nr.lon_offset > 0.0)
      ahead.push_back(&nr);
  }

  if (ahead.empty())
    return std::nullopt; // clear road ahead — no steering target needed

  // Sort by lateral position to reason about gaps between adjacent riders
  std::sort(ahead.begin(), ahead.end(),
            [](const NearbyRider* a, const NearbyRider* b) {
              return a->lat_pos < b->lat_pos;
            });

  const double half_road = ctx.road_width / 2.0;
  const double min_gap = 2.0 * rider_radius_;

  struct Gap {
    double centre;
    double width;
  };
  std::vector<Gap> gaps;

  // Gap between left road edge and leftmost rider
  {
    double right = ahead.front()->lat_pos - rider_radius_;
    double left = -half_road;
    double w = right - left;
    if (w >= min_gap)
      gaps.push_back({(left + right) / 2.0, w});
  }

  // Gaps between adjacent ahead-riders
  for (size_t i = 0; i + 1 < ahead.size(); ++i) {
    double left = ahead[i]->lat_pos + rider_radius_;
    double right = ahead[i + 1]->lat_pos - rider_radius_;
    double w = right - left;
    if (w >= min_gap)
      gaps.push_back({(left + right) / 2.0, w});
  }

  // Gap between rightmost rider and right road edge
  {
    double left = ahead.back()->lat_pos + rider_radius_;
    double right = half_road;
    double w = right - left;
    if (w >= min_gap)
      gaps.push_back({(left + right) / 2.0, w});
  }

  if (gaps.empty())
    return std::nullopt; // fully blocked — shove model handles the contact

  // Score each gap: width is the primary metric; penalise gaps on the
  // non-preferred side (soft preference, not a hard exclusion)
  const Gap* best = nullptr;
  double best_score = -std::numeric_limits<double>::infinity();

  for (const auto& g : gaps) {
    double score = g.width;
    if (preferred_side_ == Side::Left && g.centre > 0.0)
      score *= 0.5;
    if (preferred_side_ == Side::Right && g.centre < 0.0)
      score *= 0.5;

    if (score > best_score) {
      best_score = score;
      best = &g;
    }
  }

  const double target = std::clamp(best->centre, -half_road, half_road);

  // If we've already arrived, yield back to the force-driven model.
  // The pass is established; active steering is no longer needed.
  if (std::fabs(ctx.own_lat_pos - target) < arrival_tol_)
    return std::nullopt;

  return target;
}

BlockBehavior::BlockBehavior(double fatigue_threshold)
    : fatigue_threshold_(fatigue_threshold) {}

std::optional<double>
BlockBehavior::compute_lat_target(const LateralContext& ctx) const {

  // Stop blocking when too fatigued — the solver's w_prime_frac weighting
  // already degrades blocking effectiveness continuously; this threshold
  // provides a clean cut-off where the rider concedes entirely.
  if (ctx.own_w_prime_frac < fatigue_threshold_)
    return std::nullopt;

  // Find the nearest chasing rider (most negative lon_offset)
  const NearbyRider* chaser = nullptr;
  double closest_offset = 0.0; // must be strictly negative to qualify

  for (const auto& nr : ctx.nearby) {
    if (nr.lon_offset < closest_offset) {
      closest_offset = nr.lon_offset;
      chaser = &nr;
    }
  }

  if (!chaser)
    return std::nullopt; // nobody behind — nothing to block

  const double half_road = ctx.road_width / 2.0;
  return std::clamp(chaser->lat_pos, -half_road, half_road);
}
