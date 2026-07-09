// drafting.h — D1 drafting aero model.
//
// Computes a per-rider CdA multiplier (RiderState::cda_factor) from formation
// geometry.  Pure and engine-free — same isolation philosophy as
// LateralSolver — so it is unit-testable on flat state (tests/test_drafting.cpp).
//
// Model (chain riders, i.e. every role except Body):
//   Each rider links to the *best* wheel — the one giving the largest draft
//   benefit — among the link_candidates longitudinally closest in-range
//   riders ahead (D3.0: nearest-wheel linking picked the wrong shelter and
//   stepped discontinuously when a near wheel drifted out of alignment).
//   A wheel is draftable when its gap < max_draft_gap and the rider's
//   lateral displacement from that leader's wake axis is
//   < lat_cutoff_radii * leader radius.  The wake axis trails along the
//   leader's apparent wind — straight behind in still air, tilted sideways
//   by crosswind (that tilt is what makes echelons work once B2 lands real
//   wind).
//
//   Known accepted discontinuity residuals (both tiny): the ~2% front-push
//   transfers between wheels with a small step at a link switch, and a
//   candidate-set change beyond the top-link_candidates can step (a rider
//   whose 3 nearest wheels are all offset gets nothing from a 4th, aligned
//   one).
//
//   cda_factor = 1 − (1 − table(depth)) · falloff(gap) · align(Δlat)
//
//   Chain depth is continuous: depth = 1 + s_leader · depth_leader, where
//   s_leader is the leader's own link strength (falloff·align).  A fading
//   upstream link therefore shifts everyone behind it smoothly toward the
//   shallower table entry — no CdA step when a chain splits, which at 100 Hz
//   would show up as speed jitter.  table(depth) interpolates linearly
//   between entries.
//
//   A rider with someone on the wheel additionally gets table[0]'s ~2% push,
//   scaled by that follower's link strength and weighted by (1 − own link
//   strength): full for a chain head, none for a tightly linked mid-chain
//   rider (the table already includes it), fading smoothly in between.
//
// Body-role riders use the coarse blob curve instead (see DraftingParams) —
// dormant today since nothing declares roles yet.

#ifndef DRAFTING_H
#define DRAFTING_H

#include "drafting_params.h"
#include "group.h"
#include "mytypes.h"
#include <vector>

// Flat per-rider input, built by the engine each tick from one-tick-stale
// positions (fine at 100 Hz).
struct DraftRiderState {
  RiderId id = 0;
  GroupId group_id = kNoGroup;
  GroupRole role = GroupRole::Unassigned;

  double lon_pos = 0.0;
  double lat_pos = 0.0;
  double speed = 0.0;
  double radius = 0.5;   // lateral half-extent, m
  double bike_len = 1.5; // longitudinal extent, m

  // Signed apparent-wind components in this rider's frame (m/s), same
  // projection as the longitudinal headwind in Rider::update().  Both are 0
  // with today's stub wind; crosswind tilts this rider's wake axis.
  double crosswind = 0.0;
  double headwind = 0.0;
};

// Fraction of draft benefit retained at wheel-to-wheel gap g (metres).
// Piecewise linear: 1.0 at g<=0, knee_retention at gap_knee, 0 at
// max_draft_gap and beyond.
double draft_gap_falloff(double gap, const DraftingParams& p);

// Lateral position of `leader`'s wake axis at longitudinal position lon_pos.
// The axis leaves the leader along its apparent wind: at distance d behind,
// it sits d · crosswind / airspeed to the side; zero crosswind → straight
// behind.  Shared definition: the shelter test uses it to score alignment,
// and the follow controller (D2) uses it as the follower's lat_target — so
// followers steer to exactly where the draft is.
double wake_axis_lat(const DraftRiderState& leader, double lon_pos);

// CdA multipliers, one per input rider (parallel to `riders`).
std::vector<double>
compute_draft_factors(const std::vector<DraftRiderState>& riders,
                      const DraftingParams& p);

#endif
