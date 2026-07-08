#ifndef DRAFTING_PARAMS_H
#define DRAFTING_PARAMS_H

// Tunables for the drafting aero model (drafting.h).  Dependency-free so it
// can be included anywhere without cycles — sibling of grouping_params.h /
// collision_params.h.
typedef struct DraftingParams {
  // CdA multiplier by chain depth (0 = front rider).  Depths beyond the last
  // entry clamp to it — a longer paceline shelters no deeper.  Entry 0 is the
  // front rider's small push from a follower on the wheel; it applies only
  // when someone is actually linked behind.
  double paceline_table[6] = {0.98, 0.61, 0.50, 0.44, 0.42, 0.41};

  // Piecewise-linear falloff of the draft benefit vs. wheel-to-wheel gap:
  // 1.0 at contact, knee_retention at gap_knee, 0.0 at max_draft_gap
  // (which is also the chain-link cutoff).
  double gap_knee = 5.0;       // m
  double knee_retention = 0.7; // benefit fraction left at gap_knee
  double max_draft_gap = 8.0;  // m

  // Benefit fades linearly to 0 as lateral displacement from the leader's
  // wake axis reaches lat_cutoff_radii * leader radius.
  double lat_cutoff_radii = 3.0;

  // Body-role blob heuristic (dormant until the decision layer declares
  // roles): CdA multiplier by number of same-group riders ahead within
  // body_window.  Placeholder values — tune when roles go live.
  double body_curve[4] = {0.90, 0.60, 0.50, 0.47};
  double body_window = 10.0; // m
} DraftingParams;

#endif
