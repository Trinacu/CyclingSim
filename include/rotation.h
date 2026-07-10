// rotation.h — D3 paceline rotation coordinator.
//
// Owns an explicit ordered roster (a membership contract — deliberately
// separate from GroupTracker's emergent proximity classification) and a
// per-member state machine: Rotators cycle Pulling -> Drifting -> InLine;
// SittingIn members never pull and ride the rear.  Pure and engine-free,
// same isolation philosophy as drafting/follow: the engine feeds flat
// per-member inputs each tick and applies the returned directives to the
// follow subsystem.
//
// The follow graph is *resolved dynamically every tick*, not rewired on
// events:
//   - puller (front of InLine order): no follow target — its own effort
//     source is live (each rider decides its own pace; on promotion the new
//     puller inherits the previous puller's target_effort).
//   - InLine rotator: follows the previous rider in InLine order.
//   - Drifting rotator: follows the *last InLine rider* (dynamic).
//   - first SittingIn rider: follows the last InLine rider (same rule — this
//     is what makes the merge slot-open emerge); the rest chain behind each
//     other.
//
// Attach criterion: a drifter whose *position* drops below the last InLine
// rider's is appended to InLine order (one-shot and monotone — no timer or
// hysteresis).  "InLine" is bookkeeping; the lateral swing-offset fade in
// the follow subsystem finishes the merge physically.

#ifndef ROTATION_H
#define ROTATION_H

#include "mytypes.h"
#include "rotation_params.h"
#include <optional>
#include <vector>

struct RotationMember {
  RiderId id = -1;
  bool sits_in = false;
};

// Flat per-member input, built by the engine from one-tick-stale state.
struct RotationInput {
  RiderId id = -1;
  double lon_pos = 0.0;
  double speed = 0.0;
  double bike_len = 1.5;
  double crosswind = 0.0;     // apparent-wind component, picks the swing side
  double target_effort = 0.0; // inherited by the new puller on promotion
};

// One member's marching orders for this tick.
struct RotationDirective {
  RiderId id = -1;
  bool pulling = false;
  std::optional<double> set_effort; // present on the promotion tick only
  RiderId follow = -1;              // valid when !pulling
  double swing_side = 0.0;          // +1/-1 on the swing-off tick only
  double move_up_side = 0.0;        // +1/-1 every tick while in move-up
                                    // transit (sitter promotion, C-pre-b);
                                    // the engine applies the advance-side
                                    // offset + effort cap via the follow
                                    // subsystem
};

class PacelineRotation {
public:
  PacelineRotation(const std::vector<RotationMember>& roster,
                   const RotationParams& params);

  // Advance the state machine by dt and return directives for every current
  // member.  Members absent from `in` (rider removed from the engine) are
  // silently dropped from the roster.
  std::vector<RotationDirective> tick(double dt,
                                      const std::vector<RotationInput>& in);

  // Members removed by the detach rule this tick — the engine clears their
  // follow targets (they revert to plain riders).
  const std::vector<RiderId>& removed_last_tick() const { return removed_; }

  // SittingIn -> InLine promotion (C-pre-b).  The first sitter is already on
  // the last InLine wheel, so it moves to the InLine tail immediately (pure
  // bookkeeping).  A deeper sitter enters move-up transit: it rides up the
  // advance side past the sitters ahead (directive: follow the tail +
  // move_up_side) and is appended to InLine once it has passed the first
  // sitter (or, with no sitters left, closed to within engage_gap of the
  // tail) — the follow subsystem finishes the cut-in (offset fade) on its
  // own, same decoupling as the drifter merge.
  // False when the rider is not a sitter or there is no line to join.
  bool promote_sitter(RiderId id);

  // Introspection (tests / debug UI).
  RiderId puller() const { return inline_.empty() ? -1 : inline_.front(); }
  int inline_count() const { return static_cast<int>(inline_.size()); }
  int drifting_count() const { return static_cast<int>(drifting_.size()); }
  int promoting_count() const { return static_cast<int>(promoting_.size()); }
  int member_count() const {
    return static_cast<int>(inline_.size() + drifting_.size() +
                            sitting_.size() + promoting_.size());
  }
  bool is_member(RiderId id) const;

private:
  RotationParams params_;

  std::vector<RiderId> inline_;   // rotators in line, front (puller) first
  std::vector<RiderId> drifting_; // rotators merging back, oldest swing first
  std::vector<RiderId> sitting_;  // SittingIn members, front first
  std::vector<RiderId> promoting_; // sitters in move-up transit toward the
                                   // InLine tail (C-pre-b)

  // Per-member time spent detached (gap to the rider ahead > detach_gap).
  std::vector<std::pair<RiderId, double>> detach_timers_;
  double& detach_timer(RiderId id);

  double pull_timer_ = 0.0;
  std::vector<RiderId> removed_;
};

#endif
