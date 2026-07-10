#ifndef MYTYPES_H
#define MYTYPES_H

// Semantic, scenario-level identity (UI / plots / configs)
using RiderId = int;

using GroupId = int;

inline GroupId kNoGroup = -1;

using TeamId = int;

inline TeamId kNoTeam = -1;

// Which system owns a rider's target_effort.  Exactly one writer is active
// per rider at any time — the mode is the arbiter, there is no blending:
//   Follow   — the gap controller (a follow target is assigned); the effort
//              slider, any schedule and any policy effort are inert.  A
//              policy may itself have installed the follow target — it is a
//              meta-controller selecting modes, not a competing writer.
//   Schedule — an EffortSchedule drives effort each tick; the slider is inert.
//   Policy   — an IRiderPolicy's held target_effort is live (C2); the slider
//              is inert.  Policies and schedules are mutually exclusive —
//              assigning either replaces the other.
//   Manual   — set_rider_effort (UI slider / scripts) is live.
// Derived state (Follow > Schedule/Policy > Manual), never stored — assigning
// or clearing a follow target, schedule or policy is what switches mode.
// (Lives here rather than sim.h so DecisionContext (decision.h) can carry it
// without an include cycle.)
enum class EffortSource { Manual, Schedule, Follow, Policy };

#endif
