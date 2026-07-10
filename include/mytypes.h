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
//              slider and any effort schedule are inert.
//   Schedule — an EffortSchedule drives effort each tick; the slider is inert.
//   Manual   — set_rider_effort (UI slider / scripts) is live.
// Derived state (Follow > Schedule > Manual), never stored — assigning or
// clearing a follow target or schedule is what switches mode.  (Lives here
// rather than sim.h so DecisionContext (decision.h) can carry it without an
// include cycle.)
enum class EffortSource { Manual, Schedule, Follow };

#endif
