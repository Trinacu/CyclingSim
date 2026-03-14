// lateral_behavior.h
#ifndef LATERAL_BEHAVIOR_H
#define LATERAL_BEHAVIOR_H

// lateral_behavior.h defines the strategy interface for lateral rider
// decision-making.
//
// Dependency policy:
//   This header deliberately has NO dependency on sim.h, rider.h, or any
//   physics type beyond what is needed to describe the context a behavior
//   receives.  The engine builds a LateralContext from live Rider state and
//   hands it to the behavior; the behavior never touches Rider objects
//   directly.
//
// Responsibilities:
//   ILateralBehavior  — strategy interface: given context, return desired lat
//   LateralContext    — read-only description of one rider's local situation
//   NearbyRider       — lightweight descriptor of one nearby rider, anonymous
//                       (behaviors don't need to know which rider it is)

#include <optional>
#include <vector>

// ---------------------------------------------------------------------------
// NearbyRider
//
// A read-only, anonymous snapshot of one rider in the proximity window.
// "Anonymous" means the behavior receives no RiderId — it reasons only about
// relative positions and states, not identities.  This keeps behaviors
// decoupled from the rider registry.
// ---------------------------------------------------------------------------
struct NearbyRider {
  double lon_offset;   // metres ahead (+) or behind (-) own position
  double lat_pos;      // metres from road centre (same sign convention as own)
  double speed;        // longitudinal speed (m/s)
  double w_prime_frac; // W'bal fraction [0, 1]: 0 = empty, 1 = full
};

// ---------------------------------------------------------------------------
// LateralContext
//
// Everything a behavior needs to decide its desired lateral position.
// All fields are values, not references — the context is safe to copy and
// store.  The engine constructs one per rider per step.
//
// road_width is the full driveable width at the rider's current position.
// The usable lateral range is [-road_width/2, +road_width/2].
//
// nearby is pre-filtered by the engine to riders within the longitudinal
// proximity window (x_lookahead).  Behaviors should not assume any ordering.
// ---------------------------------------------------------------------------
struct LateralContext {
  // --- own state ---
  double own_lat_pos;      // current metres from road centre
  double own_lat_vel;      // current lateral velocity (m/s)
  double own_speed;        // longitudinal speed (m/s)
  double own_w_prime_frac; // W'bal fraction [0, 1]
  double road_width;       // full driveable width at own position (m)

  // --- proximity window ---
  // Riders within the engine's x_lookahead range, both ahead and behind.
  // Empty when no other riders are in range.
  std::vector<NearbyRider> nearby;
};

// ---------------------------------------------------------------------------
// ILateralBehavior
//
// Strategy interface.  One instance is assigned per rider.  The engine calls
// compute_desired_lat() during step_lateral_behavior() and writes the result
// to rider->lat_target.
//
// Contract:
//   - Returns a desired lateral position in metres from road centre.
//   - The return value SHOULD be clamped to [-road_width/2, +road_width/2]
//     by the behavior itself, though the solver will clamp it again as a
//     safety measure.
//   - Must not modify any shared state.  The method is const.
//   - Must be cheap — called every physics step (100 Hz) for every rider.
// ---------------------------------------------------------------------------
class ILateralBehavior {
public:
  virtual ~ILateralBehavior() = default;

  virtual std::optional<double>
  compute_lat_target(const LateralContext& ctx) const = 0;
};

class HoldLineBehavior : public ILateralBehavior {
  std::optional<double>
  compute_lat_target(const LateralContext& ctx) const override;
};

class OvertakeBehavior : public ILateralBehavior {
public:
  enum class Side { Left, Right, Auto };

  explicit OvertakeBehavior(Side preferred_side = Side::Auto,
                            double rider_radius = 0.5,
                            double arrival_tol = 0.15);

  std::optional<double>
  compute_lat_target(const LateralContext& ctx) const override;

private:
  Side preferred_side_;
  double rider_radius_;
  double arrival_tol_;
};

class BlockBehavior : public ILateralBehavior {
public:
  explicit BlockBehavior(double fatigue_threshold = 0.15);

  std::optional<double>
  compute_lat_target(const LateralContext& ctx) const override;

private:
  double fatigue_threshold_;
};

#endif
