#ifndef GROUP_H
#define GROUP_H

#include "grouping_params.h"
#include "mytypes.h"
#include <SDL3/SDL.h>
#include <string>
#include <unordered_map>
#include <vector>

typedef enum { Unassigned = 0, Paceline = 1, Body = 2 } GroupRole;

typedef struct GroupMember {
  RiderId id;
  double lon_pos;
  double speed;
  GroupRole role;
} GroupMember;

typedef struct Group {
  GroupId id;
  int ordinal;
  std::string display_name;
  std::vector<GroupMember> paceline;
  std::vector<GroupMember> body;

  int size() const { return static_cast<int>(paceline.size() + body.size()); }

  // Position of the frontmost member.
  // Returns 0.0 if the group is empty (should not happen after update).
  double front_pos() const;

  // Position of the rearmost member.
  double back_pos() const;

  // Longitudinal extent from rear to front, in meters
  double span() const { return front_pos() - back_pos(); }

  std::vector<GroupMember> all_members() const;
} Group;

using GroupSnapshot = std::vector<Group>;

std::string default_group_label(int ordinal, int size);

class GroupTracker {
public:
  explicit GroupTracker(const GroupingParams& params);

  // Rebuild the GroupSnapshot from the current rider positions.
  // members may be in any order — sorted internally.
  // All members start in group.body; paceline is empty after this call.
  void update(const std::vector<GroupMember>& members);

  // Distribute members into paceline / body according to their declarations.
  // Must be called after update() and before reading the snapshot.
  // Riders absent from decls, or with role == Unassigned, go into body.
  void
  apply_role_declarations(const std::unordered_map<RiderId, GroupRole>& decls);

  // --- read accessors ---

  // Returns kNoGroup if the rider is not in the current snapshot.
  GroupId get_group_id(RiderId id) const;

  // Returns Unassigned if the rider is not in the current snapshot.
  GroupRole get_role(RiderId id) const;

  // The full snapshot — valid after update() and apply_role_declarations().
  // Ordered front-to-back: index 0 is the leading group.
  const GroupSnapshot& get_snapshot() const;

private:
  GroupingParams params_;

  // Rebuilt from scratch each tick.
  GroupSnapshot snapshot_;
  std::unordered_map<RiderId, GroupId> rider_to_group_;
  std::unordered_map<RiderId, GroupRole> rider_to_role_;
};

inline SDL_FColor group_colour(GroupId ordinal) {
  if (ordinal < 0)
    return {0.55f, 0.55f, 0.55f, 1.0f}; // kNoGroup → gray

  static constexpr SDL_FColor palette[] = {
      {0.20f, 0.65f, 1.00f, 1.0f}, // 0 — blue   (front group / breakaway)
      {0.20f, 0.85f, 0.45f, 1.0f}, // 1 — green  (first chase)
      {1.00f, 0.70f, 0.10f, 1.0f}, // 2 — amber
      {0.90f, 0.30f, 0.30f, 1.0f}, // 3 — red
      {0.75f, 0.40f, 1.00f, 1.0f}, // 4 — purple
      {1.00f, 0.50f, 0.20f, 1.0f}, // 5 — orange
      {0.20f, 0.90f, 0.90f, 1.0f}, // 6 — cyan
      {1.00f, 0.40f, 0.80f, 1.0f}, // 7 — pink
  };
  constexpr int N = static_cast<int>(sizeof(palette) / sizeof(palette[0]));
  return palette[ordinal % N];
}

struct GroupContext {
  // --- own identity within the group ---
  GroupId own_group_id = kNoGroup;
  GroupRole own_role = GroupRole::Unassigned;

  // --- group topology ---
  int group_ordinal = 0; // 0 = front group
  int group_size = 0;    // total members (paceline + body)
  int paceline_size = 0;
  int body_size = 0;

  // --- position within sub-container ---
  // For paceline riders: 0 = front (pulling), increasing toward the back.
  // -1 when not in the paceline.
  int paceline_position = -1;

  bool is_paceline_front = false; // true only for paceline_position == 0
  bool is_group_front = false;    // true for the frontmost rider in the group
                                  // regardless of role

  // --- gap context ---
  // Distance in metres to the rear of the group ahead (kNoGroup ordinal-1).
  // -1 when own group is the leading group.
  double gap_to_group_ahead = -1.0;
};

#endif
