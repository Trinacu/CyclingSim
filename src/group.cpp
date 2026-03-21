#include "group.h"
#include <algorithm>
#include <limits>

double Group::front_pos() const {
  double p = std::numeric_limits<double>::lowest();
  for (const auto& m : paceline)
    p = std::max(p, m.lon_pos);
  for (const auto& m : body)
    p = std::max(p, m.lon_pos);
  return (p == std::numeric_limits<double>::lowest()) ? 0.0 : p;
}

double Group::back_pos() const {
  double p = std::numeric_limits<double>::max();
  for (const auto& m : paceline)
    p = std::min(p, m.lon_pos);
  for (const auto& m : body)
    p = std::min(p, m.lon_pos);
  return (p == std::numeric_limits<double>::max()) ? 0.0 : p;
}

std::vector<GroupMember> Group::all_members() const {
  std::vector<GroupMember> result;
  result.reserve(paceline.size() + body.size());
  result.insert(result.end(), paceline.begin(), paceline.end());
  result.insert(result.end(), body.begin(), body.end());
  return result;
}

std::string default_group_label(int ordinal, int size) {
  return "Group " + std::to_string(ordinal + 1);
}

GroupTracker::GroupTracker(const GroupingParams& params) : params_(params) {}

// ---------------------------------------------------------------------------
// update
//
// Algorithm:
//   1. Early-out on empty input.
//   2. Sort members descending by lon_pos (front of race = index 0).
//   3. Walk consecutive pairs.  Cut a new group wherever the gap between
//      adjacent riders exceeds gap_threshold.
//   4. Assign ordinal, id, and display_name to each group.
//   5. Rebuild rider_to_group_ and rider_to_role_ lookup maps.
//
// All members land in group.body after this call.
// paceline is empty until apply_role_declarations() runs.
// ---------------------------------------------------------------------------
void GroupTracker::update(const std::vector<GroupMember>& members) {
  snapshot_.clear();
  rider_to_group_.clear();
  rider_to_role_.clear();

  if (members.empty())
    return;

  // Step 1 — sort a local copy; preserve caller's buffer
  std::vector<GroupMember> sorted = members;
  std::sort(sorted.begin(), sorted.end(),
            [](const GroupMember& a, const GroupMember& b) {
              return a.lon_pos > b.lon_pos; // descending: front first
            });

  // Step 2 — scan and cut into groups
  Group current;
  current.body.push_back(sorted[0]);

  for (int i = 1; i < static_cast<int>(sorted.size()); ++i) {
    const double gap = sorted[i - 1].lon_pos - sorted[i].lon_pos;

    if (gap > params_.gap_threshold) {
      // Gap is too large — close current group and open a new one
      const int ordinal = static_cast<int>(snapshot_.size());
      current.ordinal = ordinal;
      current.id = ordinal;
      current.display_name = default_group_label(ordinal, current.size());
      snapshot_.push_back(std::move(current));
      current = Group{};
    }

    current.body.push_back(sorted[i]);
  }

  // Close the final (or only) group
  const int ordinal = static_cast<int>(snapshot_.size());
  current.ordinal = ordinal;
  current.id = ordinal;
  current.display_name = default_group_label(ordinal, current.size());
  snapshot_.push_back(std::move(current));

  // Step 3 — rebuild lookup maps
  for (const auto& group : snapshot_) {
    for (const auto& m : group.body) {
      rider_to_group_[m.id] = group.id;
      rider_to_role_[m.id] = GroupRole::Unassigned;
    }
  }
}

// ---------------------------------------------------------------------------
// apply_role_declarations
//
// For each group, collect all current members (all in body at this point),
// then redistribute according to the declarations map.
// Riders absent from decls go into body as Unassigned.
// paceline is sorted front-to-back after distribution.
// ---------------------------------------------------------------------------
void GroupTracker::apply_role_declarations(
    const std::unordered_map<RiderId, GroupRole>& decls) {

  for (auto& group : snapshot_) {
    // All members are currently in body (placed there by update())
    std::vector<GroupMember> all = std::move(group.body);
    group.paceline.clear();
    group.body.clear();

    for (auto& m : all) {
      const auto it = decls.find(m.id);
      const GroupRole role =
          (it != decls.end()) ? it->second : GroupRole::Unassigned;
      m.role = role;

      if (role == GroupRole::Paceline)
        group.paceline.push_back(m);
      else
        group.body.push_back(m);

      rider_to_role_[m.id] = role;
    }

    // Paceline ordered front-to-back: index 0 is the paceline leader
    std::sort(group.paceline.begin(), group.paceline.end(),
              [](const GroupMember& a, const GroupMember& b) {
                return a.lon_pos > b.lon_pos;
              });
  }
}

GroupId GroupTracker::get_group_id(RiderId id) const {
  const auto it = rider_to_group_.find(id);
  return (it != rider_to_group_.end()) ? it->second : kNoGroup;
}

GroupRole GroupTracker::get_role(RiderId id) const {
  const auto it = rider_to_role_.find(id);
  return (it != rider_to_role_.end()) ? it->second : GroupRole::Unassigned;
}

const GroupSnapshot& GroupTracker::get_snapshot() const { return snapshot_; }
