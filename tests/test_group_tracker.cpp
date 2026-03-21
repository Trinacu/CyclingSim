#include "group.h"
#include <cassert>
#include <cstdio>
#include <string>

// ---------------------------------------------------------------------------
// Minimal test harness — no SDL, no PhysicsEngine
// ---------------------------------------------------------------------------

static int s_pass = 0;
static int s_fail = 0;

static void check(bool condition, const char* label) {
  if (condition) {
    printf("  PASS  %s\n", label);
    ++s_pass;
  } else {
    printf("  FAIL  %s\n", label);
    ++s_fail;
  }
}

// Convenience: build a GroupMember with just id and lon_pos
static GroupMember m(RiderId id, double lon_pos, double speed = 10.0) {
  return GroupMember{id, lon_pos, speed, GroupRole::Unassigned};
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

static void test_empty() {
  printf("\n--- empty input ---\n");
  GroupTracker tracker(GroupingParams{});
  tracker.update({});
  check(tracker.get_snapshot().empty(), "snapshot is empty");
  check(tracker.get_group_id(0) == kNoGroup, "unknown rider returns kNoGroup");
  check(tracker.get_role(0) == GroupRole::Unassigned,
        "unknown rider returns Unassigned");
}

static void test_single_rider() {
  printf("\n--- single rider ---\n");
  GroupTracker tracker(GroupingParams{});
  tracker.update({m(0, 100.0)});

  const auto& snap = tracker.get_snapshot();
  check(snap.size() == 1, "one group");
  check(snap[0].ordinal == 0, "ordinal is 0");
  check(snap[0].size() == 1, "group has one member");
  check(tracker.get_group_id(0) == 0, "rider 0 is in group 0");
  check(snap[0].display_name == "Group 1", "display_name is Group 1");
}

static void test_all_in_one_group() {
  printf("\n--- all riders in one group ---\n");
  // Gaps: 5, 3, 4 — all below threshold of 10
  GroupTracker tracker(GroupingParams{10.0});
  tracker.update({
      m(0, 100.0),
      m(1, 95.0),
      m(2, 92.0),
      m(3, 88.0),
  });

  const auto& snap = tracker.get_snapshot();
  check(snap.size() == 1, "one group");
  check(snap[0].size() == 4, "four members");
  check(tracker.get_group_id(0) == 0, "rider 0 in group 0");
  check(tracker.get_group_id(3) == 0, "rider 3 in group 0");
}

static void test_all_separate() {
  printf("\n--- all riders in separate groups ---\n");
  // Gaps: 15, 20, 25 — all above threshold of 10
  GroupTracker tracker(GroupingParams{10.0});
  tracker.update({
      m(0, 100.0),
      m(1, 85.0),
      m(2, 65.0),
      m(3, 40.0),
  });

  const auto& snap = tracker.get_snapshot();
  check(snap.size() == 4, "four groups");
  check(tracker.get_group_id(0) == 0, "rider 0 in group 0 (front)");
  check(tracker.get_group_id(1) == 1, "rider 1 in group 1");
  check(tracker.get_group_id(2) == 2, "rider 2 in group 2");
  check(tracker.get_group_id(3) == 3, "rider 3 in group 3 (back)");
}

static void test_split() {
  printf("\n--- group splits when gap opens ---\n");
  GroupTracker tracker(GroupingParams{10.0});

  // Tick 1: all together
  tracker.update({m(0, 100.0), m(1, 95.0), m(2, 90.0)});
  check(tracker.get_snapshot().size() == 1, "tick 1: one group");

  // Tick 2: rider 2 drops off the back
  tracker.update({m(0, 110.0), m(1, 105.0), m(2, 90.0)});
  //             gap 0→1 = 5 (together), gap 1→2 = 15 (split)
  const auto& snap = tracker.get_snapshot();
  check(snap.size() == 2, "tick 2: two groups");
  check(tracker.get_group_id(0) == 0, "riders 0+1 in front group");
  check(tracker.get_group_id(1) == 0, "riders 0+1 in front group");
  check(tracker.get_group_id(2) == 1, "rider 2 in rear group");
  check(snap[0].size() == 2, "front group has 2 members");
  check(snap[1].size() == 1, "rear group has 1 member");
}

static void test_merge() {
  printf("\n--- groups merge when gap closes ---\n");
  GroupTracker tracker(GroupingParams{10.0});

  // Tick 1: two separate groups
  tracker.update({m(0, 100.0), m(1, 80.0)});
  check(tracker.get_snapshot().size() == 2, "tick 1: two groups");

  // Tick 2: rear rider closes the gap
  tracker.update({m(0, 100.0), m(1, 93.0)});
  check(tracker.get_snapshot().size() == 1, "tick 2: one group after merge");
  check(tracker.get_group_id(0) == 0, "rider 0 in merged group");
  check(tracker.get_group_id(1) == 0, "rider 1 in merged group");
}

static void test_three_way_split() {
  printf("\n--- three-way split ---\n");
  GroupTracker tracker(GroupingParams{10.0});

  tracker.update({
      m(0, 200.0),
      m(1, 195.0), // gap 0→1 = 5  → same group
      m(2, 180.0), // gap 1→2 = 15 → split
      m(3, 175.0), // gap 2→3 = 5  → same group as 2
      m(4, 155.0), // gap 3→4 = 20 → split
      m(5, 150.0), // gap 4→5 = 5  → same group as 4
  });

  const auto& snap = tracker.get_snapshot();
  check(snap.size() == 3, "three groups");
  check(snap[0].size() == 2, "front group: riders 0+1");
  check(snap[1].size() == 2, "middle group: riders 2+3");
  check(snap[2].size() == 2, "rear group: riders 4+5");
  check(tracker.get_group_id(0) == 0, "rider 0 in group 0");
  check(tracker.get_group_id(2) == 1, "rider 2 in group 1");
  check(tracker.get_group_id(4) == 2, "rider 4 in group 2");
}

static void test_middle_group_absorbed() {
  printf("\n--- middle group absorbed into front ---\n");
  GroupTracker tracker(GroupingParams{10.0});

  // Tick 1: three groups
  tracker.update({m(0, 200.0), m(1, 185.0), m(2, 170.0)});
  check(tracker.get_snapshot().size() == 3, "tick 1: three groups");

  // Tick 2: middle rider bridges to front
  tracker.update({m(0, 210.0), m(1, 203.0), m(2, 185.0)});
  //             gap 0→1 = 7 (joined), gap 1→2 = 18 (still split)
  const auto& snap2 = tracker.get_snapshot();
  check(snap2.size() == 2, "tick 2: two groups");
  check(snap2[0].size() == 2, "front group now has 2 members");
  check(tracker.get_group_id(0) == 0, "rider 0 still in front group");
  check(tracker.get_group_id(1) == 0, "rider 1 joined front group");
  check(tracker.get_group_id(2) == 1, "rider 2 alone in rear group");
}

static void test_ordinal_is_front_to_back() {
  printf(
      "\n--- ordinal is always front-to-back regardless of input order ---\n");
  GroupTracker tracker(GroupingParams{10.0});

  // Input is deliberately in reverse order
  tracker.update({
      m(3, 40.0),
      m(0, 100.0),
      m(2, 65.0),
      m(1, 85.0),
  });

  const auto& snap = tracker.get_snapshot();
  // Sorted positions: 100, 85, 65, 40 → all gaps > 10 → four groups
  check(snap.size() == 4, "four groups");
  check(tracker.get_group_id(0) == 0, "rider at 100 is group 0 (front)");
  check(tracker.get_group_id(1) == 1, "rider at 85 is group 1");
  check(tracker.get_group_id(2) == 2, "rider at 65 is group 2");
  check(tracker.get_group_id(3) == 3, "rider at 40 is group 3 (back)");
}

static void test_role_declarations() {
  printf("\n--- role declarations ---\n");
  GroupTracker tracker(GroupingParams{10.0});

  // Two groups: {0,1,2} and {3,4}
  tracker.update({
      m(0, 100.0),
      m(1, 96.0),
      m(2, 93.0),
      m(3, 70.0),
      m(4, 67.0),
  });

  std::unordered_map<RiderId, GroupRole> decls = {
      {0, GroupRole::Paceline},
      {1, GroupRole::Paceline},
      {2, GroupRole::Body},
      {3, GroupRole::Paceline},
      // rider 4 absent from decls → Unassigned → body
  };
  tracker.apply_role_declarations(decls);

  const auto& snap = tracker.get_snapshot();

  // Front group: 2 paceline, 1 body
  check(snap[0].paceline.size() == 2, "front group: 2 in paceline");
  check(snap[0].body.size() == 1, "front group: 1 in body");

  // Paceline is sorted front-to-back: rider 0 (pos 100) before rider 1 (pos 96)
  check(snap[0].paceline[0].id == 0, "paceline index 0 is rider 0 (frontmost)");
  check(snap[0].paceline[1].id == 1, "paceline index 1 is rider 1");
  check(snap[0].body[0].id == 2, "body contains rider 2");

  // Rear group: 1 paceline, 1 body (rider 4 defaulted to body)
  check(snap[1].paceline.size() == 1, "rear group: 1 in paceline");
  check(snap[1].body.size() == 1, "rear group: 1 in body (defaulted)");
  check(snap[1].paceline[0].id == 3, "rear group paceline is rider 3");
  check(snap[1].body[0].id == 4, "rear group body is rider 4 (defaulted)");

  // Role queries
  check(tracker.get_role(0) == GroupRole::Paceline, "get_role(0) == Paceline");
  check(tracker.get_role(2) == GroupRole::Body, "get_role(2) == Body");
  check(tracker.get_role(4) == GroupRole::Unassigned,
        "get_role(4) == Unassigned");
}

static void test_role_declarations_cleared_each_tick() {
  printf("\n--- role declarations do not persist across ticks ---\n");
  GroupTracker tracker(GroupingParams{10.0});

  tracker.update({m(0, 100.0), m(1, 95.0)});
  tracker.apply_role_declarations(
      {{0, GroupRole::Paceline}, {1, GroupRole::Paceline}});

  check(tracker.get_snapshot()[0].paceline.size() == 2,
        "tick 1: 2 in paceline");

  // Tick 2: update without any declarations
  tracker.update({m(0, 110.0), m(1, 105.0)});
  // apply_role_declarations with empty map
  tracker.apply_role_declarations({});

  check(tracker.get_snapshot()[0].paceline.size() == 0,
        "tick 2: paceline is empty");
  check(tracker.get_snapshot()[0].body.size() == 2, "tick 2: both in body");
  check(tracker.get_role(0) == GroupRole::Unassigned, "tick 2: role cleared");
}

// ---------------------------------------------------------------------------
// main
// ---------------------------------------------------------------------------

int main() {
  printf("=== GroupTracker tests ===\n");

  test_empty();
  test_single_rider();
  test_all_in_one_group();
  test_all_separate();
  test_split();
  test_merge();
  test_three_way_split();
  test_middle_group_absorbed();
  test_ordinal_is_front_to_back();
  test_role_declarations();
  test_role_declarations_cleared_each_tick();

  printf("\n=== Results: %d passed, %d failed ===\n", s_pass, s_fail);
  return s_fail == 0 ? 0 : 1;
}
