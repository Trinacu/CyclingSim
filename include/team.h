// team.h — team registry (C-pre-a).
//
// Teams are static race entities, deliberately unlike GroupTracker's emergent
// proximity groups: created once at scenario setup, never rebuilt.  A Team
// holds the roster (and, C4-era, the race plan its director works from).
// Riders carry only a TeamId — same identity pattern as RiderId/GroupId — so
// configs and snapshots stay trivially copyable; the registry is the single
// owner of Team state.  Header-only, same precedent as effortschedule.h.

#ifndef TEAM_H
#define TEAM_H

#include "mytypes.h"
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

struct Team {
  TeamId id = kNoTeam;
  std::string name;
  std::vector<RiderId> roster;
};

class TeamRegistry {
public:
  TeamId add_team(std::string name) {
    const TeamId id = static_cast<TeamId>(teams_.size());
    teams_.push_back(Team{id, std::move(name), {}});
    return id;
  }

  // nullptr for kNoTeam / unknown ids.
  const Team* get(TeamId id) const {
    if (id < 0 || id >= static_cast<TeamId>(teams_.size()))
      return nullptr;
    return &teams_[id];
  }

  // kNoTeam if the rider was never registered.
  TeamId team_of(RiderId rider) const {
    auto it = rider_to_team_.find(rider);
    return it == rider_to_team_.end() ? kNoTeam : it->second;
  }

  // Records the rider→team mapping and appends to the team roster.  An
  // unknown team id degrades to kNoTeam — the rider rides unaffiliated.
  void register_rider(RiderId rider, TeamId team) {
    if (!get(team)) {
      rider_to_team_[rider] = kNoTeam;
      return;
    }
    rider_to_team_[rider] = team;
    teams_[team].roster.push_back(rider);
  }

  int team_count() const { return static_cast<int>(teams_.size()); }

private:
  std::vector<Team> teams_;
  std::unordered_map<RiderId, TeamId> rider_to_team_;
};

#endif
