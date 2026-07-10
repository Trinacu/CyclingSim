#include "rotation.h"
#include <algorithm>
#include <cmath>

namespace {

const RotationInput* find_input(const std::vector<RotationInput>& in,
                                RiderId id) {
  for (const auto& s : in)
    if (s.id == id)
      return &s;
  return nullptr;
}

// Drop every id from `list` that has no input this tick (rider vanished).
void prune_missing(std::vector<RiderId>& list,
                   const std::vector<RotationInput>& in) {
  list.erase(std::remove_if(list.begin(), list.end(),
                            [&in](RiderId id) {
                              return find_input(in, id) == nullptr;
                            }),
             list.end());
}

} // namespace

PacelineRotation::PacelineRotation(const std::vector<RotationMember>& roster,
                                   const RotationParams& params)
    : params_(params) {
  for (const auto& m : roster) {
    if (m.sits_in)
      sitting_.push_back(m.id);
    else
      inline_.push_back(m.id);
  }
}

bool PacelineRotation::is_member(RiderId id) const {
  auto has = [id](const std::vector<RiderId>& v) {
    return std::find(v.begin(), v.end(), id) != v.end();
  };
  return has(inline_) || has(drifting_) || has(sitting_) || has(promoting_);
}

bool PacelineRotation::promote_sitter(RiderId id) {
  auto it = std::find(sitting_.begin(), sitting_.end(), id);
  if (it == sitting_.end() || inline_.empty())
    return false;

  const bool first = (it == sitting_.begin());
  sitting_.erase(it);
  detach_timer(id) = 0.0;
  if (first)
    inline_.push_back(id); // already on the tail's wheel: pure bookkeeping
  else
    promoting_.push_back(id); // physical move-up; attaches in tick()
  return true;
}

double& PacelineRotation::detach_timer(RiderId id) {
  for (auto& [tid, t] : detach_timers_)
    if (tid == id)
      return t;
  detach_timers_.emplace_back(id, 0.0);
  return detach_timers_.back().second;
}

std::vector<RotationDirective>
PacelineRotation::tick(double dt, const std::vector<RotationInput>& in) {
  removed_.clear();
  prune_missing(inline_, in);
  prune_missing(drifting_, in);
  prune_missing(sitting_, in);
  prune_missing(promoting_, in);

  // --- 1. Attach drifters positionally ---
  // A drifter whose position has dropped below the last InLine rider's joins
  // the back of the line.  When several have crossed in the same tick,
  // attach front-to-back (highest position first) so they append in spatial
  // order.
  bool attached = true;
  while (attached && !drifting_.empty() && !inline_.empty()) {
    attached = false;
    const RotationInput* tail = find_input(in, inline_.back());
    int best = -1;
    double best_pos = 0.0;
    for (int i = 0; i < static_cast<int>(drifting_.size()); ++i) {
      const RotationInput* d = find_input(in, drifting_[i]);
      if (d->lon_pos < tail->lon_pos && (best < 0 || d->lon_pos > best_pos)) {
        best = i;
        best_pos = d->lon_pos;
      }
    }
    if (best >= 0) {
      inline_.push_back(drifting_[best]);
      drifting_.erase(drifting_.begin() + best);
      attached = true;
    }
  }

  // --- 1b. Attach promoting sitters ---
  // Positional, like the drifter rule: a promoting rider joins the InLine
  // tail once it has passed the first sitter — nobody from the queue is left
  // between it and the line, so the first sitter's dynamic retarget to the
  // new tail lands on a rider *ahead* of it (its controller opens the slot
  // naturally, same geometry as the drifter merge).  With no sitters left
  // the criterion falls back to closing within engage_gap of the tail.  The
  // follow subsystem finishes the cut-in (offset fade) on its own.  Several
  // arriving in one tick append in spatial order, frontmost first.  Like
  // drifters, promoting riders are exempt from the detach rule below — the
  // whole point of the transit is riding deliberately gapped; one that never
  // closes just keeps riding capped (smarter handling is C4-era).
  {
    bool attached_p = true;
    while (attached_p && !promoting_.empty() && !inline_.empty()) {
      attached_p = false;
      const RotationInput* tail = find_input(in, inline_.back());
      int best = -1;
      double best_pos = 0.0;
      for (int i = 0; i < static_cast<int>(promoting_.size()); ++i) {
        const RotationInput* p = find_input(in, promoting_[i]);
        bool arrived;
        if (!sitting_.empty()) {
          const RotationInput* front_sitter = find_input(in, sitting_.front());
          arrived = p->lon_pos > front_sitter->lon_pos;
        } else {
          arrived =
              (tail->lon_pos - p->lon_pos - tail->bike_len) <=
              params_.engage_gap;
        }
        if (arrived && (best < 0 || p->lon_pos > best_pos)) {
          best = i;
          best_pos = p->lon_pos;
        }
      }
      if (best >= 0) {
        inline_.push_back(promoting_[best]);
        detach_timer(promoting_[best]) = 0.0;
        promoting_.erase(promoting_.begin() + best);
        attached_p = true;
      }
    }
  }

  // --- 2. Detach cleanup ---
  // A member riding detached from the rider ahead for detach_time is removed
  // from the roster.  Drifters are exempt: their "gap" to the tail is
  // negative by construction while merging; a blown drifter attaches
  // positionally first and is then caught by this rule as an InLine member.
  {
    std::vector<std::pair<RiderId, RiderId>> pairs; // (member, ahead)
    for (size_t i = 1; i < inline_.size(); ++i)
      pairs.emplace_back(inline_[i], inline_[i - 1]);
    if (!sitting_.empty() && !inline_.empty()) {
      pairs.emplace_back(sitting_[0], inline_.back());
      for (size_t k = 1; k < sitting_.size(); ++k)
        pairs.emplace_back(sitting_[k], sitting_[k - 1]);
    }

    for (const auto& [id, ahead_id] : pairs) {
      const RotationInput* me = find_input(in, id);
      const RotationInput* ahead = find_input(in, ahead_id);
      const double gap = ahead->lon_pos - me->lon_pos - ahead->bike_len;
      double& timer = detach_timer(id);
      if (gap > params_.detach_gap)
        timer += dt;
      else
        timer = 0.0;

      if (timer > params_.detach_time) {
        removed_.push_back(id);
        for (auto* list : {&inline_, &drifting_, &sitting_})
          list->erase(std::remove(list->begin(), list->end(), id),
                      list->end());
      }
    }
  }

  // --- 3. Pull-end trigger ---
  // The timer advances only while the line is engaged (first follower on the
  // puller's wheel), and a rotation never leaves fewer than 2 riders in line.
  RiderId promoted = -1;
  double promoted_effort = 0.0;
  RiderId swung = -1;
  double side = 0.0;
  if (inline_.size() >= 3) {
    const RotationInput* pull = find_input(in, inline_[0]);
    const RotationInput* second = find_input(in, inline_[1]);
    const double engage =
        pull->lon_pos - second->lon_pos - pull->bike_len;
    if (engage < params_.engage_gap)
      pull_timer_ += dt;

    if (pull_timer_ >= params_.pull_time) {
      pull_timer_ = 0.0;
      swung = inline_.front();
      // Windward side: the wake blows toward +crosswind, so the exposed side
      // is the opposite one.
      side = (std::fabs(pull->crosswind) > 1e-6)
                 ? (pull->crosswind > 0.0 ? -1.0 : 1.0)
                 : params_.default_side;
      promoted = inline_[1];
      promoted_effort = pull->target_effort;
      inline_.erase(inline_.begin());
      drifting_.push_back(swung);
      detach_timer(swung) = 0.0;
    }
  }

  // --- 4. Directives ---
  std::vector<RotationDirective> out;
  out.reserve(inline_.size() + drifting_.size() + sitting_.size() +
              promoting_.size());
  for (size_t i = 0; i < inline_.size(); ++i) {
    RotationDirective d;
    d.id = inline_[i];
    if (i == 0) {
      d.pulling = true;
      if (d.id == promoted)
        d.set_effort = promoted_effort;
    } else {
      d.follow = inline_[i - 1];
    }
    out.push_back(d);
  }
  // A fully depleted line (everyone removed) leaves drifters and sitters
  // with follow = -1; the engine treats that as "no directive".
  const RiderId tail = inline_.empty() ? -1 : inline_.back();
  for (RiderId id : drifting_) {
    RotationDirective d;
    d.id = id;
    d.follow = tail; // dynamic: always the current tail
    if (id == swung)
      d.swing_side = side;
    out.push_back(d);
  }
  for (size_t k = 0; k < sitting_.size(); ++k) {
    RotationDirective d;
    d.id = sitting_[k];
    d.follow = (k == 0) ? tail : sitting_[k - 1];
    out.push_back(d);
  }
  for (RiderId id : promoting_) {
    RotationDirective d;
    d.id = id;
    d.follow = tail; // dynamic: always the current tail
    if (tail >= 0) {
      // Advance side: opposite the swing (windward) side — move up on the
      // sheltered flank, clear of drifting traffic.
      const RotationInput* me = find_input(in, id);
      d.move_up_side = (std::fabs(me->crosswind) > 1e-6)
                           ? (me->crosswind > 0.0 ? 1.0 : -1.0)
                           : -params_.default_side;
    }
    out.push_back(d);
  }
  return out;
}
