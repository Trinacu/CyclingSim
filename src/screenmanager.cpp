#include "screenmanager.h"
#include "appstate.h"
#include "screen.h"

ScreenManager::ScreenManager(AppState* state) : app(state) {}

std::unique_ptr<IScreen> ScreenManager::create_screen(ScreenType type) {
  switch (type) {
  case ScreenType::Menu:
    return std::make_unique<MenuScreen>(app);
  case ScreenType::Simulation:
    return std::make_unique<SimulationScreen>(app);
  case ScreenType::Result:
    return std::make_unique<ResultsScreen>(app);
  case ScreenType::Plot:
    return std::make_unique<PlotScreen>(app);
  }
  return nullptr;
}

void ScreenManager::push(ScreenType type) {
  // stack.push_back(create_screen(type));
  pending.type = PendingActionType::Push;
  pending.screen = type;
}

void ScreenManager::pop() {
  // if (!stack.empty()) {
  //   stack.pop_back();
  // }
  pending.type = PendingActionType::Pop;
}

void ScreenManager::replace(ScreenType type) {
  // pop();
  // push(type);
  pending.type = PendingActionType::Replace;
  pending.screen = type;
}

void ScreenManager::handle_event(const SDL_Event* e) {
  if (!stack.empty()) {
    stack.back()->handle_event(e);
  }
}

void ScreenManager::update() {
  // --- 1. Apply queued transition BEFORE updating top screen ---
  if (pending.type != PendingActionType::None) {
    switch (pending.type) {
    case PendingActionType::Push:
      stack.push_back(create_screen(pending.screen));
      break;

    case PendingActionType::Pop:
      if (!stack.empty())
        stack.pop_back();
      break;

    case PendingActionType::Replace:
      if (!stack.empty())
        stack.pop_back();
      stack.push_back(create_screen(pending.screen));
      break;

    default:
      break;
    }

    pending.type = PendingActionType::None;
  }

  // --- 2. Now update current top screen safely ---
  if (!stack.empty()) {
    stack.back()->update();
  }
  // or if we add modal
  // for (auto& s : stack)
  // if (s->is_modal()) break;
  // else s->update();
  // screens get virtual bool is_modal()
}

void ScreenManager::render() {
  // Important: render *all* screens if you want overlays.
  for (auto& s : stack)
    s->render();
}
