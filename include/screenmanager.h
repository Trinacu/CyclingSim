#ifndef SCREEN_MANAGER_H
#define SCREEN_MANAGER_H

#include "screen.h"
#include <memory>
#include <vector>

class AppState;

enum class PendingActionType { None, Push, Pop, Replace };

struct PendingAction {
  PendingActionType type = PendingActionType::None;
  ScreenType screen;
};

class ScreenManager {
public:
  ScreenManager(AppState* state);

  // Core operations
  void push(ScreenType type);
  void pop();                    // removes top screen
  void replace(ScreenType type); // pop then push

  // Game loop hooks
  void handle_event(const SDL_Event* e);
  void update();
  void render();

  // Utilities
  bool empty() const { return stack.empty(); }
  IScreen* top() const { return stack.empty() ? nullptr : stack.back().get(); }

private:
  AppState* app;
  std::vector<std::unique_ptr<IScreen>> stack;

  std::unique_ptr<IScreen> create_screen(ScreenType type);

  PendingAction pending;
};

#endif
