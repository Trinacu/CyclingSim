#include "appstate.h"
#include "screen.h"

void AppState::switch_screen(ScreenType type) {
  delete screen;
  current_screen = type;

  switch (type) {
  case ScreenType::Menu:
    screen = new MenuScreen(this);
    break;
  case ScreenType::Simulation:
    screen = new SimulationScreen(this);
    break;
  case ScreenType::Result:
    screen = new ResultsScreen(this);
    break;
  }
}
