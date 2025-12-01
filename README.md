#Cycling Simulation (C++ / SDL / ImGui / Eigen)

A real-time cycling physics simulation written in modern C++.
Features:

Physically-based rider model

Aerodynamics, gravity, inertia, rolling resistance

Variable wind (per course segment)

Multi-rider simulation

Real-time engine with time-scaling

ImGui/ImPlot UI overlays

SDL3 rendering + custom world renderer

Snapshot-based thread-safe rendering architecture

📁 Project Structure
/
├── src/                # C++ source files
├── include/            # Headers (including Eigen)
├── vendor/             # (Ignored) third-party like ImGui, ImPlot
├── resources/          # Textures, fonts, course data
├── CMakeLists.txt      # (If you use CMake)
└── README.md

🚴 Physics Model

Each rider simulates:

Aerodynamic drag (CdA, yaw, wheel drag)

Rolling resistance (Crr)

Gravitational forces based on course slope

Rotational inertia of wheels

Headwind (wind.heading, wind.speed)

Energy model (planned)

Newton & Householder solvers for the speed–power equation

🖥 Rendering

Rendering is split into:

World drawables: course, riders

UI drawables: stopwatch, time controls, rider panel

Snapshot-based double buffer for thread-safe physics → render data transfer

SDL3 for 2D rendering

ImGui / ImPlot overlays for debug or analysis

🎮 Controls (current)

Left-click rider (bottom right point) → focus camera & update rider panel

Right-click drag → pan camera (future)

Mouse wheel → zoom (future)

ESC → back to menu
P →  plot screen
S →  simulation screen

