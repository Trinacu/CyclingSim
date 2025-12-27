#!/usr/bin/env python3
from pathlib import Path
import datetime

ROOT = Path(__file__).resolve().parents[1]
SRC = ROOT / "src"
KB = ROOT / "kb"

KB.mkdir(exist_ok=True)

def write(path: Path, content: str):
    path.write_text(content.strip() + "\n", encoding="utf-8")

def header(title: str) -> str:
    ts = datetime.datetime.now().isoformat(timespec="seconds")
    return f"""{title}
{'=' * len(title)}

Generated on: {ts}
"""

# ---- Project overview ----
write(
    KB / "00_project_overview.txt",
    header("Project Overview") + """
This is a C++ physics simulation project using SDL.

Core goals:
- Deterministic simulation
- Decoupled simulation and rendering
- Strict ownership boundaries

Non-goals:
- Real-time network sync
- ECS-style global state

Constraints:
- SOLID principles are enforced
- No hidden cross-module dependencies
"""
)

# ---- Architecture ----
modules = [p.name for p in SRC.iterdir() if p.is_dir()]

write(
    KB / "01_architecture.txt",
    header("Architecture Overview") + f"""
High-level modules:

{chr(10).join(f"- {m}" for m in sorted(modules))}

Dependency direction:
- core has no dependencies
- physics depends on core
- rendering depends on core
- main wires everything together
"""
)

# ---- Physics ----
physics_files = list((SRC / "physics").rglob("*.cpp"))

write(
    KB / "02_physics_model.txt",
    header("Physics Model") + f"""
Physics is implemented in the following files:

{chr(10).join(f"- {p.relative_to(SRC)}" for p in physics_files)}

Key assumptions:
- Fixed timestep
- Deterministic integration
- No rendering-side mutation
"""
)

# ---- Rendering ----
render_files = list((SRC / "render").rglob("*.cpp"))

write(
    KB / "03_rendering_pipeline.txt",
    header("Rendering Pipeline") + f"""
Rendering responsibilities:

{chr(10).join(f"- {p.relative_to(SRC)}" for p in render_files)}

Rendering loop:
- Interpolates simulation state
- Never advances simulation
"""
)

# ---- Threading ----
write(
    KB / "04_threading_and_timing.txt",
    header("Threading and Timing") + """
Threads:
- Main thread: event handling + rendering
- Simulation thread: physics update

Rules:
- No shared mutable state without synchronization
- Simulation publishes immutable snapshots
"""
)

# ---- Interfaces ----
headers = list(SRC.rglob("*.h"))

write(
    KB / "05_core_interfaces.txt",
    header("Core Interfaces") + f"""
Public headers:

{chr(10).join(f"- {h.relative_to(SRC)}" for h in headers)}

Interfaces should:
- Express ownership clearly
- Avoid leaking implementation details
"""
)

# ---- Glossary ----
write(
    KB / "99_glossary.txt",
    header("Glossary") + """
Simulation step:
A fixed-duration physics update.

Frame:
A rendered image, may interpolate simulation state.

Snapshot:
Immutable simulation state handed to renderer.
"""
)

print("Knowledge Base regenerated.")

