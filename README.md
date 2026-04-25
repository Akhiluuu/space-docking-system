# DtProjectScenarios: Stewart Platform Leg Joint Viewer

This project is a standalone, high-fidelity 3D visualization and simulation tool for a 6-DOF Stewart Platform. It is designed to simulate and visualize spacecraft docking maneuvers, focusing on the mechanical response of the platform to contact forces.

## 📺 Video Simulation
[Watch the Video Simulation Here](https://drive.google.com/file/d/1jWFvHD1v2-1ksuHLuicHKOQ9wc0emPQD/view?usp=sharing)

---

## 🛠 Project Overview

The **Leg Joint Viewer** provides a real-time look at how the Stewart Platform's six telescopic legs respond to docking scenarios. Unlike a simple animation, this tool uses a rigid-body physics engine to calculate the platform's motion based on force events defined in XML scenarios.

### Core Components
- **Visualization Engine**: OpenGL 3.3 core profile with GLM for transformations.
- **Mesh Loading**: Assimp is used to load high-detail OBJ models for the platform, base, and legs.
- **Interactive Dashboard**: Dear ImGui provides a real-time control panel to load models, edit forces, and start/stop playback.
- **Physics Engine**: A custom force-to-motion integrator for 6-DOF rigid body dynamics.

---

## 📂 Project Structure

- `src/`: C++ source logic (Integration, XML parsing, Rendering, and the Mars-specific `scenarios_precombined.cpp`).
- `include/`: Header definitions and the data model (including `BulletSimulation.h`).
- `assets/models/`: High-resolution OBJ models for the Stewart Platform.
- `scenarios/`: XML files defining the docking missions (includes Mars-specific `_new.xml` files).

---

## 🚀 How the Visualization Works

### 1. The Physics Model
The platform's motion is driven by a two-part system:
1.  **Scripted Approach**: The platform follows a predefined path toward the target.
2.  **Force Response**: When "Contact" occurs, the simulation calculates the translational and rotational response using:
    - **Linear**: $F = m \cdot a$ (Force equals mass times acceleration)
    - **Angular**: $\tau = I \cdot \alpha$ (Torque equals inertia times angular acceleration)

### 2. Contact Detection & Torque
The platform features a **hexagonal sensor layout**. Each sensor measures compression forces ($f_x, f_y, f_z$). 
- **Torque Calculation**: Torque is calculated as $\tau = r \times F$ for each sensor location. This allows off-center impacts to cause the platform to tilt or rotate naturally.
- **Damping**: A small damping factor is applied between events to simulate a zero-g style drift and stabilization.

### 3. Leg Geometry (Inverse Kinematics)
The legs are not independently animated. Instead, the system uses **Inverse Kinematics**:
- The platform pose (position and rotation) is determined by the physics engine.
- For each frame, the system recomputes the distance between the fixed base anchors and the moving top anchors.
- The 3D leg models are then dynamically rotated and scaled (separating the casing and the rod) to match these geometric endpoints.

### 4. Mechanical Validation
The viewer continuously validates the "reach" of the platform. If any leg length falls outside the mechanical limits (**0.15m to 0.258m**), the system flags the pose, ensuring the simulation stays within physical constraints.

---

## 📑 Scenarios

Scenarios are defined in XML and can be modified via the **Event Editor** in the dashboard.

- **Normal Docking**: A centered, controlled approach with symmetric forces.
- **Off-Center Contact**: An asymmetric impact that tests the platform's ability to absorb lateral forces and rotational torque.
- **Bounce-Back**: Simulates high-impact scenarios where the platform recoils from the target.

---

## ⚙️ Build and Run Instructions

### Requirements
- **C++20** compatible compiler (MSVC 2019+, GCC 10+, or Clang 12+)
- **CMake 3.20+**
- **Python 3** (Used for GLAD header generation)

### Steps to Build
1.  Open a terminal (PowerShell or CMD) in the `DtProjectScenarios` directory.
2.  Create a build directory and configure the project:
    ```powershell
    mkdir build
    cd build
    cmake ..
    ```
3.  Compile the project:
    ```powershell
    cmake --build . --config Release
    ```

### How to Run
Once compiled, you can run the executable from the build folder:
```powershell
.\Release\DtProjectScenarios.exe
```

### Quick Start Guide
1.  **Select Scenario**: Choose a mission (e.g., Normal or Off-Center) from the dashboard.
2.  **Load Model**: Click **LOAD MODEL** to initialize the meshes and XML data.
3.  **Start Viz**: Click **START VISUALIZATION** to begin the physics simulation.
4.  **Edit**: Change forces in the **Event Editor** and click **SAVE & RELOAD** to see the updated response.

---

## 🎮 Controls

- **Mouse Left-Click**: Rotate Camera
- **Mouse Scroll**: Zoom In/Out
- **LOAD MODEL**: Loads the selected XML scenario.
- **START VISUALIZATION**: Begins the simulation playback.
- **SAVE & RELOAD**: Saves edited forces from the dashboard back to the XML and restarts the sim.
