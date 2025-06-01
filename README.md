# UAV Strategic Deconfliction System

This project implements a strategic deconfliction system for Unmanned Aerial Vehicles (UAVs) operating in shared airspace. It verifies whether a primary drone's planned waypoint mission is safe to execute by checking for conflicts in space and time against the simulated flight paths of multiple other drones.

## Project Structure

```
flytbase_deconfliction/
├── main.py                 # Main executable script to run scenarios
├── output_visualizations/  # Directory for generated plots and animations (created on run)
├── src/
│   ├── __init__.py         # Makes 'src' a Python package
│   ├── data_structures.py  # Defines data models for missions, waypoints, etc.
│   ├── conflict_checker.py # Core logic for spatial and temporal conflict detection
│   └── visualization.py    # Generates plots and animations of missions and conflicts
├── README.md               # This file
└── Reflection.md           # Document discussing design, AI use, and scalability
```

## Features

*   **Spatial Conflict Check:** Validates if the primary mission's path intersects with any other drone’s trajectory within a defined safety buffer.
*   **Temporal Conflict Check:** Ensures no other drone is present in the same spatial area during overlapping time segments, considering the primary mission's overall time window and an assumed average speed.
*   **Conflict Explanation:** Provides details about detected conflicts, including location and timing.
*   **Query Interface:** A Python function (`run_deconfliction_check` in `conflict_checker.py`) accepts the primary drone’s mission and returns a status and conflict details.
*   **Simulation & Visualization:**
    *   Generates 2D and basic 3D plots depicting drone trajectories and highlighted conflicts.
    *   Generates 2D animations for visualizing scenarios over time (saved as MP4 if FFmpeg is available, otherwise falls back to GIF).

## Setup

### Prerequisites

*   Python 3.7+
*   `pip` (Python package installer)
*   `ffmpeg` (Recommended for saving animations as higher quality `.mp4` files. If not available, animations will be saved as `.gif` files).
    *   Download and install from [ffmpeg.org](https://ffmpeg.org/download.html).
    *   Ensure `ffmpeg` is added to your system's PATH for MP4 support.

### Installation

1.  **Clone the repository (or download the files):**
    ```bash
    # If it were a git repository:
    # git clone <repository_url>
    # cd flytbase_deconfliction
    ```
    For now, ensure you have the `flytbase_deconfliction` directory with `main.py` and the `src/` subdirectory.

2.  **Install required Python packages:**
    The primary dependency is `matplotlib` for visualizations.
    Navigate to the project's root directory (`flytbase_deconfliction`) in your terminal and run:
    ```bash
    pip install matplotlib
    ```

## Running the System

The `main.py` script is configured to run several predefined scenarios demonstrating different conflict situations (conflict, no conflict, spatial proximity without temporal conflict, multiple drones).

To execute the demo:

1.  Navigate to the root directory of the project (`flytbase_deconfliction`) in your terminal.
2.  Run the main script:
    ```bash
    python main.py
    ```

### Output

*   **Console Output:** The script will print the status and detailed explanations for each deconfliction scenario to the console.
*   **Visualizations:** Plots (`.png`) and animations (either `.mp4` or `.gif`) for each scenario will be saved in the `output_visualizations/` directory, which will be created automatically if it doesn't exist.

    *   `scenario1_conflict_2d.png`, `scenario1_conflict_3d.png`, `scenario1_conflict_animation.(mp4|gif)`
    *   `scenario2_no_conflict_2d.png`, `scenario2_no_conflict_3d.png`, `scenario2_no_conflict_animation.(mp4|gif)`
    *   `scenario3_spatial_not_temporal_2d.png`, `scenario3_spatial_not_temporal_3d.png`, `scenario3_spatial_not_temporal_animation.(mp4|gif)`
    *   `scenario4_multi_mixed_2d.png`, `scenario4_multi_mixed_3d.png`, `scenario4_multi_mixed_animation.(mp4|gif)`

**Note on Animations:** The script will attempt to save animations as `.mp4` files using FFmpeg. If FFmpeg is not correctly installed or not found in the system PATH, it will fall back to saving the animations as `.gif` files using the Pillow library. Static plots (`.png`) will always be generated.

## How it Works

1.  **Data Structures (`src/data_structures.py`):** Defines how drone waypoints, mission plans (primary drone), and simulated flight trajectories (other drones with timed waypoints) are represented.
2.  **Conflict Checker (`src/conflict_checker.py`):**
    *   `perform_spatial_check`: Calculates the minimum distance between segments of the primary mission and simulated flights. If this distance is less than a predefined safety buffer, a spatial proximity is flagged.
    *   `check_temporal_conflict`: For each spatial proximity, this function estimates the time intervals during which both the primary and simulated drones occupy their respective conflicting segments.
        *   The primary drone's timing is estimated by assuming it flies at a constant average speed to complete its entire mission within the given overall time window.
        *   Simulated drones have explicit timing for their waypoints.
        *   If these time intervals overlap (considering a safety time buffer), a temporal conflict is confirmed.
    *   `run_deconfliction_check`: Orchestrates the spatial and temporal checks and compiles the results.
3.  **Visualization (`src/visualization.py`):** Uses `matplotlib` to create:
    *   `plot_missions_2d`: Static 2D plots showing all trajectories, highlighting waypoints, and marking spatial and temporal conflict zones.
    *   `plot_missions_3d`: Basic static 3D plots.
    *   `generate_conflict_animation_2d`: Dynamic 2D animations showing drone movements over time, with visual cues for conflicts. Attempts to save as MP4, falls back to GIF if FFmpeg is unavailable.
4.  **Main Script (`main.py`):** Sets up different scenarios with sample data, calls the deconfliction service, and then invokes the visualization functions to output the results.

## Extending the System

*   **Input Data:** Modify `main.py` to load drone mission data from files (e.g., JSON, CSV) instead of using hardcoded examples.
*   **Conflict Resolution:** Currently, the system only detects conflicts. Future work could involve suggesting alternative paths or timings.
*   **Advanced Visualization:** Enhance 3D/4D visualizations for better spatio-temporal understanding.
*   **Real-time Capabilities:** Integrate with real-time data feeds for dynamic deconfliction.