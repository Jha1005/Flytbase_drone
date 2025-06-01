"""
Main executable for the UAV Strategic Deconfliction System.

This script demonstrates the system's functionality by:
1. Defining sample primary drone mission and simulated flight schedules.
2. Running the deconfliction checks (spatial and temporal).
3. Printing the results to the console.
4. Generating visualizations of the missions and conflicts.
"""
import os

# Ensure the src directory is in PYTHONPATH for imports if running from project root
# This is often handled by IDEs or by running as a module (python -m flytbase_deconfliction.main)
# For direct script execution from the project root, this helps:
import sys
# Assuming 'flytbase_deconfliction' is the project root directory name.
# And this 'main.py' is inside 'flytbase_deconfliction' directory.
# If 'main.py' is in 'flytbase_deconfliction/src', then path should be '..'
# Current structure: flytbase_deconfliction/main.py and flytbase_deconfliction/src/
# So, we need to add 'flytbase_deconfliction' (project root) to path to find 'src' package.
# Or, more simply, if 'main.py' is at the same level as the 'src' folder:
# sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
# If main.py is inside flytbase_deconfliction folder, and src is flytbase_deconfliction/src
# then the package is src.
# Let's assume main.py is in the root of the project 'flytbase_deconfliction'
# and 'src' is a package within it.

try:
    from src.data_structures import MissionWaypoint, TimeWindow, DroneMission, SimulatedFlight
    from src.conflict_checker import run_deconfliction_check, DEFAULT_SAFETY_BUFFER_DISTANCE, DEFAULT_SAFETY_BUFFER_TIME
    from src.visualization import plot_missions_2d, plot_missions_3d, generate_conflict_animation_2d
except ImportError as e:
    print(f"Error importing modules: {e}")
    print("Please ensure that the script is run from the project root directory (flytbase_deconfliction),")
    print("or that the 'src' directory is correctly in the PYTHONPATH.")
    print("Example: python main.py (if in flytbase_deconfliction directory)")
    print("Or: python -m src.main (if main.py was inside src and src was a package)")
    sys.exit(1)

# --- Configuration ---
OUTPUT_DIR = "output_visualizations"
os.makedirs(OUTPUT_DIR, exist_ok=True)

# Safety parameters (can be overridden if needed)
SAFETY_DISTANCE = DEFAULT_SAFETY_BUFFER_DISTANCE  # meters
SAFETY_TIME_BUFFER = DEFAULT_SAFETY_BUFFER_TIME    # seconds


def create_scenario_1_conflict():
    """
    Scenario 1: A clear case of spatial and temporal conflict.
    Primary drone flies straight. Simulated drone crosses its path at the same time.
    """
    print("\n--- SCENARIO 1: Clear Conflict ---")
    # Primary Drone Mission
    pm_waypoints = [
        MissionWaypoint(x=0, y=50, z=10),
        MissionWaypoint(x=200, y=50, z=10)
    ]
    # Total distance = 200m. If time window is 0-20s, speed = 10m/s.
    # Segment 1 (0,50) to (200,50) is traversed from T=0 to T=20.
    pm_time_window = TimeWindow(start_time=0, end_time=20) # 20 seconds total
    primary_mission = DroneMission("PM_Conflict", pm_waypoints, pm_time_window)

    # Simulated Flights
    # SF1 crosses PM's path around x=100, y=50.
    # PM reaches x=100 at T=10s (halfway through its 20s mission).
    # SF1 segment (100,0) to (100,100)
    # If SF1 is at (100,50) at T=10s, it's a conflict.
    sim_flight1_traj = [
        (MissionWaypoint(x=100, y=0, z=10), 5),   # Arrives at (100,0) at T=5s
        (MissionWaypoint(x=100, y=100, z=10), 15) # Arrives at (100,100) at T=15s
                                                 # So, at (100,50) at T=10s.
    ]
    sim_flight1 = SimulatedFlight("SF_Cross_Conflict", sim_flight1_traj)

    simulated_flights = [sim_flight1]

    # Run Deconfliction
    result = run_deconfliction_check(primary_mission, simulated_flights, SAFETY_DISTANCE, SAFETY_TIME_BUFFER)
    print(f"Deconfliction Status: {result.status}")
    print("Explanation:")
    print(result.explanation)

    # Visualize
    plot_missions_2d(
        primary_mission, simulated_flights, result,
        title="Scenario 1: Clear Conflict (2D)",
        output_filename=os.path.join(OUTPUT_DIR, "scenario1_conflict_2d.png")
    )
    plot_missions_3d(
        primary_mission, simulated_flights, result,
        title="Scenario 1: Clear Conflict (3D)",
        output_filename=os.path.join(OUTPUT_DIR, "scenario1_conflict_3d.png")
    )
    if result.status != "clear": # Animate if there's something interesting
        generate_conflict_animation_2d(
            primary_mission, simulated_flights, result,
            output_filename=os.path.join(OUTPUT_DIR, "scenario1_conflict_animation.mp4"),
            safety_buffer_dist=SAFETY_DISTANCE
        )
    return result

def create_scenario_2_no_conflict():
    """
    Scenario 2: A conflict-free mission. Paths are well separated.
    """
    print("\n--- SCENARIO 2: No Conflict (Clear) ---")
    # Primary Drone Mission
    pm_waypoints = [
        MissionWaypoint(x=0, y=0, z=20),
        MissionWaypoint(x=50, y=50, z=20),
        MissionWaypoint(x=100, y=0, z=20)
    ]
    pm_time_window = TimeWindow(start_time=0, end_time=100) # 100 seconds
    primary_mission = DroneMission("PM_Clear", pm_waypoints, pm_time_window)

    # Simulated Flights (far away)
    sim_flight1_traj = [
        (MissionWaypoint(x=500, y=500, z=30), 0),
        (MissionWaypoint(x=600, y=550, z=30), 50)
    ]
    sim_flight1 = SimulatedFlight("SF_FarAway", sim_flight1_traj)

    sim_flight2_traj = [
        (MissionWaypoint(x=-100, y=-50, z=10), 10),
        (MissionWaypoint(x=-150, y=-100, z=10), 60)
    ]
    sim_flight2 = SimulatedFlight("SF_OtherSide", sim_flight2_traj)
    
    simulated_flights = [sim_flight1, sim_flight2]

    # Run Deconfliction
    result = run_deconfliction_check(primary_mission, simulated_flights, SAFETY_DISTANCE, SAFETY_TIME_BUFFER)
    print(f"Deconfliction Status: {result.status}")
    print("Explanation:")
    print(result.explanation)

    # Visualize
    plot_missions_2d(
        primary_mission, simulated_flights, result,
        title="Scenario 2: No Conflict (2D)",
        output_filename=os.path.join(OUTPUT_DIR, "scenario2_no_conflict_2d.png")
    )
    plot_missions_3d(
        primary_mission, simulated_flights, result,
        title="Scenario 2: No Conflict (3D)",
        output_filename=os.path.join(OUTPUT_DIR, "scenario2_no_conflict_3d.png")
    )
    # Optional: Animate even clear scenarios if desired
    generate_conflict_animation_2d(
        primary_mission, simulated_flights, result,
        output_filename=os.path.join(OUTPUT_DIR, "scenario2_no_conflict_animation.mp4"),
        safety_buffer_dist=SAFETY_DISTANCE
    )
    return result

def create_scenario_3_spatial_proximity_no_temporal_conflict():
    """
    Scenario 3: Paths are spatially close, but drones pass at different times.
    Should result in "clear_with_spatial_warning" or similar if logic supports it,
    otherwise "clear" but visualization will show proximity.
    """
    print("\n--- SCENARIO 3: Spatial Proximity, No Temporal Conflict ---")
    # Primary Drone Mission
    pm_waypoints = [
        MissionWaypoint(x=0, y=50, z=10),
        MissionWaypoint(x=200, y=50, z=10) # Traversed T=0 to T=20 (speed 10m/s)
    ]
    pm_time_window = TimeWindow(start_time=0, end_time=20)
    primary_mission = DroneMission("PM_SpatialProx", pm_waypoints, pm_time_window)

    # Simulated Flight - crosses same point (100,50) but at a much later time
    # PM is at (100,50) at T=10s.
    sim_flight1_traj = [
        (MissionWaypoint(x=100, y=0, z=10), 50),   # Arrives at (100,0) at T=50s
        (MissionWaypoint(x=100, y=100, z=10), 60)  # Arrives at (100,100) at T=60s
                                                  # So, at (100,50) at T=55s. No temporal overlap.
    ]
    sim_flight1 = SimulatedFlight("SF_Cross_Late", sim_flight1_traj)
    simulated_flights = [sim_flight1]

    result = run_deconfliction_check(primary_mission, simulated_flights, SAFETY_DISTANCE, SAFETY_TIME_BUFFER)
    print(f"Deconfliction Status: {result.status}")
    print("Explanation:")
    print(result.explanation)

    plot_missions_2d(
        primary_mission, simulated_flights, result,
        title="Scenario 3: Spatial Proximity, No Temporal Conflict (2D)",
        output_filename=os.path.join(OUTPUT_DIR, "scenario3_spatial_not_temporal_2d.png")
    )
    plot_missions_3d(
        primary_mission, simulated_flights, result,
        title="Scenario 3: Spatial Proximity, No Temporal Conflict (3D)",
        output_filename=os.path.join(OUTPUT_DIR, "scenario3_spatial_not_temporal_3d.png")
    )
    generate_conflict_animation_2d(
        primary_mission, simulated_flights, result,
        output_filename=os.path.join(OUTPUT_DIR, "scenario3_spatial_not_temporal_animation.mp4"),
        safety_buffer_dist=SAFETY_DISTANCE
    )
    return result

def create_scenario_4_multiple_sim_drones():
    """
    Scenario 4: Multiple simulated drones, some conflicting, some not.
    """
    print("\n--- SCENARIO 4: Multiple Simulated Drones, Mixed Results ---")
    # Primary Drone Mission
    pm_waypoints = [
        MissionWaypoint(x=0, y=0, z=5),
        MissionWaypoint(x=150, y=150, z=5), # Diagonal path
        MissionWaypoint(x=300, y=0, z=5)    # Another diagonal
    ]
    # Total dist approx 150*sqrt(2) + 150*sqrt(2) = 300*sqrt(2) approx 424m
    # Time window 0-60s. Speed approx 7 m/s.
    # Midpoint (150,150) reached around T=30s.
    pm_time_window = TimeWindow(start_time=0, end_time=60)
    primary_mission = DroneMission("PM_MultiSim", pm_waypoints, pm_time_window)

    # SF1: Conflicts with first leg of PM
    # PM first leg (0,0) to (150,150) takes ~30s. (75,75) at T=~15s.
    sf1_traj = [
        (MissionWaypoint(x=75, y=50, z=5), 10),
        (MissionWaypoint(x=75, y=100, z=5), 20) # Crosses PM path near (75,75) around T=15s
    ]
    sf1 = SimulatedFlight("SF_Multi_Conflict1", sf1_traj)

    # SF2: Clear, different area
    sf2_traj = [
        (MissionWaypoint(x=-50, y=200, z=10), 0),
        (MissionWaypoint(x=50, y=200, z=10), 30)
    ]
    sf2 = SimulatedFlight("SF_Multi_Clear", sf2_traj)

    # SF3: Spatially close to PM's second leg, but temporally separated
    # PM second leg (150,150) to (300,0) takes ~30s (from T=~30 to T=~60).
    # Midpoint of second leg (225, 75) at T=~45s.
    sf3_traj = [
        (MissionWaypoint(x=225, y=50, z=5), 5),  # Too early
        (MissionWaypoint(x=225, y=100, z=5), 15) # Too early
    ]
    sf3 = SimulatedFlight("SF_Multi_SpatialOnly", sf3_traj)

    simulated_flights = [sf1, sf2, sf3]

    result = run_deconfliction_check(primary_mission, simulated_flights, SAFETY_DISTANCE, SAFETY_TIME_BUFFER)
    print(f"Deconfliction Status: {result.status}")
    print("Explanation:")
    print(result.explanation)

    plot_missions_2d(
        primary_mission, simulated_flights, result,
        title="Scenario 4: Multiple Drones, Mixed (2D)",
        output_filename=os.path.join(OUTPUT_DIR, "scenario4_multi_mixed_2d.png")
    )
    plot_missions_3d(
        primary_mission, simulated_flights, result,
        title="Scenario 4: Multiple Drones, Mixed (3D)",
        output_filename=os.path.join(OUTPUT_DIR, "scenario4_multi_mixed_3d.png")
    )
    generate_conflict_animation_2d(
        primary_mission, simulated_flights, result,
        output_filename=os.path.join(OUTPUT_DIR, "scenario4_multi_mixed_animation.mp4"),
        safety_buffer_dist=SAFETY_DISTANCE
    )
    return result


if __name__ == "__main__":
    print("Starting UAV Strategic Deconfliction System Demo")
    print(f"Output visualizations will be saved to: {os.path.abspath(OUTPUT_DIR)}")
    print(f"Using Safety Distance: {SAFETY_DISTANCE}m, Safety Time Buffer: {SAFETY_TIME_BUFFER}s")

    # Run scenarios
    res1 = create_scenario_1_conflict()
    res2 = create_scenario_2_no_conflict()
    res3 = create_scenario_3_spatial_proximity_no_temporal_conflict()
    res4 = create_scenario_4_multiple_sim_drones()

    print("\n--- All Scenarios Processed ---")
    print(f"Scenario 1 ('PM_Conflict') Status: {res1.status}")
    print(f"Scenario 2 ('PM_Clear') Status: {res2.status}")
    print(f"Scenario 3 ('PM_SpatialProx') Status: {res3.status}")
    print(f"Scenario 4 ('PM_MultiSim') Status: {res4.status}")

    print("\nDemo finished. Check the 'output_visualizations' directory for plots and animations.")
    print("Note: Animation generation requires FFmpeg. If animations are missing, FFmpeg might not be installed or accessible in PATH.")