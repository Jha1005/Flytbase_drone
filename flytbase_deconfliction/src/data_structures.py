"""
Defines the data structures for representing drone missions and flight paths.
"""
from typing import List, Tuple, NamedTuple

# Using simple tuples for coordinates for now, can be extended to a class if more attributes are needed.
# For 2D: (x, y)
# For 3D (Extra Credit): (x, y, z)
Waypoint = Tuple[float, float]  # Defaulting to 2D for now
Waypoint3D = Tuple[float, float, float]

class MissionWaypoint(NamedTuple):
    """Represents a single waypoint in a mission with its spatial coordinates."""
    x: float
    y: float
    z: float = 0.0  # Optional altitude, defaults to 0 for 2D

class TimeWindow(NamedTuple):
    """Represents a time window with a start and end time."""
    start_time: float
    end_time: float

class DroneMission(NamedTuple):
    """
    Represents the primary drone's mission.
    """
    mission_id: str
    waypoints: List[MissionWaypoint]
    time_window: TimeWindow # Overall time window for the entire mission

class SimulatedFlight(NamedTuple):
    """
    Represents a simulated flight path of another drone.
    Each waypoint in a simulated flight has an associated arrival time.
    """
    flight_id: str
    # List of (Waypoint, arrival_time_at_waypoint)
    # This structure implies the drone moves from one waypoint to the next, arriving at the specified time.
    # The time to reach the first waypoint from its origin is its arrival_time.
    trajectory: List[Tuple[MissionWaypoint, float]] # (Waypoint, time_at_waypoint)

    # Optional: Define an overall operational window for the simulated flight if needed
    # operational_window: TimeWindow

# Example Usage (will be removed or moved to a test/example file later)
if __name__ == '__main__':
    # Primary Drone Mission Example
    mission_waypoints = [
        MissionWaypoint(x=0, y=0),
        MissionWaypoint(x=100, y=0),
        MissionWaypoint(x=100, y=100),
        MissionWaypoint(x=0, y=100),
        MissionWaypoint(x=0, y=0)
    ]
    mission_time_window = TimeWindow(start_time=0, end_time=3600) # Mission must be completed in 1 hour
    primary_mission = DroneMission(
        mission_id="primary_drone_001",
        waypoints=mission_waypoints,
        time_window=mission_time_window
    )
    print(f"Primary Mission: {primary_mission}")

    # Simulated Flight Example
    sim_flight_1_trajectory = [
        (MissionWaypoint(x=50, y=-50), 600),      # Arrives at (50, -50) at T=600s
        (MissionWaypoint(x=50, y=50), 1200),      # Arrives at (50, 50) at T=1200s
        (MissionWaypoint(x=150, y=50), 1800)      # Arrives at (150, 50) at T=1800s
    ]
    simulated_flight_1 = SimulatedFlight(
        flight_id="sim_drone_A",
        trajectory=sim_flight_1_trajectory
    )
    print(f"Simulated Flight 1: {simulated_flight_1}")

    # For 3D (Extra Credit)
    mission_waypoints_3d = [
        MissionWaypoint(x=0, y=0, z=10),
        MissionWaypoint(x=100, y=0, z=10),
        MissionWaypoint(x=100, y=100, z=15),
        MissionWaypoint(x=0, y=100, z=15),
        MissionWaypoint(x=0, y=0, z=10)
    ]
    primary_mission_3d = DroneMission(
        mission_id="primary_drone_3D_001",
        waypoints=mission_waypoints_3d,
        time_window=mission_time_window
    )
    print(f"Primary Mission 3D: {primary_mission_3d}")

    sim_flight_2_trajectory_3d = [
        (MissionWaypoint(x=50, y=-50, z=12), 600),
        (MissionWaypoint(x=50, y=50, z=12), 1200),
        (MissionWaypoint(x=150, y=50, z=18), 1800)
    ]
    simulated_flight_2 = SimulatedFlight(
        flight_id="sim_drone_B_3D",
        trajectory=sim_flight_2_trajectory_3d
    )
    print(f"Simulated Flight 2 (3D): {simulated_flight_2}")