"""
Core logic for checking spatial and temporal conflicts between drone missions.
"""
import math
from typing import List, Tuple, Optional, Dict, Any, NamedTuple
from .data_structures import MissionWaypoint, DroneMission, SimulatedFlight, TimeWindow

# --- Helper Geometric Functions ---

def distance_between_points(p1: MissionWaypoint, p2: MissionWaypoint) -> float:
    """Calculates the Euclidean distance between two waypoints."""
    return math.sqrt((p1.x - p2.x)**2 + (p1.y - p2.y)**2 + (p1.z - p2.z)**2)

def get_line_segment_from_waypoints(wp1: MissionWaypoint, wp2: MissionWaypoint) -> Tuple[MissionWaypoint, MissionWaypoint]:
    """Returns a line segment defined by two waypoints."""
    return (wp1, wp2)

def midpoint_3d(wp1: MissionWaypoint, wp2: MissionWaypoint) -> MissionWaypoint:
    """Calculates the midpoint of the segment defined by two waypoints."""
    return MissionWaypoint(
        x=(wp1.x + wp2.x) / 2,
        y=(wp1.y + wp2.y) / 2,
        z=(wp1.z + wp2.z) / 2
    )

# --- Spatial Conflict Detection ---

MIN_SEPARATION_DISTANCE = 10.0  # meters, configurable safety buffer

class SpatialConflict(NamedTuple):
    """Details of a detected spatial conflict."""
    primary_mission_segment: Tuple[MissionWaypoint, MissionWaypoint]
    conflicting_flight_id: str
    conflicting_flight_segment: Tuple[MissionWaypoint, MissionWaypoint]
    # Potentially add: estimated_conflict_point, distance_at_closest_approach

def check_segment_intersection(
    seg1_p1: MissionWaypoint, seg1_p2: MissionWaypoint,
    seg2_p1: MissionWaypoint, seg2_p2: MissionWaypoint,
    buffer: float
) -> bool:
    """
    Checks if two line segments (drone paths) intersect or come within a buffer distance.
    This is a simplified check. A more robust method would involve checking distance
    between line segments. For now, we can approximate by checking if endpoints of one
    segment are too close to the other segment, or if segments cross.

    A common approach for line segment intersection:
    1. Check orientation of triplets (p1, q1, p2), (p1, q1, q2), (p2, q2, p1), (p2, q2, q1).
    2. Handle collinear cases.

    For simplicity in this initial version, we can use a less precise but faster check:
    - Calculate the shortest distance from each endpoint of segment1 to segment2.
    - Calculate the shortest distance from each endpoint of segment2 to segment1.
    - If any of these distances are less than the buffer, consider it a potential conflict.
    - This doesn't perfectly handle segments that pass close by without endpoints being close,
      or segments that cross. A full segment-to-segment distance algorithm is more accurate.

    Let's implement a placeholder for now and refine it.
    A simple check: if the minimum distance between any two endpoints of the
    two segments is less than the buffer, or if the segments are very close.
    This is a very rough approximation.
    """
    # This is a placeholder for a proper segment intersection or distance check.
    # For a more accurate check, one would implement a segment-to-segment distance algorithm.
    # For example, see: http://geomalgorithms.com/a07-_distance.html (dist3D_Segment_to_Segment)

    # Simplified check: if any endpoint of one segment is too close to an endpoint of the other.
    # This is NOT a full intersection check but a starting point.
    points_to_check = [
        (seg1_p1, seg2_p1), (seg1_p1, seg2_p2),
        (seg1_p2, seg2_p1), (seg1_p2, seg2_p2)
    ]
    for p1, p2 in points_to_check:
        if distance_between_points(p1, p2) < buffer:
            return True # Simplified: endpoints are too close

    # A more robust check would involve projecting points onto lines and checking segment bounds.
    # For now, this is a placeholder.
    # TODO: Implement a robust line segment intersection or minimum distance check.
    # A common method is to check if the bounding boxes overlap first, then do a more precise check.
    # For 3D, this becomes more complex.

    # Let's consider a slightly more involved (but still not perfect) approach:
    # Check distance from endpoints of seg1 to line defined by seg2, and vice-versa,
    # ensuring the closest point on the line lies within the segment.

    if is_point_near_segment(seg1_p1, seg2_p1, seg2_p2, buffer) or \
       is_point_near_segment(seg1_p2, seg2_p1, seg2_p2, buffer) or \
       is_point_near_segment(seg2_p1, seg1_p1, seg1_p2, buffer) or \
       is_point_near_segment(seg2_p2, seg1_p1, seg1_p2, buffer):
        return True

    # This still doesn't cover all crossing scenarios without endpoint proximity.
    # A full Separating Axis Theorem (SAT) based approach or segment distance algorithm is needed for robustness.
    return False


def closest_point_on_segment(p: MissionWaypoint, a: MissionWaypoint, b: MissionWaypoint) -> MissionWaypoint:
    """Finds the point on segment AB closest to point P."""
    ap = (p.x - a.x, p.y - a.y, p.z - a.z)
    ab = (b.x - a.x, b.y - a.y, b.z - a.z)

    ab_squared = ab[0]**2 + ab[1]**2 + ab[2]**2
    if ab_squared == 0: # a and b are the same point
        return a

    ap_dot_ab = ap[0]*ab[0] + ap[1]*ab[1] + ap[2]*ab[2]
    t = ap_dot_ab / ab_squared

    if t < 0:
        closest = a
    elif t > 1:
        closest = b
    else:
        closest = MissionWaypoint(a.x + t * ab[0], a.y + t * ab[1], a.z + t * ab[2])
    return closest

def is_point_near_segment(p: MissionWaypoint, seg_p1: MissionWaypoint, seg_p2: MissionWaypoint, buffer: float) -> bool:
    """Checks if point p is within buffer distance of segment (seg_p1, seg_p2)."""
    closest = closest_point_on_segment(p, seg_p1, seg_p2)
    return distance_between_points(p, closest) < buffer


def perform_spatial_check(
    primary_mission: DroneMission,
    simulated_flights: List[SimulatedFlight],
    safety_buffer: float = MIN_SEPARATION_DISTANCE
) -> List[SpatialConflict]:
    """
    Checks the primary drone's mission path for spatial conflicts with simulated flights.
    A conflict occurs if any segment of the primary mission comes within `safety_buffer`
    of any segment of a simulated flight's trajectory.
    """
    spatial_conflicts: List[SpatialConflict] = []

    # Iterate through each segment of the primary mission path
    for i in range(len(primary_mission.waypoints) - 1):
        pm_wp1 = primary_mission.waypoints[i]
        pm_wp2 = primary_mission.waypoints[i+1]
        primary_segment = (pm_wp1, pm_wp2)

        # Compare with each segment of each simulated flight
        for sim_flight in simulated_flights:
            if len(sim_flight.trajectory) < 2: # Need at least two points for a segment
                continue

            for j in range(len(sim_flight.trajectory) - 1):
                # Simulated flight waypoints are (MissionWaypoint, time_at_waypoint)
                sim_wp1_data = sim_flight.trajectory[j]
                sim_wp2_data = sim_flight.trajectory[j+1]
                
                sim_wp1 = sim_wp1_data[0] # MissionWaypoint object
                sim_wp2 = sim_wp2_data[0] # MissionWaypoint object
                sim_segment = (sim_wp1, sim_wp2)

                # Enhanced proximity check:
                # 1. Check endpoints of seg1 against seg2
                # 2. Check endpoints of seg2 against seg1
                # 3. Check midpoint of seg1 against seg2
                # 4. Check midpoint of seg2 against seg1
                # TODO: Still recommend a robust 3D segment-to-segment distance check for production.

                conflict_detected = False
                # Check endpoints
                if is_point_near_segment(pm_wp1, sim_wp1, sim_wp2, safety_buffer) or \
                   is_point_near_segment(pm_wp2, sim_wp1, sim_wp2, safety_buffer) or \
                   is_point_near_segment(sim_wp1, pm_wp1, pm_wp2, safety_buffer) or \
                   is_point_near_segment(sim_wp2, pm_wp1, pm_wp2, safety_buffer):
                    conflict_detected = True
                
                # Check midpoints if no conflict detected yet by endpoints
                if not conflict_detected:
                    pm_midpoint = midpoint_3d(pm_wp1, pm_wp2)
                    sim_midpoint = midpoint_3d(sim_wp1, sim_wp2)

                    if is_point_near_segment(pm_midpoint, sim_wp1, sim_wp2, safety_buffer) or \
                       is_point_near_segment(sim_midpoint, pm_wp1, pm_wp2, safety_buffer):
                        conflict_detected = True
                
                if conflict_detected:
                    conflict = SpatialConflict(
                        primary_mission_segment=primary_segment,
                        conflicting_flight_id=sim_flight.flight_id,
                        conflicting_flight_segment=sim_segment
                    )
                    # Add only if not already present (based on segments and flight_id)
                    # This simple list append and then set conversion is okay for now.
                    spatial_conflicts.append(conflict)
    
    # Remove duplicate conflicts. Using set for NamedTuples works if they are hashable.
    # A set of tuples might be better for unique conflicts if NamedTuple is hashable
    # For now, this might list multiple "reasons" for the same segment pair conflict.
    return list(set(spatial_conflicts)) # Basic deduplication

# --- Temporal Conflict Detection (Placeholder - to be implemented next) ---

class TemporalConflict(NamedTuple):
    """Details of a detected temporal conflict."""
    conflicting_flight_id: str
    primary_segment_involved: Tuple[MissionWaypoint, MissionWaypoint]
    sim_segment_involved: Tuple[MissionWaypoint, MissionWaypoint]
    time_overlap_start: float
    time_overlap_end: float
    details: str # e.g., "Primary drone at segment X from T1-T2, Sim drone Y at segment Z from T3-T4"

def check_temporal_conflict(
    primary_mission: DroneMission,
    simulated_flights: List[SimulatedFlight],
    spatial_conflicts: List[SpatialConflict],
    safety_buffer_time: float = 30.0 # Default from DEFAULT_SAFETY_BUFFER_TIME
) -> List[TemporalConflict]:
    """
    Checks for temporal conflicts based on identified spatial conflicts.
    Assumes primary drone flies at a constant average speed to complete its mission
    exactly within its specified time_window.
    """
    temporal_conflicts_found: List[TemporalConflict] = []

    if not primary_mission.waypoints or len(primary_mission.waypoints) < 2:
        return [] # No segments for primary mission

    # 1. Calculate total mission distance and average speed for the primary drone
    total_primary_mission_dist = 0.0
    for i in range(len(primary_mission.waypoints) - 1):
        total_primary_mission_dist += distance_between_points(
            primary_mission.waypoints[i], primary_mission.waypoints[i+1]
        )

    mission_duration_allowed = primary_mission.time_window.end_time - primary_mission.time_window.start_time

    avg_speed: float = 0.0
    if mission_duration_allowed <= 0:
        if total_primary_mission_dist > 0: # Impossible mission: positive distance in non-positive time
            # This mission is ill-defined for temporal analysis based on average speed.
            # No temporal conflicts can be meaningfully derived under this assumption.
            return []
        # If dist is 0 and duration is 0 or negative, it's a stationary point "existing" at/before start_time.
        # avg_speed remains 0. Segment durations will be 0.
    elif total_primary_mission_dist > 0 : # Positive distance and positive time
        avg_speed = total_primary_mission_dist / mission_duration_allowed
    # If total_primary_mission_dist is 0 and mission_duration_allowed > 0 (stationary for a period), avg_speed is 0.
    
    # 2. Calculate time intervals for each primary mission segment
    primary_segment_timings = []
    current_pm_time = primary_mission.time_window.start_time
    for i in range(len(primary_mission.waypoints) - 1):
        wp1 = primary_mission.waypoints[i]
        wp2 = primary_mission.waypoints[i+1]
        seg_dist = distance_between_points(wp1, wp2)
        
        seg_duration = 0.0
        if avg_speed > 0:
            seg_duration = seg_dist / avg_speed
        elif seg_dist == 0: # 0-length segment (e.g. drone hovers or waypoints are identical)
            seg_duration = 0.0
        # If avg_speed is 0 and seg_dist > 0: this implies infinite time for this segment.
        # This case should be caught by mission_duration_allowed <= 0 and total_dist > 0 check.
        # If total_dist was 0, then all seg_dist must be 0.

        pm_seg_start_time = current_pm_time
        pm_seg_end_time = current_pm_time + seg_duration
        primary_segment_timings.append({
            "segment": (wp1, wp2),
            "start_time": pm_seg_start_time,
            "end_time": pm_seg_end_time
        })
        current_pm_time = pm_seg_end_time
    
    # Sanity check: Ensure calculated total time does not exceed mission window due to float issues (optional)
    # if total_primary_mission_dist > 0 and abs(current_pm_time - primary_mission.time_window.end_time) > 1e-5: # Epsilon for float comparison
    #     print(f"Warning: Primary mission calculated end time {current_pm_time} differs from window end {primary_mission.time_window.end_time}")


    # 3. Create a lookup for simulated flights by ID
    sim_flights_map = {sf.flight_id: sf for sf in simulated_flights}

    # 4. For each spatial conflict, check for temporal overlap
    for sp_conflict in spatial_conflicts:
        # Find the timing for the primary mission segment involved in the spatial conflict
        pm_timing_info = next((t for t in primary_segment_timings if t["segment"] == sp_conflict.primary_mission_segment), None)
        if not pm_timing_info:
            continue # Should not happen if spatial conflicts are valid

        pm_s_time = pm_timing_info["start_time"]
        pm_e_time = pm_timing_info["end_time"]

        # Find the simulated flight and its conflicting segment's timing
        sim_flight = sim_flights_map.get(sp_conflict.conflicting_flight_id)
        if not sim_flight or len(sim_flight.trajectory) < 2:
            continue

        sim_s_time = -1.0
        sim_e_time = -1.0
        found_sim_segment = False
        for j in range(len(sim_flight.trajectory) - 1):
            sim_wp1_data = sim_flight.trajectory[j]
            sim_wp2_data = sim_flight.trajectory[j+1]
            current_sim_segment_obj = (sim_wp1_data[0], sim_wp2_data[0])

            if current_sim_segment_obj == sp_conflict.conflicting_flight_segment:
                sim_s_time = sim_wp1_data[1]  # Arrival time at start of this sim segment
                sim_e_time = sim_wp2_data[1]  # Arrival time at end of this sim segment
                found_sim_segment = True
                break
        
        if not found_sim_segment:
            continue # Should not happen

        # Check for overlap: max_start_time < min_end_time (considering safety_buffer_time)
        # Primary interval: [pm_s_time, pm_e_time]
        # Sim interval: [sim_s_time, sim_e_time]
        # Conflict if primary overlaps with [sim_s_time - buffer, sim_e_time + buffer]

        buffered_sim_start = sim_s_time - safety_buffer_time
        buffered_sim_end = sim_e_time + safety_buffer_time
        
        # Calculate actual overlap period for reporting (unbuffered)
        actual_overlap_start = max(pm_s_time, sim_s_time)
        actual_overlap_end = min(pm_e_time, sim_e_time)

        # Check for conflict with buffer
        # Conflict if (pm_s_time < buffered_sim_end) and (buffered_sim_start < pm_e_time)
        if pm_s_time < buffered_sim_end and buffered_sim_start < pm_e_time:
            # Ensure the reported actual_overlap is positive if it exists
            if actual_overlap_start < actual_overlap_end:
                details_str = (
                    f"Primary segment {sp_conflict.primary_mission_segment[0]}->{sp_conflict.primary_mission_segment[1]} "
                    f"(time [{pm_s_time:.2f}s, {pm_e_time:.2f}s]) overlaps with "
                    f"{sim_flight.flight_id}'s segment {sp_conflict.conflicting_flight_segment[0]}->{sp_conflict.conflicting_flight_segment[1]} "
                    f"(time [{sim_s_time:.2f}s, {sim_e_time:.2f}s]). "
                    f"Safety buffer: {safety_buffer_time:.2f}s. "
                    f"Actual overlap: [{actual_overlap_start:.2f}s, {actual_overlap_end:.2f}s]."
                )
                temporal_conflicts_found.append(TemporalConflict(
                    conflicting_flight_id=sim_flight.flight_id,
                    primary_segment_involved=sp_conflict.primary_mission_segment,
                    sim_segment_involved=sp_conflict.conflicting_flight_segment,
                    time_overlap_start=actual_overlap_start,
                    time_overlap_end=actual_overlap_end,
                    details=details_str
                ))
    
    return list(set(temporal_conflicts_found)) # Deduplicate


# --- Main Deconfliction Function ---

class DeconflictionResult(NamedTuple):
    status: str # "clear" or "conflict_detected"
    spatial_conflicts: List[SpatialConflict]
    temporal_conflicts: List[TemporalConflict]
    explanation: str

DEFAULT_SAFETY_BUFFER_DISTANCE = 10.0 # meters
DEFAULT_SAFETY_BUFFER_TIME = 30.0 # seconds

def run_deconfliction_check(
    primary_mission: DroneMission,
    simulated_flights: List[SimulatedFlight],
    safety_buffer_distance: float = DEFAULT_SAFETY_BUFFER_DISTANCE,
    safety_buffer_time: float = DEFAULT_SAFETY_BUFFER_TIME # Placeholder for temporal
) -> DeconflictionResult:
    """
    Main interface function to run deconfliction checks.
    """
    explanation_lines = []

    # 1. Perform Spatial Check
    spatial_conflicts = perform_spatial_check(primary_mission, simulated_flights, safety_buffer_distance)

    if spatial_conflicts:
        explanation_lines.append(f"Detected {len(spatial_conflicts)} potential spatial proximity instance(s):")
        for i, sc in enumerate(spatial_conflicts):
            pm_seg_str = f"P_Drone Segment ({sc.primary_mission_segment[0].x:.1f},{sc.primary_mission_segment[0].y:.1f},{sc.primary_mission_segment[0].z:.1f})-({sc.primary_mission_segment[1].x:.1f},{sc.primary_mission_segment[1].y:.1f},{sc.primary_mission_segment[1].z:.1f})"
            sim_seg_str = f"Sim_Drone {sc.conflicting_flight_id} Segment ({sc.conflicting_flight_segment[0].x:.1f},{sc.conflicting_flight_segment[0].y:.1f},{sc.conflicting_flight_segment[0].z:.1f})-({sc.conflicting_flight_segment[1].x:.1f},{sc.conflicting_flight_segment[1].y:.1f},{sc.conflicting_flight_segment[1].z:.1f})"
            explanation_lines.append(f"  Spatial Proximity {i+1}: {pm_seg_str} is close to {sim_seg_str}.")
    else:
        explanation_lines.append("No spatial proximities detected.")
        # If no spatial conflicts, no need for temporal checks based on them.
        return DeconflictionResult(
            status="clear",
            spatial_conflicts=[],
            temporal_conflicts=[],
            explanation="\n".join(explanation_lines) + "\nMission is spatially clear."
        )

    # 2. Perform Temporal Check on identified spatial conflicts
    explanation_lines.append("\nPerforming temporal analysis on spatial proximity areas...")
    temporal_conflicts = check_temporal_conflict(
        primary_mission,
        simulated_flights,
        spatial_conflicts, # Pass the list of spatial conflicts
        safety_buffer_time
    )

    if temporal_conflicts:
        explanation_lines.append(f"Detected {len(temporal_conflicts)} temporal conflict(s):")
        for i, tc in enumerate(temporal_conflicts):
            explanation_lines.append(f"  Temporal Conflict {i+1}: {tc.details}")
    else:
        explanation_lines.append("No temporal conflicts confirmed for the identified spatial proximity areas.")
        explanation_lines.append("This means that while paths are close, they are not occupied simultaneously under the assumed primary drone schedule.")

    # Determine overall status
    final_status = "clear"
    if temporal_conflicts: # If there are confirmed temporal conflicts
        final_status = "conflict_detected"
        explanation_lines.append("\nOverall Status: CONFLICT DETECTED due to spatio-temporal overlaps.")
    elif spatial_conflicts: # Spatial proximity but no temporal conflict under assumed schedule
        final_status = "clear_with_spatial_warning" # A more nuanced status
        explanation_lines.append("\nOverall Status: CLEAR (with spatial proximity warning). Paths are close but no time overlap with assumed schedule.")
        explanation_lines.append("Consider mission timing flexibility or increasing spatial separation.")
    else: # No spatial, hence no temporal
        final_status = "clear"
        explanation_lines.append("\nOverall Status: CLEAR. No spatial or temporal conflicts detected.")


    return DeconflictionResult(
        status=final_status,
        spatial_conflicts=spatial_conflicts,
        temporal_conflicts=temporal_conflicts, # Will be populated by check_temporal_conflict
        explanation="\n".join(explanation_lines)
    )


if __name__ == '__main__':
    # Example Usage (requires data_structures to be in the python path or same dir for direct run)
    # To run this directly for testing, ensure PYTHONPATH includes the project root or run as a module.
    # For simplicity, let's redefine minimal structures here for a quick test,
    # or ensure this is run in an environment where `from .data_structures import ...` works.

    # Re-defining for standalone test, normally imported
    class MissionWaypoint(NamedTuple):
        x: float
        y: float
        z: float = 0.0

    class TimeWindow(NamedTuple):
        start_time: float
        end_time: float

    class DroneMission(NamedTuple):
        mission_id: str
        waypoints: List[MissionWaypoint]
        time_window: TimeWindow

    class SimulatedFlight(NamedTuple):
        flight_id: str
        trajectory: List[Tuple[MissionWaypoint, float]]


    print("Running conflict_checker.py example...")

    # Primary Drone Mission Example
    pm_waypoints = [
        MissionWaypoint(x=0, y=0, z=10), MissionWaypoint(x=100, y=0, z=10),
        MissionWaypoint(x=100, y=100, z=10), MissionWaypoint(x=0, y=100, z=10)
    ]
    pm_time_window = TimeWindow(start_time=0, end_time=3600)
    primary_mission_eg = DroneMission("PM01", pm_waypoints, pm_time_window)

    # Simulated Flights
    sim_flight1_traj = [
        (MissionWaypoint(x=50, y=-10, z=10), 100), # Arrives at (50,-10,10) at t=100
        (MissionWaypoint(x=50, y=60, z=10), 700)  # Arrives at (50,60,10) at t=700. Segment duration = 600s
                                                  # Primary mission segment (0,0,10)-(100,0,10) is length 100.
                                                  # Total PM dist = 100+100+100 = 300. PM duration = 3600s.
                                                  # PM avg speed = 300/3600 = 1/12 units/sec.
                                                  # Time for PM seg1 (len 100) = 100 / (1/12) = 1200s.
                                                  # PM seg1: t=[0, 1200].
                                                  # Sim seg1: t=[100, 700]. Overlap.
    ]
    sim_flight_1_eg = SimulatedFlight("SF01_CONFLICT", sim_flight1_traj)

    # Simulated flight that is spatially close but temporally separated
    sim_flight2_traj_spatial_only = [
        (MissionWaypoint(x=50, y=-5, z=10), 3000), # Starts late
        (MissionWaypoint(x=50, y=5, z=10), 3100)   # Finishes late, no overlap with PM seg1 [0,1200]
    ]
    sim_flight_2_eg_spatial_only = SimulatedFlight("SF02_SPATIAL_ONLY", sim_flight2_traj_spatial_only)


    # Simulated flight that is far away
    sim_flight3_traj_clear = [
        (MissionWaypoint(x=200, y=200, z=10), 100),
        (MissionWaypoint(x=250, y=250, z=10), 200) # No conflict
    ]
    sim_flight_3_eg_clear = SimulatedFlight("SF03_CLEAR", sim_flight3_traj_clear)
    
    simulated_flights_eg = [sim_flight_1_eg, sim_flight_2_eg_spatial_only, sim_flight_3_eg_clear]

    # Test spatial check (retained for direct validation if needed)
    # print("\n--- Testing Spatial Check (Direct) ---")
    # spatial_conflicts_found = perform_spatial_check(primary_mission_eg, simulated_flights_eg, safety_buffer=15.0)
    # if spatial_conflicts_found:
    #     print(f"Found {len(spatial_conflicts_found)} direct spatial conflicts:")
    #     for conflict in spatial_conflicts_found:
    #         print(f"  Primary segment: {conflict.primary_mission_segment}")
    #         print(f"  Conflicting flight: {conflict.conflicting_flight_id}, segment: {conflict.conflicting_flight_segment}")
    # else:
    #     print("No direct spatial conflicts found.")

    # Test main deconfliction function
    print("\n--- Testing Main Deconfliction (Scenario with mixed conflicts) ---")
    # Use default safety_buffer_time (30s) from the function definition
    result = run_deconfliction_check(primary_mission_eg, simulated_flights_eg, safety_buffer_distance=15.0, safety_buffer_time=30.0)
    print(f"Deconfliction Status: {result.status}")
    print("Explanation:")
    print(result.explanation)
    if result.temporal_conflicts:
        print(f"Number of temporal conflicts: {len(result.temporal_conflicts)}")
    if result.spatial_conflicts and not result.temporal_conflicts:
        print(f"Number of spatial proximities without temporal conflict: {len(result.spatial_conflicts)}")


    # Example of a completely clear scenario
    pm_clear_waypoints = [MissionWaypoint(x=0,y=0,z=50), MissionWaypoint(x=10,y=0,z=50)] # Short mission
    primary_mission_clear = DroneMission("PM_CLEAR", pm_clear_waypoints, pm_time_window)
    sim_far_away_traj = [(MissionWaypoint(x=1000,y=1000,z=50), 100), (MissionWaypoint(x=1010,y=1000,z=50),200)]
    sim_flight_far_away = SimulatedFlight("SF_FAR", sim_far_away_traj)
    
    print("\n--- Testing Clear Scenario ---")
    result_clear = run_deconfliction_check(primary_mission_clear, [sim_flight_far_away], safety_buffer_distance=15.0)
    print(f"Deconfliction Status (Clear): {result_clear.status}")
    print("Explanation (Clear):")
    print(result_clear.explanation)

    # Test point near segment
    p = MissionWaypoint(5,0,0)
    s1 = MissionWaypoint(0,0,0)
    s2 = MissionWaypoint(10,0,0)
    print(f"\nIs point {p} near segment {s1}-{s2} (buffer 1.0)? {is_point_near_segment(p,s1,s2,1.0)}") # True
    p2 = MissionWaypoint(15,0,0)
    print(f"Is point {p2} near segment {s1}-{s2} (buffer 1.0)? {is_point_near_segment(p2,s1,s2,1.0)}") # False (closest is s2, dist 5)
    print(f"Is point {p2} near segment {s1}-{s2} (buffer 6.0)? {is_point_near_segment(p2,s1,s2,6.0)}") # True

    p3 = MissionWaypoint(5,5,0) # Point off the line
    print(f"Is point {p3} near segment {s1}-{s2} (buffer 1.0)? {is_point_near_segment(p3,s1,s2,1.0)}") # False (dist is 5)
    print(f"Is point {p3} near segment {s1}-{s2} (buffer 6.0)? {is_point_near_segment(p3,s1,s2,6.0)}") # True

    # Test 3D distance
    p3d_1 = MissionWaypoint(0,0,0)
    p3d_2 = MissionWaypoint(3,4,0) # dist 5 in xy
    print(f"Distance 2D (3,4,0) from (0,0,0): {distance_between_points(p3d_1, p3d_2)}")
    p3d_3 = MissionWaypoint(3,4,12) # dist sqrt(3^2+4^2+12^2) = sqrt(25+144) = sqrt(169) = 13
    print(f"Distance 3D (3,4,12) from (0,0,0): {distance_between_points(p3d_1, p3d_3)}")