"""
Handles the visualization of drone missions, trajectories, and conflicts.
Requires matplotlib: pip install matplotlib
"""
import os
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from typing import List, Optional, Tuple

# Assuming data_structures and conflict_checker are in the same package level
# For direct execution or testing, ensure PYTHONPATH is set up correctly.
try:
    from .data_structures import DroneMission, SimulatedFlight, MissionWaypoint
    from .conflict_checker import SpatialConflict, TemporalConflict, DeconflictionResult, distance_between_points
except ImportError:
    # Fallback for cases where the script might be run directly for simple tests
    # and the package structure isn't immediately recognized.
    # This is not ideal for production but helps in isolated development.
    print("Warning: Running visualization.py potentially outside of a package context. Relative imports might fail.")
    from data_structures import DroneMission, SimulatedFlight, MissionWaypoint # type: ignore
    from conflict_checker import SpatialConflict, TemporalConflict, DeconflictionResult, distance_between_points # type: ignore


def plot_missions_2d(
    primary_mission: DroneMission,
    simulated_flights: List[SimulatedFlight],
    deconfliction_result: Optional[DeconflictionResult] = None,
    title: str = "Drone Missions Overview (2D)",
    output_filename: Optional[str] = None
):
    """
    Generates a 2D plot of the primary drone mission and simulated flight paths.
    Highlights spatial and temporal conflicts if provided.
    """
    fig, ax = plt.subplots(figsize=(12, 10))

    # Plot Primary Mission
    pm_x = [wp.x for wp in primary_mission.waypoints]
    pm_y = [wp.y for wp in primary_mission.waypoints]
    ax.plot(pm_x, pm_y, 'o-', label=f"Primary Mission ({primary_mission.mission_id})", color='blue', linewidth=2, markersize=8)
    ax.text(pm_x[0], pm_y[0], ' Start', color='blue', va='bottom', ha='left')
    ax.text(pm_x[-1], pm_y[-1], ' End', color='blue', va='top', ha='right')

    # Plot Simulated Flights
    colors = plt.cm.get_cmap('viridis', len(simulated_flights) + 1) # type: ignore
    for i, sim_flight in enumerate(simulated_flights):
        if not sim_flight.trajectory:
            continue
        sim_x = [wp_data[0].x for wp_data in sim_flight.trajectory]
        sim_y = [wp_data[0].y for wp_data in sim_flight.trajectory]
        # Times for annotation
        sim_t = [wp_data[1] for wp_data in sim_flight.trajectory]

        ax.plot(sim_x, sim_y, 's--', label=f"Simulated Flight ({sim_flight.flight_id})", color=colors(i), linewidth=1.5, markersize=6)
        if sim_x: # Check if list is not empty
            ax.text(sim_x[0], sim_y[0], f' S (T={sim_t[0]:.0f})', color=colors(i), fontsize=8, va='bottom')
            ax.text(sim_x[-1], sim_y[-1], f' E (T={sim_t[-1]:.0f})', color=colors(i), fontsize=8, va='top')


    # Highlight Conflicts if present
    if deconfliction_result:
        # Highlight Spatial Conflicts (areas of proximity)
        if deconfliction_result.spatial_conflicts:
            for i, sp_conflict in enumerate(deconfliction_result.spatial_conflicts):
                # Primary segment
                ps_wp1, ps_wp2 = sp_conflict.primary_mission_segment
                ax.plot([ps_wp1.x, ps_wp2.x], [ps_wp1.y, ps_wp2.y], color='orange', linewidth=3, linestyle=':', alpha=0.7, label=f'_nolegend_SpatialProx{i+1}_PM')
                
                # Sim segment
                ss_wp1, ss_wp2 = sp_conflict.conflicting_flight_segment
                ax.plot([ss_wp1.x, ss_wp2.x], [ss_wp1.y, ss_wp2.y], color='red', linewidth=3, linestyle=':', alpha=0.7, label=f'_nolegend_SpatialProx{i+1}_SIM_{sp_conflict.conflicting_flight_id}')
                
                # Mark center of spatial conflict area (approx)
                # Midpoint of the primary conflicting segment for annotation
                mid_x = (ps_wp1.x + ps_wp2.x) / 2
                mid_y = (ps_wp1.y + ps_wp2.y) / 2
                ax.scatter(mid_x, mid_y, s=100, color='yellow', marker='X', edgecolors='black', zorder=5, label=f'_nolegend_Spatial_Conflict_Marker_{i}')
                ax.text(mid_x, mid_y, f' SP{i+1}', color='black', fontsize=9, ha='center', va='center')


        # Highlight Temporal Conflicts (actual collisions in time and space)
        if deconfliction_result.temporal_conflicts:
            for i, tc in enumerate(deconfliction_result.temporal_conflicts):
                # The temporal conflict refers to segments already marked by spatial.
                # We can add a more prominent marker or annotation for temporal.
                pm_seg_wp1, pm_seg_wp2 = tc.primary_segment_involved
                
                # Midpoint of the primary temporally conflicting segment for annotation
                tc_mid_x = (pm_seg_wp1.x + pm_seg_wp2.x) / 2
                tc_mid_y = (pm_seg_wp1.y + pm_seg_wp2.y) / 2
                
                ax.scatter(tc_mid_x, tc_mid_y, s=200, color='magenta', marker='*', edgecolors='black', zorder=6, label=f'_nolegend_Temporal_Conflict_Marker_{i}')
                ax.text(tc_mid_x, tc_mid_y, f' TC{i+1}\n{tc.conflicting_flight_id}\n[{tc.time_overlap_start:.0f}-{tc.time_overlap_end:.0f}s]',
                          color='magenta', fontsize=10, ha='center', va='bottom',
                          bbox=dict(facecolor='white', alpha=0.5, pad=0.2))

    ax.set_xlabel("X Coordinate (meters)")
    ax.set_ylabel("Y Coordinate (meters)")
    ax.set_title(title)
    ax.legend(loc='best', fontsize='small')
    ax.grid(True, linestyle='--', alpha=0.7)
    ax.axis('equal') # Ensure aspect ratio is equal for correct spatial representation

    if output_filename:
        plt.savefig(output_filename)
        print(f"Plot saved to {output_filename}")
    else:
        plt.show()

    plt.close(fig) # Close the figure to free memory


def generate_conflict_animation_2d(
    primary_mission: DroneMission,
    simulated_flights: List[SimulatedFlight],
    deconfliction_result: DeconflictionResult, # Required for animation context
    output_filename: str = "mission_animation_2d.mp4",
    frames_per_second: int = 10,
    safety_buffer_dist: float = 10.0 # From conflict_checker.DEFAULT_SAFETY_BUFFER_DISTANCE
):
    """
    Generates a 2D animation of the missions, highlighting drones and conflicts over time.
    (This is a more complex function and will be a simplified representation)
    """
    print(f"Attempting to generate 2D animation: {output_filename}")
    # This is a placeholder for a more detailed animation.
    # A full animation would require interpolating positions at each time step.

    fig, ax = plt.subplots(figsize=(12, 10))
    ax.set_xlabel("X Coordinate (meters)")
    ax.set_ylabel("Y Coordinate (meters)")
    ax.grid(True, linestyle='--', alpha=0.7)
    ax.axis('equal')

    # Determine time range for animation
    min_time = primary_mission.time_window.start_time
    max_time = primary_mission.time_window.end_time
    for sf in simulated_flights:
        if sf.trajectory:
            min_time = min(min_time, sf.trajectory[0][1])
            max_time = max(max_time, sf.trajectory[-1][1])
    
    if max_time <= min_time: # Avoid issues with zero or negative duration
        max_time = min_time + 100 # Default duration if times are problematic
        if not primary_mission.waypoints and not any(sf.trajectory for sf in simulated_flights):
             print("Cannot generate animation: No mission data or valid time range.")
             plt.close(fig)
             return


    time_steps = list(range(int(min_time), int(max_time) + 1, 1)) # 1-second intervals
    if not time_steps:
        time_steps = [min_time] # Single frame if duration is too short

    # Static paths for reference
    pm_x_full = [wp.x for wp in primary_mission.waypoints]
    pm_y_full = [wp.y for wp in primary_mission.waypoints]
    ax.plot(pm_x_full, pm_y_full, ':', color='blue', alpha=0.3, label='_nolegend_PM Path')

    sim_paths_full = []
    for sf in simulated_flights:
        if sf.trajectory:
            sim_x_full = [wp_data[0].x for wp_data in sf.trajectory]
            sim_y_full = [wp_data[0].y for wp_data in sf.trajectory]
            sim_paths_full.append(ax.plot(sim_x_full, sim_y_full, ':', color='gray', alpha=0.3, label=f'_nolegend_{sf.flight_id} Path')[0])
        else:
            sim_paths_full.append(None)


    # Dynamic elements (drone positions)
    pm_dot, = ax.plot([], [], 'o', color='blue', markersize=10, label=f"Primary ({primary_mission.mission_id})")
    sim_dots = [ax.plot([], [], 's', markersize=8, label=f"Sim ({sf.flight_id})")[0] for sf in simulated_flights]
    time_text = ax.text(0.02, 0.95, '', transform=ax.transAxes)
    conflict_indicator_circles = [] # For highlighting drones during conflict

    # Pre-calculate primary mission speed and segment times (simplified from conflict_checker)
    total_pm_dist = sum(distance_between_points(primary_mission.waypoints[i], primary_mission.waypoints[i+1]) for i in range(len(primary_mission.waypoints)-1))
    pm_duration = primary_mission.time_window.end_time - primary_mission.time_window.start_time
    pm_avg_speed = (total_pm_dist / pm_duration) if pm_duration > 0 else 0

    def get_position_at_time(waypoints: List[MissionWaypoint], mission_start_time: float, mission_end_time: float, current_time: float, avg_speed: Optional[float] = None) -> Optional[MissionWaypoint]:
        """Gets position of primary drone assuming constant speed."""
        if not waypoints or len(waypoints) < 2 or current_time < mission_start_time or current_time > mission_end_time:
            return waypoints[0] if waypoints and current_time <= mission_start_time else (waypoints[-1] if waypoints and current_time >= mission_end_time else None)

        if avg_speed is None: # Fallback if not provided (should be for primary)
            total_dist = sum(distance_between_points(waypoints[i], waypoints[i+1]) for i in range(len(waypoints)-1))
            duration = mission_end_time - mission_start_time
            avg_speed_calc = (total_dist / duration) if duration > 0 else 0
        else:
            avg_speed_calc = avg_speed
        
        if avg_speed_calc == 0: # Drone is stationary or mission has no duration/distance
             # Find which waypoint it should be at based on time (if waypoints had individual times)
             # For now, assume it stays at the first waypoint if speed is 0.
            return waypoints[0]


        dist_to_cover = (current_time - mission_start_time) * avg_speed_calc
        
        cumulative_dist = 0.0
        for i in range(len(waypoints) - 1):
            p1, p2 = waypoints[i], waypoints[i+1]
            seg_len = distance_between_points(p1, p2)
            if cumulative_dist + seg_len >= dist_to_cover:
                # Drone is on this segment
                fraction = (dist_to_cover - cumulative_dist) / seg_len if seg_len > 0 else 0
                x = p1.x + fraction * (p2.x - p1.x)
                y = p1.y + fraction * (p2.y - p1.y)
                z = p1.z + fraction * (p2.z - p1.z)
                return MissionWaypoint(x,y,z)
            cumulative_dist += seg_len
        return waypoints[-1] # Reached end

    def get_sim_position_at_time(trajectory: List[Tuple[MissionWaypoint, float]], current_time: float) -> Optional[MissionWaypoint]:
        """Gets position of simulated drone by interpolating between timed waypoints."""
        if not trajectory: return None
        if current_time < trajectory[0][1]: return trajectory[0][0] # Before start
        if current_time > trajectory[-1][1]: return trajectory[-1][0] # After end

        for i in range(len(trajectory) - 1):
            wp_data1, wp_data2 = trajectory[i], trajectory[i+1]
            p1, t1 = wp_data1
            p2, t2 = wp_data2
            if t1 <= current_time <= t2:
                if t1 == t2: return p1 # Hovering or instant jump
                fraction = (current_time - t1) / (t2 - t1)
                x = p1.x + fraction * (p2.x - p1.x)
                y = p1.y + fraction * (p2.y - p1.y)
                z = p1.z + fraction * (p2.z - p1.z)
                return MissionWaypoint(x,y,z)
        return trajectory[-1][0] # Should be caught by earlier checks

    def update(frame_time: float):
        # Update Primary Mission Drone
        pm_pos = get_position_at_time(primary_mission.waypoints, primary_mission.time_window.start_time, primary_mission.time_window.end_time, frame_time, pm_avg_speed)
        if pm_pos:
            pm_dot.set_data([pm_pos.x], [pm_pos.y])
        else:
            pm_dot.set_data([],[])

        # Update Simulated Drones
        sim_positions = []
        for i, sf in enumerate(simulated_flights):
            sim_pos = get_sim_position_at_time(sf.trajectory, frame_time)
            if sim_pos:
                sim_dots[i].set_data([sim_pos.x], [sim_pos.y])
                sim_dots[i].set_color(plt.cm.get_cmap('viridis', len(simulated_flights) +1)(i)) # type: ignore
                sim_positions.append(sim_pos)
            else:
                sim_dots[i].set_data([],[])
                sim_positions.append(None)
        
        # Clear previous conflict indicators
        for circle in conflict_indicator_circles:
            circle.remove()
        conflict_indicator_circles.clear()

        # Check and highlight current conflicts (simplified: proximity check at current time)
        if pm_pos:
            for i, sim_pos in enumerate(sim_positions):
                if sim_pos:
                    dist = distance_between_points(pm_pos, sim_pos)
                    if dist < safety_buffer_dist: # Using the spatial safety buffer for visual cue
                        # Highlight PM drone
                        circle1 = plt.Circle((pm_pos.x, pm_pos.y), safety_buffer_dist, color='red', fill=False, linestyle='--', alpha=0.8, zorder=10)
                        ax.add_artist(circle1)
                        conflict_indicator_circles.append(circle1)
                        # Highlight Sim drone
                        circle2 = plt.Circle((sim_pos.x, sim_pos.y), safety_buffer_dist, color='red', fill=False, linestyle='--', alpha=0.8, zorder=10)
                        ax.add_artist(circle2)
                        conflict_indicator_circles.append(circle2)
                        
                        # Check if this specific pair is in a *confirmed* temporal conflict from deconfliction_result
                        for tc in deconfliction_result.temporal_conflicts:
                            if tc.conflicting_flight_id == simulated_flights[i].flight_id and \
                               tc.time_overlap_start <= frame_time <= tc.time_overlap_end:
                                # More prominent highlight for confirmed temporal conflict
                                circle1.set_color('magenta')
                                circle1.set_linewidth(2.5)
                                circle2.set_color('magenta')
                                circle2.set_linewidth(2.5)
                                break


        time_text.set_text(f'Time: {frame_time:.1f}s')
        
        all_dots = [pm_dot] + sim_dots + conflict_indicator_circles # type: ignore
        return tuple(all_dots) + (time_text,)


    # Set animation range based on actual data
    all_x = pm_x_full + [wp_data[0].x for sf in simulated_flights if sf.trajectory for wp_data in sf.trajectory]
    all_y = pm_y_full + [wp_data[0].y for sf in simulated_flights if sf.trajectory for wp_data in sf.trajectory]
    if all_x and all_y:
        ax.set_xlim(min(all_x) - 50, max(all_x) + 50)
        ax.set_ylim(min(all_y) - 50, max(all_y) + 50)
    else: # Default view if no data
        ax.set_xlim(-100, 100)
        ax.set_ylim(-100, 100)
    
    ax.legend(loc='upper right', fontsize='small')
    ax.set_title("Drone Mission Animation (2D)")

    if not time_steps:
        print("No time steps to animate. Skipping animation generation.")
        plt.close(fig)
        return

    # Create animation
    # Interval is 1000 / frames_per_second
    ani = FuncAnimation(fig, update, frames=time_steps, blit=True, interval=1000/frames_per_second, repeat=False)

    try:
        print(f"Attempting to save animation as MP4: {output_filename}")
        ani.save(output_filename, writer='ffmpeg', fps=frames_per_second)
        print(f"MP4 animation saved successfully to {output_filename}")
    except Exception as e_mp4:
        print(f"Failed to save as MP4: {e_mp4}")
        print("FFmpeg might not be installed or not in PATH.")
        
        gif_filename = os.path.splitext(output_filename)[0] + ".gif"
        print(f"Attempting to save animation as GIF instead: {gif_filename}")
        try:
            ani.save(gif_filename, writer='pillow', fps=frames_per_second)
            print(f"GIF animation saved successfully to {gif_filename}")
        except Exception as e_gif:
            print(f"Failed to save as GIF: {e_gif}")
            print("Pillow writer might also have issues, or another error occurred.")
    finally:
        plt.close(fig)


# --- Placeholder for 3D/4D Visualization ---
def plot_missions_3d(
    primary_mission: DroneMission,
    simulated_flights: List[SimulatedFlight],
    deconfliction_result: Optional[DeconflictionResult] = None,
    title: str = "Drone Missions Overview (3D)",
    output_filename: Optional[str] = None
):
    """
    Generates a 3D plot of drone missions. (Extra Credit)
    """
    fig = plt.figure(figsize=(14, 12))
    ax = fig.add_subplot(111, projection='3d')

    # Plot Primary Mission
    pm_x = [wp.x for wp in primary_mission.waypoints]
    pm_y = [wp.y for wp in primary_mission.waypoints]
    pm_z = [wp.z for wp in primary_mission.waypoints]
    ax.plot(pm_x, pm_y, pm_z, 'o-', label=f"Primary Mission ({primary_mission.mission_id})", color='blue', linewidth=2, markersize=8)
    if pm_x: # Check if list is not empty
        ax.text(pm_x[0], pm_y[0], pm_z[0], ' Start', color='blue')
        ax.text(pm_x[-1], pm_y[-1], pm_z[-1], ' End', color='blue')


    # Plot Simulated Flights
    colors = plt.cm.get_cmap('viridis', len(simulated_flights) + 1) # type: ignore
    for i, sim_flight in enumerate(simulated_flights):
        if not sim_flight.trajectory:
            continue
        sim_x = [wp_data[0].x for wp_data in sim_flight.trajectory]
        sim_y = [wp_data[0].y for wp_data in sim_flight.trajectory]
        sim_z = [wp_data[0].z for wp_data in sim_flight.trajectory]
        sim_t = [wp_data[1] for wp_data in sim_flight.trajectory]

        ax.plot(sim_x, sim_y, sim_z, 's--', label=f"Simulated Flight ({sim_flight.flight_id})", color=colors(i), linewidth=1.5, markersize=6)
        if sim_x: # Check if list is not empty
            ax.text(sim_x[0], sim_y[0], sim_z[0], f' S (T={sim_t[0]:.0f})', color=colors(i), fontsize=8)
            ax.text(sim_x[-1], sim_y[-1], sim_z[-1], f' E (T={sim_t[-1]:.0f})', color=colors(i), fontsize=8)

    # Highlight Conflicts (simplified for 3D)
    if deconfliction_result:
        if deconfliction_result.spatial_conflicts:
            for i, sp_conflict in enumerate(deconfliction_result.spatial_conflicts):
                ps_wp1, ps_wp2 = sp_conflict.primary_mission_segment
                ax.plot([ps_wp1.x, ps_wp2.x], [ps_wp1.y, ps_wp2.y], [ps_wp1.z, ps_wp2.z], color='orange', linewidth=4, linestyle=':', alpha=0.7)
                
                ss_wp1, ss_wp2 = sp_conflict.conflicting_flight_segment
                ax.plot([ss_wp1.x, ss_wp2.x], [ss_wp1.y, ss_wp2.y], [ss_wp1.z, ss_wp2.z], color='red', linewidth=4, linestyle=':', alpha=0.7)
                
                mid_x = (ps_wp1.x + ps_wp2.x) / 2
                mid_y = (ps_wp1.y + ps_wp2.y) / 2
                mid_z = (ps_wp1.z + ps_wp2.z) / 2
                ax.scatter(mid_x, mid_y, mid_z, s=150, color='yellow', marker='X', edgecolors='black', depthshade=True, label=f'_nolegend_Spatial_Conflict_Marker3D_{i}')
                # ax.text(mid_x, mid_y, mid_z, f'SP{i+1}', color='black', fontsize=9)


        if deconfliction_result.temporal_conflicts:
            for i, tc in enumerate(deconfliction_result.temporal_conflicts):
                pm_seg_wp1, _ = tc.primary_segment_involved
                tc_mid_x, tc_mid_y, tc_mid_z = pm_seg_wp1.x, pm_seg_wp1.y, pm_seg_wp1.z # Approx location
                ax.scatter(tc_mid_x, tc_mid_y, tc_mid_z, s=250, color='magenta', marker='*', edgecolors='black', depthshade=True, label=f'_nolegend_Temporal_Conflict_Marker3D_{i}')
                # ax.text(tc_mid_x, tc_mid_y, tc_mid_z, f'TC{i+1}\n{tc.conflicting_flight_id}\n[{tc.time_overlap_start:.0f}-{tc.time_overlap_end:.0f}s]', color='magenta', fontsize=10)


    ax.set_xlabel("X Coordinate (meters)")
    ax.set_ylabel("Y Coordinate (meters)")
    ax.set_zlabel("Z Coordinate (Altitude - meters)")
    ax.set_title(title)
    ax.legend(loc='best', fontsize='small')
    # ax.view_init(elev=20., azim=-35) # Example view angle

    if output_filename:
        plt.savefig(output_filename)
        print(f"3D Plot saved to {output_filename}")
    else:
        plt.show()
    plt.close(fig)

# Example Usage (for testing this module directly)
if __name__ == '__main__':
    print("Running visualization.py example...")

    # Define dummy data structures if not imported (e.g. running file directly)
    # This is usually handled by the try-except ImportError block above.
    # If that fails, these local re-definitions are a last resort for __main__
    from collections import namedtuple
    if 'MissionWaypoint' not in globals():
        MissionWaypoint = namedtuple('MissionWaypoint', ['x', 'y', 'z']) # type: ignore
    if 'TimeWindow' not in globals():
        TimeWindow = namedtuple('TimeWindow', ['start_time', 'end_time']) # type: ignore
    if 'DroneMission' not in globals():
        DroneMission = namedtuple('DroneMission', ['mission_id', 'waypoints', 'time_window']) # type: ignore
    if 'SimulatedFlight' not in globals():
        SimulatedFlight = namedtuple('SimulatedFlight', ['flight_id', 'trajectory']) # type: ignore
    if 'SpatialConflict' not in globals():
        SpatialConflict = namedtuple('SpatialConflict', ['primary_mission_segment', 'conflicting_flight_id', 'conflicting_flight_segment']) # type: ignore
    if 'TemporalConflict' not in globals():
        TemporalConflict = namedtuple('TemporalConflict', ['conflicting_flight_id', 'primary_segment_involved', 'sim_segment_involved', 'time_overlap_start', 'time_overlap_end', 'details']) # type: ignore
    if 'DeconflictionResult' not in globals():
        DeconflictionResult = namedtuple('DeconflictionResult', ['status', 'spatial_conflicts', 'temporal_conflicts', 'explanation']) # type: ignore


    # Example Data
    pm_waypoints_2d = [
        MissionWaypoint(x=0, y=0, z=0), MissionWaypoint(x=100, y=0, z=0),
        MissionWaypoint(x=100, y=100, z=0), MissionWaypoint(x=0, y=100, z=0),
        MissionWaypoint(x=0, y=0, z=0)
    ]
    pm_time_window = TimeWindow(start_time=0, end_time=360) # 6 minutes
    primary_mission_ex = DroneMission("PM_Viz", pm_waypoints_2d, pm_time_window)

    sim_flight1_traj_2d = [
        (MissionWaypoint(x=50, y=-10, z=0), 60),
        (MissionWaypoint(x=50, y=60, z=0), 180) 
    ]
    sim_flight1_ex = SimulatedFlight("SF_Viz1", sim_flight1_traj_2d)

    sim_flight2_traj_2d = [
        (MissionWaypoint(x=10, y=10, z=0), 30),
        (MissionWaypoint(x=110, y=10, z=0), 150) # Spatially close to PM's first segment
    ]
    sim_flight2_ex = SimulatedFlight("SF_Viz2_NoConflict", sim_flight2_traj_2d)
    
    simulated_flights_ex = [sim_flight1_ex, sim_flight2_ex]

    # Mock Deconfliction Result for visualization
    # Spatial conflict on PM segment 0-1 and SF_Viz1 segment 0-1
    sc1_pm_seg = (pm_waypoints_2d[0], pm_waypoints_2d[1])
    sc1_sim_seg = (sim_flight1_traj_2d[0][0], sim_flight1_traj_2d[1][0])
    spatial_conflict1 = SpatialConflict(
        primary_mission_segment=sc1_pm_seg,
        conflicting_flight_id="SF_Viz1",
        conflicting_flight_segment=sc1_sim_seg
    )
    # Temporal conflict for the same
    temporal_conflict1 = TemporalConflict(
        conflicting_flight_id="SF_Viz1",
        primary_segment_involved=sc1_pm_seg,
        sim_segment_involved=sc1_sim_seg,
        time_overlap_start=60, # Example overlap
        time_overlap_end=100,  # Example overlap (PM seg1 time: 0 to 100/ ( (100+100+100+100)/360 ) = 90s. So PM seg1 is [0,90])
                               # Sim seg1 is [60,180]. Overlap [60,90]
        details="PM seg [0,0]-[100,0] (T:0-90s) vs SF_Viz1 seg [50,-10]-[50,60] (T:60-180s)"
    )

    mock_result_with_conflict = DeconflictionResult(
        status="conflict_detected",
        spatial_conflicts=[spatial_conflict1],
        temporal_conflicts=[temporal_conflict1],
        explanation="Mock conflict detected for visualization."
    )
    
    mock_result_no_conflict = DeconflictionResult(
        status="clear",
        spatial_conflicts=[],
        temporal_conflicts=[],
        explanation="Mock clear for visualization."
    )

    # Test 2D Plotting
    print("\n--- Testing 2D Plot (No Conflict) ---")
    plot_missions_2d(primary_mission_ex, simulated_flights_ex, mock_result_no_conflict, "2D Plot - No Conflict Scenario", "plot_2d_no_conflict.png")

    print("\n--- Testing 2D Plot (With Conflict) ---")
    plot_missions_2d(primary_mission_ex, simulated_flights_ex, mock_result_with_conflict, "2D Plot - Conflict Scenario", "plot_2d_with_conflict.png")

    # Test 3D Plotting (using z=0 for these 2D waypoints, but structure is 3D)
    pm_waypoints_3d = [MissionWaypoint(x=wp.x, y=wp.y, z=(i*5)) for i, wp in enumerate(pm_waypoints_2d)] # Add some Z
    primary_mission_3d_ex = DroneMission("PM_Viz_3D", pm_waypoints_3d, pm_time_window)
    
    sim_flight1_traj_3d = [(MissionWaypoint(wp[0].x, wp[0].y, z=10), wp[1]) for wp in sim_flight1_traj_2d]
    sim_flight1_3d_ex = SimulatedFlight("SF_Viz1_3D", sim_flight1_traj_3d)
    simulated_flights_3d_ex = [sim_flight1_3d_ex]

    # Mock 3D conflict result
    sc1_pm_seg_3d = (pm_waypoints_3d[0], pm_waypoints_3d[1])
    sc1_sim_seg_3d = (sim_flight1_traj_3d[0][0], sim_flight1_traj_3d[1][0])
    spatial_conflict1_3d = SpatialConflict(sc1_pm_seg_3d, "SF_Viz1_3D", sc1_sim_seg_3d)
    temporal_conflict1_3d = TemporalConflict("SF_Viz1_3D", sc1_pm_seg_3d, sc1_sim_seg_3d, 60,90, "3D mock conflict")
    mock_result_3d_conflict = DeconflictionResult("conflict_detected", [spatial_conflict1_3d], [temporal_conflict1_3d], "3D Mock")


    print("\n--- Testing 3D Plot (With Conflict) ---")
    plot_missions_3d(primary_mission_3d_ex, simulated_flights_3d_ex, mock_result_3d_conflict, "3D Plot - Conflict Scenario", "plot_3d_with_conflict.png")

    # Test Animation (will try to save, ensure ffmpeg is available or change writer)
    # This is a more intensive operation.
    print("\n--- Testing 2D Animation (With Conflict) ---")
    # Ensure safety_buffer_dist matches what might be used in conflict_checker for visual consistency
    # Using a default here for the visualization function itself.
    generate_conflict_animation_2d(primary_mission_ex, simulated_flights_ex, mock_result_with_conflict, "mission_animation_2d_conflict.mp4", safety_buffer_dist=10.0)
    
    print("\n--- Testing 2D Animation (No Conflict) ---")
    generate_conflict_animation_2d(primary_mission_ex, [sim_flight2_ex], mock_result_no_conflict, "mission_animation_2d_no_conflict.mp4", safety_buffer_dist=10.0)

    print("Visualization example run complete. Check for .png and .mp4 files.")