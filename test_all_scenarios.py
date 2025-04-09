from datetime import datetime, timedelta
import matplotlib.pyplot as plt
import numpy as np
from conflict_visualization import Waypoint, DroneTrajectory, DeconflictionService

def create_visualization(scenario_name, primary_trajectory, other_trajectories, conflicts, safety_buffer):
    """Create and save a visualization for a specific scenario"""
    fig = plt.figure(figsize=(15, 10))
    
    # 3D trajectory plot
    ax1 = fig.add_subplot(221, projection='3d')
    
    # Plot primary trajectory
    x1 = [wp.x for wp in primary_trajectory.waypoints]
    y1 = [wp.y for wp in primary_trajectory.waypoints]
    z1 = [wp.z for wp in primary_trajectory.waypoints]
    ax1.plot(x1, y1, z1, 'go-', label='Primary')
    
    # Plot other trajectories
    for i, traj in enumerate(other_trajectories):
        x = [wp.x for wp in traj.waypoints]
        y = [wp.y for wp in traj.waypoints]
        z = [wp.z for wp in traj.waypoints]
        ax1.plot(x, y, z, 'o-', label=f'Drone {traj.drone_id}')
    
    # Plot conflicts if any
    if conflicts:
        sample_conflicts = conflicts[::max(1, len(conflicts)//10)]  # Sample up to 10 conflicts
        conflict_x = [c.location.x for c in sample_conflicts]
        conflict_y = [c.location.y for c in sample_conflicts]
        conflict_z = [c.location.z for c in sample_conflicts]
        ax1.scatter(conflict_x, conflict_y, conflict_z, c='red', s=100, alpha=0.5, label='Conflicts')
    
    ax1.set_xlabel('X')
    ax1.set_ylabel('Y')
    ax1.set_zlabel('Z')
    ax1.set_title('3D Drone Trajectories')
    ax1.legend()
    
    # 2D top view
    ax2 = fig.add_subplot(222)
    ax2.plot(x1, y1, 'go-', label='Primary')
    for i, traj in enumerate(other_trajectories):
        x = [wp.x for wp in traj.waypoints]
        y = [wp.y for wp in traj.waypoints]
        ax2.plot(x, y, 'o-', label=f'Drone {traj.drone_id}')
    if conflicts:
        ax2.scatter([c.location.x for c in sample_conflicts], 
                   [c.location.y for c in sample_conflicts], 
                   c='red', s=50, alpha=0.5)
    ax2.set_xlabel('X')
    ax2.set_ylabel('Y')
    ax2.set_title('Top View (XY Plane)')
    ax2.grid(True)
    
    # Text summary
    ax3 = fig.add_subplot(223)
    ax3.axis('off')
    
    summary_text = f"""Scenario: {scenario_name}
    
Status: {"Conflict detected" if conflicts else "Clear"}
Number of conflicts: {len(conflicts) if conflicts else 0}
Safety buffer: {safety_buffer} meters

"""
    
    if conflicts:
        for i, conflict in enumerate(conflicts[:5]):
            summary_text += f"""
Conflict {i+1}:
  Time: {conflict.time.strftime('%H:%M:%S')}
  Location: ({conflict.location.x:.1f}, {conflict.location.y:.1f}, {conflict.location.z:.1f})
  Distance: {conflict.distance:.2f} m
  Between: {conflict.drone1_id} and {conflict.drone2_id}
"""
    
    ax3.text(0, 0.5, summary_text, fontsize=10, va='center')
    
    # Safety distances
    ax4 = fig.add_subplot(224)
    if conflicts:
        conflict_times = [(c.time - primary_trajectory.start_time).total_seconds() / 60 for c in conflicts]
        distances = [c.distance for c in conflicts]
        ax4.plot(conflict_times, distances, 'b-', label='Distance')
        ax4.axhline(y=safety_buffer, color='r', linestyle='--', label='Safety Buffer')
        ax4.set_xlabel('Time (minutes)')
        ax4.set_ylabel('Distance (m)')
        ax4.set_title('Distance Between Drones During Conflicts')
        ax4.grid(True)
        ax4.legend()
    else:
        ax4.text(0.5, 0.5, "No conflicts detected", ha='center', va='center', fontsize=14)
        ax4.set_title('Conflict Analysis')
    
    # Save the figure
    plt.tight_layout()
    filename = f"{scenario_name.lower().replace(' ', '_')}.png"
    plt.savefig(filename, dpi=200)
    plt.close(fig)
    print(f"Visualization saved as '{filename}'")

def test_scenario(scenario_name, primary_trajectory, other_trajectories, safety_buffer=20.0):
    """Test a specific scenario and create visualization"""
    print(f"\n=== Testing {scenario_name} ===")
    
    # Create service
    service = DeconflictionService(safety_buffer_distance=safety_buffer)
    
    # Register other trajectories
    for traj in other_trajectories:
        service.register_trajectory(traj)
    
    # Check for conflicts
    result = service.check_mission_safety(primary_trajectory)
    
    # Print results
    status = result["status"]
    conflicts = result["conflicts"]
    print(f"Mission safety status: {status}")
    print(f"Number of conflicts: {len(conflicts) if conflicts else 0}")
    
    if conflicts:
        # Print first few conflicts
        for i, conflict in enumerate(conflicts[:3]):
            print(f"Conflict {i+1}:")
            print(f"  Time: {conflict.time}")
            print(f"  Distance: {conflict.distance:.2f} m")
    
    # Create visualization
    create_visualization(scenario_name, primary_trajectory, other_trajectories, conflicts, safety_buffer)
    
    return result

def run_all_tests():
    """Run all test scenarios"""
    
    # Common time window
    base_start = datetime(2025, 4, 7, 10, 0, 0)
    base_end = datetime(2025, 4, 7, 10, 30, 0)
    
    # 1. Identical paths (guaranteed conflict)
    primary = DroneTrajectory(
        "primary",
        [Waypoint(0, 0, 100), Waypoint(100, 100, 100), Waypoint(200, 200, 100)],
        base_start,
        base_end
    )
    
    other = DroneTrajectory(
        "other",
        [Waypoint(0, 0, 100), Waypoint(100, 100, 100), Waypoint(200, 200, 100)],
        base_start,
        base_end
    )
    
    test_scenario("Identical Paths", primary, [other], 50.0)
    
    # 2. Close parallel paths
    primary = DroneTrajectory(
        "primary",
        [Waypoint(0, 0, 100), Waypoint(100, 100, 100), Waypoint(200, 200, 100)],
        base_start,
        base_end
    )
    
    other = DroneTrajectory(
        "other",
        [Waypoint(20, 0, 100), Waypoint(120, 100, 100), Waypoint(220, 200, 100)],
        base_start,
        base_end
    )
    
    test_scenario("Parallel Paths", primary, [other], 30.0)
    
    # 3. Crossing paths
    primary = DroneTrajectory(
        "primary",
        [Waypoint(0, 100, 100), Waypoint(100, 100, 100), Waypoint(200, 100, 100)],
        base_start,
        base_end
    )
    
    other = DroneTrajectory(
        "other",
        [Waypoint(100, 0, 100), Waypoint(100, 100, 100), Waypoint(100, 200, 100)],
        base_start,
        base_end
    )
    
    test_scenario("Crossing Paths", primary, [other], 20.0)
    
    # 4. Altitude separation
    primary = DroneTrajectory(
        "primary",
        [Waypoint(0, 0, 50), Waypoint(100, 100, 50), Waypoint(200, 200, 50)],
        base_start,
        base_end
    )
    
    other = DroneTrajectory(
        "other",
        [Waypoint(0, 0, 150), Waypoint(100, 100, 150), Waypoint(200, 200, 150)],
        base_start,
        base_end
    )
    
    test_scenario("Altitude Separation", primary, [other], 80.0)
    
    # 5. Temporal separation (same path, different times)
    primary = DroneTrajectory(
        "primary",
        [Waypoint(0, 0, 100), Waypoint(100, 100, 100), Waypoint(200, 200, 100)],
        base_start,
        base_start + timedelta(minutes=10)
    )
    
    other = DroneTrajectory(
        "other",
        [Waypoint(0, 0, 100), Waypoint(100, 100, 100), Waypoint(200, 200, 100)],
        base_start + timedelta(minutes=20),
        base_end
    )
    
    test_scenario("Temporal Separation", primary, [other], 20.0)
    
    # 6. Multiple drones
    primary = DroneTrajectory(
        "primary",
        [Waypoint(0, 0, 100), Waypoint(100, 100, 100), Waypoint(200, 200, 100)],
        base_start,
        base_end
    )
    
    drone1 = DroneTrajectory(
        "drone1",
        [Waypoint(0, 200, 100), Waypoint(100, 100, 100), Waypoint(200, 0, 100)],
        base_start,
        base_end
    )
    
    drone2 = DroneTrajectory(
        "drone2",
        [Waypoint(200, 0, 100), Waypoint(100, 100, 110), Waypoint(0, 200, 100)],
        base_start,
        base_end
    )
    
    drone3 = DroneTrajectory(
        "drone3",
        [Waypoint(300, 300, 90), Waypoint(350, 350, 90), Waypoint(400, 400, 90)],
        base_start,
        base_end
    )
    
    test_scenario("Multiple Drones", primary, [drone1, drone2, drone3], 25.0)
    
    # 7. Edge case: Brief encounter
    primary = DroneTrajectory(
        "primary",
        [Waypoint(0, 0, 100), Waypoint(100, 0, 100), Waypoint(200, 0, 100)],
        base_start,
        base_end
    )
    
    other = DroneTrajectory(
        "other",
        [Waypoint(100, -100, 100), Waypoint(100, 100, 100)],
        base_start + timedelta(minutes=10),
        base_start + timedelta(minutes=20)
    )
    
    test_scenario("Brief Encounter", primary, [other], 20.0)
    
    # 8. Edge case: Barely within safety buffer
    primary = DroneTrajectory(
        "primary",
        [Waypoint(0, 0, 100), Waypoint(100, 0, 100), Waypoint(200, 0, 100)],
        base_start,
        base_end
    )
    
    other = DroneTrajectory(
        "other",
        [Waypoint(0, 19, 100), Waypoint(100, 19, 100), Waypoint(200, 19, 100)],
        base_start,
        base_end
    )
    
    test_scenario("Barely Within Safety", primary, [other], 20.0)

if __name__ == "__main__":
    run_all_tests()
