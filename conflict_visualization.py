import numpy as np
import math
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.patches as patches
from datetime import datetime, timedelta

class Waypoint:
    def __init__(self, x, y, z=None, time=None):
        self.x = x
        self.y = y
        self.z = z if z is not None else 0
        self.time = time  # Optional specific time for this waypoint
        
    def distance_to(self, other_waypoint):
        """Calculate Euclidean distance to another waypoint"""
        if hasattr(other_waypoint, 'z'):
            return math.sqrt((self.x - other_waypoint.x)**2 + 
                           (self.y - other_waypoint.y)**2 + 
                           (self.z - other_waypoint.z)**2)
        else:
            return math.sqrt((self.x - other_waypoint.x)**2 + 
                           (self.y - other_waypoint.y)**2)
    
    def __repr__(self):
        if hasattr(self, 'z') and self.z is not None:
            return f"Waypoint(x={self.x}, y={self.y}, z={self.z}, time={self.time})"
        else:
            return f"Waypoint(x={self.x}, y={self.y}, time={self.time})"

class DroneTrajectory:
    def __init__(self, drone_id, waypoints, start_time, end_time=None):
        self.drone_id = drone_id
        self.waypoints = waypoints
        self.start_time = start_time
        
        # If end_time is not provided, use the last waypoint's time if available
        if end_time is None and waypoints[-1].time is not None:
            self.end_time = waypoints[-1].time
        else:
            self.end_time = end_time
            
        # Calculate estimated times for each waypoint
        self.waypoint_times = self._calculate_waypoint_times()
        
    def _calculate_waypoint_times(self):
        """Calculate estimated arrival times at each waypoint"""
        # If all waypoints already have times, use those
        if all(wp.time is not None for wp in self.waypoints):
            return [wp.time for wp in self.waypoints]
        
        total_distance = 0
        segments = []
        
        # Calculate total distance and segment distances
        for i in range(len(self.waypoints) - 1):
            dist = self.waypoints[i].distance_to(self.waypoints[i+1])
            total_distance += dist
            segments.append(dist)
        
        # Calculate total mission duration
        total_duration = (self.end_time - self.start_time).total_seconds()
        
        # Calculate times based on proportional distances
        times = [self.start_time]
        elapsed = 0
        
        for i, segment in enumerate(segments):
            # Proportion of total distance = proportion of total time
            segment_time = (segment / total_distance) * total_duration if total_distance > 0 else 0
            elapsed += segment_time
            times.append(self.start_time + timedelta(seconds=elapsed))
            
        return times
    
    def get_position_at_time(self, query_time):
        """Interpolate drone position at a specific time"""
        # Check if time is within trajectory timeframe
        if query_time < self.waypoint_times[0] or query_time > self.waypoint_times[-1]:
            return None
            
        # Find the waypoint segment containing this time
        for i in range(len(self.waypoint_times) - 1):
            t1 = self.waypoint_times[i]
            t2 = self.waypoint_times[i+1]
            
            if t1 <= query_time <= t2:
                wp1 = self.waypoints[i]
                wp2 = self.waypoints[i+1]
                
                # Linear interpolation factor
                factor = ((query_time - t1).total_seconds() / 
                         (t2 - t1).total_seconds()) if t2 != t1 else 0
                
                x = wp1.x + factor * (wp2.x - wp1.x)
                y = wp1.y + factor * (wp2.y - wp1.y)
                z = wp1.z + factor * (wp2.z - wp1.z)
                
                return Waypoint(x, y, z, query_time)
                
        return None
        
    def __repr__(self):
        return f"DroneTrajectory(drone_id={self.drone_id}, waypoints={len(self.waypoints)}, " \
               f"start={self.start_time}, end={self.end_time})"

class Conflict:
    def __init__(self, drone1_id, drone2_id, location, time, distance):
        self.drone1_id = drone1_id
        self.drone2_id = drone2_id
        self.location = location  # Waypoint object at conflict location
        self.time = time  # Time of conflict
        self.distance = distance  # Distance between drones at conflict
        
    def __repr__(self):
        return f"Conflict(drones={self.drone1_id} & {self.drone2_id}, " \
               f"location={self.location}, time={self.time}, distance={self.distance})"

class DeconflictionService:
    def __init__(self, safety_buffer_distance=10.0, time_step=timedelta(seconds=1)):
        self.safety_buffer = safety_buffer_distance
        self.time_step = time_step
        self.registered_trajectories = []
    
    def register_trajectory(self, trajectory):
        """Register a drone trajectory in the system"""
        self.registered_trajectories.append(trajectory)
        
    def check_mission_safety(self, mission_trajectory):
        """Check if the mission trajectory is safe against all registered trajectories"""
        conflicts = []
        
        for other_trajectory in self.registered_trajectories:
            if other_trajectory.drone_id == mission_trajectory.drone_id:
                continue  # Skip comparing to self
                
            # Find conflicts between these two trajectories
            trajectory_conflicts = self._find_conflicts(mission_trajectory, other_trajectory)
            conflicts.extend(trajectory_conflicts)
                
        return {
            "status": "clear" if not conflicts else "conflict detected",
            "conflicts": conflicts
        }
    
    def _find_conflicts(self, trajectory1, trajectory2):
        """Find all conflicts between two trajectories"""
        conflicts = []
        
        # Determine overlapping time range
        start_time = max(trajectory1.waypoint_times[0], trajectory2.waypoint_times[0])
        end_time = min(trajectory1.waypoint_times[-1], trajectory2.waypoint_times[-1])
        
        # If no temporal overlap, no conflicts possible
        if start_time > end_time:
            return conflicts
            
        # Check at regular time intervals
        current_time = start_time
        while current_time <= end_time:
            # Get interpolated positions at this time
            pos1 = trajectory1.get_position_at_time(current_time)
            pos2 = trajectory2.get_position_at_time(current_time)
            
            if pos1 and pos2:  # Both drones are active at this time
                distance = pos1.distance_to(pos2)
                
                if distance < self.safety_buffer:
                    # Calculate midpoint for conflict location
                    conflict_x = (pos1.x + pos2.x) / 2
                    conflict_y = (pos1.y + pos2.y) / 2
                    conflict_z = (pos1.z + pos2.z) / 2
                    
                    conflict = Conflict(
                        drone1_id=trajectory1.drone_id,
                        drone2_id=trajectory2.drone_id,
                        location=Waypoint(conflict_x, conflict_y, conflict_z, current_time),
                        time=current_time,
                        distance=distance
                    )
                    conflicts.append(conflict)
                    
            current_time += self.time_step
                    
        return conflicts

class TrajectoryVisualizer:
    def __init__(self, is_3d=False):
        self.is_3d = is_3d
        self.fig = plt.figure(figsize=(12, 10))
        if is_3d:
            self.ax = self.fig.add_subplot(111, projection='3d')
        else:
            self.ax = self.fig.add_subplot(111)
        
        # Set up colors for trajectories
        self.colors = ['blue', 'green', 'red', 'purple', 'orange', 'cyan', 'magenta', 'yellow']
        
    def plot_trajectory(self, trajectory, color=None, label=None):
        """Plot a single trajectory"""
        if color is None:
            color = self.colors[hash(trajectory.drone_id) % len(self.colors)]
            
        x_coords = [wp.x for wp in trajectory.waypoints]
        y_coords = [wp.y for wp in trajectory.waypoints]
        
        if self.is_3d:
            z_coords = [wp.z for wp in trajectory.waypoints]
            self.ax.plot(x_coords, y_coords, z_coords, 'o-', color=color, 
                         label=label or f"Drone {trajectory.drone_id}")
            self.ax.set_zlabel('Altitude')
        else:
            self.ax.plot(x_coords, y_coords, 'o-', color=color, 
                         label=label or f"Drone {trajectory.drone_id}")
            
        # Mark waypoints
        if self.is_3d:
            self.ax.scatter(x_coords, y_coords, z_coords, color=color, s=50)
        else:
            self.ax.scatter(x_coords, y_coords, color=color, s=50)
            
    def highlight_conflict(self, conflict, safety_buffer=None):
        """Highlight a conflict area"""
        location = conflict.location
        radius = safety_buffer or conflict.distance / 2
        
        if self.is_3d:
            # Create a sphere
            u, v = np.mgrid[0:2*np.pi:20j, 0:np.pi:10j]
            x = location.x + radius * np.cos(u) * np.sin(v)
            y = location.y + radius * np.sin(u) * np.sin(v)
            z = location.z + radius * np.cos(v)
            self.ax.plot_surface(x, y, z, color='red', alpha=0.3)
            
            # Add text annotation
            self.ax.text(location.x, location.y, location.z + radius, 
                         f"Conflict at {location.time.strftime('%H:%M:%S')}\n"
                         f"Distance: {conflict.distance:.2f}",
                         color='black', fontsize=8)
        else:
            # Create a circle
            circle = plt.Circle((location.x, location.y), radius, color='red', alpha=0.3)
            self.ax.add_patch(circle)
            
            # Add text annotation
            self.ax.annotate(f"Conflict at {location.time.strftime('%H:%M:%S')}\n"
                            f"Distance: {conflict.distance:.2f}",
                            (location.x, location.y), 
                            textcoords="offset points", 
                            xytext=(0, 10), 
                            ha='center', fontsize=8)
    
    def save_static_visualization(self, filename="trajectory_visualization.png"):
        """Save the visualization to a file instead of displaying it"""
        self.ax.set_xlabel('X')
        self.ax.set_ylabel('Y')
        self.ax.legend()
        plt.title('UAV Trajectory Visualization')
        plt.grid(True)
        plt.tight_layout()
        plt.savefig(filename)
        plt.close()
        print(f"Visualization saved to {filename}")
            
    def show(self):
        """Display the plot"""
        self.ax.set_xlabel('X')
        self.ax.set_ylabel('Y')
        self.ax.legend()
        plt.title('UAV Trajectory Visualization')
        plt.grid(True)
        plt.tight_layout()
        plt.show()

# Main interface function
def query_deconfliction_service(mission_waypoints, mission_start_time, mission_end_time, 
                                deconfliction_service, is_3d=False, visualize=True):
    """
    Query the deconfliction service to check if a planned mission is safe.
    
    Args:
        mission_waypoints: List of Waypoint objects defining the mission
        mission_start_time: Start time of the mission window
        mission_end_time: End time of the mission window
        deconfliction_service: The deconfliction service instance
        is_3d: Boolean indicating if this is a 3D mission
        visualize: Boolean indicating if visualization should be shown
        
    Returns:
        Dict with status ("clear" or "conflict detected") and conflict details
    """
    # Create mission trajectory
    mission_trajectory = DroneTrajectory(
        drone_id="primary",
        waypoints=mission_waypoints,
        start_time=mission_start_time,
        end_time=mission_end_time
    )
    
    # Query the deconfliction service
    result = deconfliction_service.check_mission_safety(mission_trajectory)
    
    # Visualize if requested
    if visualize:
        visualizer = TrajectoryVisualizer(is_3d=is_3d)
        visualizer.plot_trajectory(mission_trajectory, color='green', label='Primary Mission')
        
        # Plot other drone trajectories
        for trajectory in deconfliction_service.registered_trajectories:
            visualizer.plot_trajectory(trajectory)
            
        # Highlight conflicts
        for conflict in result["conflicts"]:
            visualizer.highlight_conflict(conflict)
        
        # Save the visualization to a file
        scenario_name = "mission"
        visualizer.save_static_visualization(f"{scenario_name}_visualization.png")
        print(f"Visualization saved to {scenario_name}_visualization.png")
            
        # Show the visualization
        visualizer.show()
    
    return result

# No example test function - use test_all_scenarios.py instead