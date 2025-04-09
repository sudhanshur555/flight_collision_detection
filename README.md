# UAV Strategic Deconfliction System

A system for verifying whether a drone's planned waypoint mission is safe to execute in shared airspace by checking for conflicts in both space and time against the simulated flight paths of multiple other drones.

## Overview

This project implements a strategic deconfliction system that:
- Verifies if a drone's planned mission can be safely executed in shared airspace
- Checks for spatial and temporal conflicts with other drone flight paths
- Visualizes trajectories and potential conflicts in 3D space and time
- Provides detailed explanations when conflicts are detected

## Requirements

- Python 3.12.3
- Dependencies:
  - numpy
  - matplotlib

## Installation

1. Clone the repository or download the files:
```bash
mkdir test_env
cd test_env
python -m venv test_enviroment
source test_enviroment/bin/activate
git clone https://github.com/sudhanshur555/flight_collision_detection.git
cd flight_collision_detection

```

2. Install dependencies:
```bash
pip install "numpy<2.0.0" matplotlib==3.8.2
```

## File Structure

The project consists of the following key files:

- `conflict_visualization.py` - Core implementation with all classes and algorithms
- `test_all_scenarios.py` - Comprehensive test suite with visualizations

## Usage

### Running the Test Scenarios

Run the comprehensive test suite to evaluate the system with various scenarios:

```bash
python test_all_scenarios.py
```

This will run 8 different test scenarios that demonstrate various aspects of the deconfliction system:
1. Identical Paths - Two drones following exactly the same path
2. Parallel Paths - Drones moving in the same direction with slight offset
3. Crossing Paths - Drones whose paths intersect at a specific point
4. Altitude Separation - Drones with the same horizontal path but different altitudes
5. Temporal Separation - Drones using the same path but at different times
6. Multiple Drones - One primary drone with multiple other drones in the airspace
7. Brief Encounter - Drones that only come close for a brief moment
8. Barely Within Safety - Testing the edge case of being just within the safety buffer

For each scenario, the system will:
- Check for conflicts between trajectories
- Generate visualizations showing the drone paths and any conflicts
- Save these visualizations as PNG files
- Print results to the console

## Core Components

### Waypoint

Represents a point in 3D space with (x, y, z) coordinates and optional time.

```python
waypoint = Waypoint(100, 200, 50)  # x=100, y=200, altitude=50
```

### DroneTrajectory

Represents a drone's flight path as a series of waypoints with timing information.

```python
trajectory = DroneTrajectory(
    drone_id="drone1",
    waypoints=[waypoint1, waypoint2, waypoint3],
    start_time=datetime(2025, 4, 7, 10, 0, 0),
    end_time=datetime(2025, 4, 7, 10, 30, 0)
)
```

### DeconflictionService

Central service for registering trajectories and checking for conflicts.

```python
service = DeconflictionService(safety_buffer_distance=20.0)
service.register_trajectory(other_trajectory)
result = service.check_mission_safety(primary_trajectory)
```

### TrajectoryVisualizer

Handles visualization of trajectories and conflicts in static form.

```python
visualizer = TrajectoryVisualizer(is_3d=True)
visualizer.plot_trajectory(trajectory)
visualizer.highlight_conflict(conflict)
visualizer.save_static_visualization("output.png")
```

## Creating Your Own Scenarios

Here's how to create and test custom scenarios:

```python
from datetime import datetime, timedelta
from deconfliction import Waypoint, DroneTrajectory, DeconflictionService

# Create a deconfliction service
service = DeconflictionService(safety_buffer_distance=20.0)

# Define primary mission details
primary_waypoints = [
    Waypoint(0, 0, 100),
    Waypoint(100, 100, 150),
    Waypoint(200, 200, 100)
]
mission_start = datetime(2025, 4, 7, 10, 0, 0)
mission_end = datetime(2025, 4, 7, 10, 30, 0)
primary_trajectory = DroneTrajectory("primary", primary_waypoints, mission_start, mission_end)

# Define another drone's trajectory
other_waypoints = [
    Waypoint(50, 50, 110),
    Waypoint(150, 150, 140),
    Waypoint(250, 250, 110)
]
other_trajectory = DroneTrajectory("other", other_waypoints, mission_start, mission_end)
service.register_trajectory(other_trajectory)

# Check for conflicts
result = service.check_mission_safety(primary_trajectory)

# Print results
print(f"Mission status: {result['status']}")
if result['conflicts']:
    print(f"Found {len(result['conflicts'])} conflicts")
    for conflict in result['conflicts'][:3]:  # Show first 3
        print(f"Conflict at {conflict.time}, distance: {conflict.distance}m")
```

## Output Examples

The system generates detailed outputs for each scenario:

1. **Console Output**:
   - Mission safety status ("clear" or "conflict detected")
   - Number of conflicts detected
   - Details of each conflict (time, location, distance, drones involved)

2. **Visualization Files**:
   - 3D trajectory visualization
   - 2D top-down view
   - Conflict summary information
   - Distance graphs for conflict scenarios

## Troubleshooting

- **Visualization issues**: If 3D visualization has problems, try using 2D mode by setting `is_3d=False`
- **Performance issues**: For scenarios with many conflicts, modify the time step in `DeconflictionService` to improve performance
