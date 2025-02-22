# Autonomous-Drone-Navigation-System using HoverAviary

## Overview
This project is a **drone waypoint navigation system** built using the **HoverAviary** environment from `gym-pybullet-drones`. The system enables a drone to navigate through a sequence of predefined waypoints, visually marking each waypoint and updating its status upon reaching it. The drone's control mechanism is implemented using **DSLPIDControl**.


## Features
- **Waypoint visualization:** Displays waypoints as red spheres in the PyBullet environment.
- **Navigation control:** Uses **DSLPIDControl** to guide the drone to each waypoint.
- **Real-time status updates:** Changes waypoint color to green when reached.
- **Hovering mechanism:** Ensures the drone hovers at each waypoint for a minimum time.
- **Simulation parameters:** Configurable physics engine, control frequency, and GUI.

## Demo

🎥 ![Watch the Demo Video](https://github.com/ajaykrishna00-7/Autonomous_drone_navigation/blob/d283094f2df546ab377037cc99f21e807d3134f9/Sample_video.mp4")

## Installation
Clone this repository and navigate to the project directory:

```bash
git clone https://github.com/BargavanR/Autonomous-Drone-Navigation-Challenge.git
cd Drone-navigation-system
pip install -r Requirements.txt
```

## Usage
Run the script with default waypoints:

```bash
python Final_code.py
```

To specify custom waypoints, modify the `waypoints` list in the script:

```python
waypoints = [(0, 0, 1), (2, 2, 2), (3, 3, 2), (2, 2, 1)]
```

## Code Explanation
### `add_waypoint_markers(env, waypoints, sphere_size=0.13)`
Adds visual sphere markers and numerical labels to waypoints in the PyBullet simulation.

### `change_marker_color(waypoint_id, env)`
Updates the waypoint color to green when reached by the drone.

### `drone_path_execution(waypoints, ...)`
Main function that initializes the environment, sets up the drone controllers, and simulates the navigation process.

## Configuration
Modify parameters such as:

- **GUI Mode:** Set `gui=False` to run without visualization.
- **Simulation Duration:** Adjust `duration_sec` for longer or shorter runs.
- **Control Frequency:** Tune `control_freq_hz` for smoother navigation.

### Team Members:
- **Ajay Krishna P (Myself)**
- **Bargavan R**

## License
This project is open-source and available under the MIT License.
