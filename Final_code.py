# Import necessary libraries
import os
import time
import numpy as np  # For mathematical operations and array handling
import pybullet as pb  # PyBullet for physics simulation

# Import required modules from gym-pybullet-drones
from gym_pybullet_drones.utils.enums import DroneModel, Physics  # Drone model types and physics engine selection
from gym_pybullet_drones.envs.HoverAviary import HoverAviary  # Environment where drones hover and move
from gym_pybullet_drones.control.DSLPIDControl import DSLPIDControl  # PID controller for drone stabilization
from gym_pybullet_drones.utils.Logger import Logger  # Logs simulation data (not used in this script)
from gym_pybullet_drones.utils.utils import sync, str2bool  # Utilities for synchronization and boolean conversion


def add_waypoint_markers(env, targets, marker_size=0.13):
    """Creates visual markers (spheres) in PyBullet to represent waypoints."""

    marker_ids = []  # List to store the IDs of waypoint markers

    # Iterate over each waypoint and create a visual marker
    for index, position in enumerate(targets):
        # Create a spherical visual shape for the waypoint marker
        marker_visual = pb.createVisualShape(
            shapeType=pb.GEOM_SPHERE,  # Defines shape type as a sphere
            radius=marker_size,  # Sets the radius of the sphere
            rgbaColor=[1, 0, 0, 0],  # Sets color to white (fully invisible)
            physicsClientId=env.getPyBulletClient()  # Links to the PyBullet simulation
        )

        # Attach the visual marker to a MultiBody object to place it in the simulation
        marker_id = pb.createMultiBody(
            baseVisualShapeIndex=marker_visual,  # Assigns the visual shape created
            basePosition=position,  # Sets the position of the waypoint marker
            physicsClientId=env.getPyBulletClient()  # Links to the simulation
        )

        marker_ids.append(marker_id)  # Store the marker ID for future reference

        # Add a numerical label above the waypoint for easy identification
        pb.addUserDebugText(
            f"{index+1}",  # Converts waypoint index to text (1-based)
            position + np.array([0, 0, marker_size + 0.1]),  # Position above the sphere
            textSize=1.5,  # Set font size
            textColorRGB=[1, 1, 1],  # Text color (white)
            physicsClientId=env.getPyBulletClient()  # Links to simulation
        )
    
    return marker_ids  # Return list of waypoint marker IDs


def change_marker_color(marker_id, env):
    """Updates the waypoint marker color to green when reached."""
    
    pb.changeVisualShape(
        marker_id, -1,  # Apply change to the marker
        rgbaColor=[0, 1, 0, 1],  # Change to green color (fully visible)
        physicsClientId=env.getPyBulletClient()  # Links to simulation
    )


def drone_path_execution(checkpoints, drone_model=DroneModel("cf2x"), fleet_size=1, physics_engine=Physics("pyb"),
                          enable_gui=True, record_sim=False, graph_output=True, debug_ui=False, enable_obstacles=False,
                          sim_rate=240, ctrl_rate=48, sim_time=20, colab_mode=False):
    """Handles drone navigation through waypoints using HoverAviary environment."""
    
    # Set initial positions of drones (starting at height 0.1m)
    START_POSITIONS = np.array([[0, 0, 0.1] for _ in range(fleet_size)])
    
    # Set initial orientations (roll, pitch, yaw) to zero
    START_ORIENTATIONS = np.array([[0, 0, 0] for _ in range(fleet_size)])

    # Initialize the drone simulation environment
    sim_env = HoverAviary(
        drone_model=drone_model,  # Type of drone being used
        initial_xyzs=START_POSITIONS,  # Initial drone positions
        initial_rpys=START_ORIENTATIONS,  # Initial drone orientations
        physics=physics_engine,  # Defines physics engine (PyBullet)
        pyb_freq=sim_rate,  # Simulation update rate
        ctrl_freq=ctrl_rate,  # Drone control frequency
        gui=enable_gui,  # Enable or disable GUI visualization
        record=record_sim  # Enable or disable recording
    )

    # Override default observation method to get drone state
    sim_env._computeObs = lambda: np.array([sim_env._getDroneStateVector(0) for _ in range(fleet_size)])

    # Override action preprocessing to ensure valid RPM values
    sim_env._preprocessAction = lambda act: np.array([np.clip(act[i, :], 0, sim_env.MAX_RPM) for i in range(fleet_size)])
    
    # Store original step function and override it
    default_step_function = sim_env.step
    def new_step_function(act):
        """Ensures actions are preprocessed before applying them to the simulation."""
        return default_step_function(sim_env._preprocessAction(act))
    
    sim_env.step = new_step_function  # Replace step method

    # Get PyBullet client ID for simulation control
    PHYSICS_CLIENT = sim_env.getPyBulletClient()

    # Initialize PID controllers for drones
    controllers = [DSLPIDControl(drone_model=drone_model) for _ in range(fleet_size)]
    
    # Initialize drone actions (all motors off initially)
    actions = np.zeros((fleet_size, 4))

    # Create visual markers for waypoints
    waypoint_markers = add_waypoint_markers(sim_env, checkpoints)

    START_TIME = time.time()  # Store the simulation start time
    waypoint_progress = np.zeros(fleet_size, dtype=int)  # Track each drone's current waypoint
    total_waypoints = len(checkpoints)  # Total number of waypoints

    hover_duration = np.zeros(fleet_size)  # Track hover time at waypoints
    DISTANCE_THRESHOLD = 0.15  # Minimum distance to detect waypoint arrival
    MIN_HOVER_TIME = 0.5  # Hover time for regular waypoints
    FINAL_HOVER_TIME = 2.0  # Extended hover time for final waypoint
    
    # Main simulation loop
    for frame in range(0, int(sim_time * sim_env.CTRL_FREQ)):
        # Step the simulation environment and get observations
        observations, _, _, _, _ = sim_env.step(actions)
        
        all_complete = True  # Flag to check if all drones have reached their waypoints
        
        for d in range(fleet_size):
            if waypoint_progress[d] < total_waypoints:
                all_complete = False  # At least one drone has waypoints left
                
                # Get current waypoint position
                target_location = checkpoints[waypoint_progress[d]]
                
                # Compute the required control action
                actions[d, :], _, _ = controllers[d].computeControlFromState(
                    control_timestep=sim_env.CTRL_TIMESTEP,
                    state=observations[d],
                    target_pos=target_location,
                    target_rpy=START_ORIENTATIONS[d]
                )

                # Calculate distance to the waypoint
                distance_to_target = np.linalg.norm(observations[d][:3] - target_location)

                # If drone is close enough, increase hover time counter
                if distance_to_target < DISTANCE_THRESHOLD:
                    hover_duration[d] += sim_env.CTRL_TIMESTEP
                else:
                    hover_duration[d] = 0  # Reset hover time if drone moves away

                # Determine the required hover time
                required_hover = FINAL_HOVER_TIME if waypoint_progress[d] == total_waypoints - 1 else MIN_HOVER_TIME

                # Move to next waypoint if hover time is satisfied
                if hover_duration[d] >= required_hover:
                    change_marker_color(waypoint_markers[waypoint_progress[d]], sim_env)
                    waypoint_progress[d] += 1
                    hover_duration[d] = 0

        sim_env.render()  # Render the simulation
        if enable_gui:
            sync(frame, START_TIME, sim_env.CTRL_TIMESTEP)  # Synchronize simulation timing

        if all_complete:  # If all drones are finished, exit loop
            print("All drones have completed their waypoints. Ending simulation.")
            break

    sim_env.close()  # Close the simulation


# Run simulation with predefined waypoints
if __name__ == "__main__":
    path_checkpoints = [(0, 0, 1), (2, 1, 2), (3, 1, 2), (2, 2, 1)]
    drone_path_execution(path_checkpoints)
