
import gym
import pybullet as p
import numpy as np
import time
import sys
sys.path.append('/home/bargavan/VIT_submission/gym-pybullet-drones')
from gym_pybullet_drones.envs.HoverAviary import HoverAviary

output_limit= 5
#  PID Controller Class
class PIDController:
    def __init__(self, kp, ki, kd):
        self.kp = kp 
        self.ki = ki 
        self.kd = kd 
        self.prev_error = 0
        self.integral = 0
        self.last_time = time.time()

    def compute(self, target, current):
        error = target - current
        current_time = time.time()
        dt = current_time - self.last_time if self.last_time else 1.0
        self.last_time = current_time

        self.integral += error * dt
        derivative = (error - self.prev_error) / dt
        
        if output_limit:
            self.integral = np.clip(self.integral, -output_limit, output_limit)
        
        output = self.kp * error + self.ki * self.integral + self.kd * derivative
        self.prev_error = error
        return output
def run():
    # Initialize Environment
    env = HoverAviary(gui=True)

    state, info = env.reset()  
    #  Print state structure (for debugging)
    #print("State:", state)
    #print("State shape:", state[0].shape) 

    #  PID Controllers for X, Y, and Z control
    pid_x = PIDController(kp=0.001, ki=1, kd=0.60)  # Tune PID Gains
    pid_y = PIDController(kp=0.001, ki=1, kd=0.68)
    pid_z = PIDController(kp=0.001, ki=1, kd=0.70)  # More gain in Z for altitude control
    
    #  Target Setpoint (x, y, z)
    #x_set, y_set, z_set = 2, 0, 1 # Drone should reach this point
    #ajay try 2,0,0   2,2,0 and 0,2,0 and then increase the z point wise
    #  Simulation Loop
    x_set, y_set, z_set = int(input("Enter x: ")), int(input("Enter y: ")), int(input("Enter z: "))
    for i in range(1000):

        
        #  Extract Drone Position from State (Corrected)
        x, y, z = state[0][0], state[0][1], state[0][2]  # ✅ Extract correctly
        #if x_set==x:
            #break
         #   print(f"reached at {x}")
        
        #if y_set == y:
           # print("hi")
        #if z_set >=z:
         #   print(f"reached at {z}")

          #  break
            #$#print("hee")

            
        #  Compute PID outputs
        control_x = pid_x.compute(x_set, x)
        control_y = pid_y.compute(y_set, y)
        control_z = pid_z.compute(z_set, z)
        #print(f"Drone_error_adder=[{control_x,control_y,control_z}]")    
        #yaw = state[0][8]  # Extract yaw from state
        #print(f"Yaw Angle: {yaw}")

        # Convert PID outputs to rotor RPMs (Basic Mapping)
        base_thrust = 0.2 + 0.02 * abs(z_set - z)
        action = np.array([
        base_thrust + control_z - control_x - control_y,  
        base_thrust + control_z + control_x + control_y,  
        base_thrust + control_z + control_x - control_y,  
        base_thrust + control_z - control_x + control_y   
        ])
        print(f"action={action}")

        #print(f"action={action}")
        
        action = np.clip(action, 0.1, 0.3)  # Constrain RPMs between limits
        action = np.reshape(action, (1, 4))

        #  Apply Action & Get Updated State
        state, reward, done, info, _ = env.step(action)  

        #  Render Environment & Print Status
        env.render()
        #print(f"Step {i}: Position ({x:.2f}, {y:.2f}, {z:.2f}) -> Target ({x_set}, {y_set}, {z_set})")

        time.sleep(0.05)  # Slow down simulation for better visualization
        
        if done:
            break
    env.close()

run()

def stay():
    env = HoverAviary(gui=True)
    state, info = env.reset()  
