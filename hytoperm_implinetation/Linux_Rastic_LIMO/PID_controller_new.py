"""
This file is used for Persistent Monitoring By Justen Li w/ Professor Sean Andersson.

This file contains a PID controller for the RASTIC Limo robots. 
We send the robot two controls using this file, a steering angle
and a linear velocity. The steering angle is determined by the
cross-product between a unit vector from the robot in the direction 
of its heading and the vector from the robot to its target.
The linear velocity is determined by the dot-product of these 
vectors.
"""
import numpy as np
import math


class PID:
    # The PID object
    def __init__(self, steer_kp=1., steer_ki=0.0045, steer_kd=0 , vel_kp=1., vel_ki=0.0045, vel_kd=0.0017, dt=.01):
        # The init values are tunable, I settled on values that worked okay for me. Further tuning may be reasonable

        # The control gains for the steering input
        self.steer_kp = steer_kp
        self.steer_ki = steer_ki
        self.steer_kd = steer_kd

        # The control gains for the velocity input
        self.vel_kp = vel_kp
        self.vel_ki = vel_ki
        self.vel_kd = vel_kd    

        # How many seconds before a new control is published
        self.dt = dt

    def steerControl(self, x, x_d, heading_vector, prev_e, prev_int):
        # Finds the steering control
        
        # x[] = state[]
        # state[0, 0] = limo.position_x
        # state[1, 0] = limo.position_z

        # First find unit vector of robot in heading direction
        #heading_vector = np.array([[-np.cos(current_yaw)], [np.sin(current_yaw)]])

        unit_theta = heading_vector

        # Find vector from robot to desired point
        ref = x_d[:2]-x[:2]
      
        print("ref: ", ref)
        
        if np.linalg.norm(ref) != 0:  
            ref = ref/np.linalg.norm(ref) #Normalize

        e = np.cross(unit_theta.T, ref.T)[0]

        # For small angles, cross product ≈ sin(angle), angle (in radians)
        # This gives you the signed error directly

        print("e (cross_product): ", e)

        # # Normalize vectors
        # if np.linalg.norm(heading_dir) != 0:
        #     heading_unit = heading_dir / np.linalg.norm(heading_dir)
        # if np.linalg.norm(ref) < 0.01:  # Small threshold for "at target"
        #     ref = ref/np.linalg.norm(ref) #Normalize

        # # Calculate angle using atan2 for signed result
        # heading_angle = math.atan2(heading_unit[1], heading_unit[0])
        # target_angle = math.atan2(ref[1], ref[0])
        # print("heading_angle: ", heading_angle, "target_angle: ", target_angle)

        # # Calculate difference
        # angle_error = target_angle - heading_angle
        
        # # Normalize to [-π, π]
        # if angle_error > math.pi:
        #     angle_error -= 2 * math.pi
        # elif angle_error < -math.pi:
        #     angle_error += 2 * math.pi
        
        # e = angle_error
        # print("e: ", e)


        # # Dot product gives cosine of angle between vectors
        # # dot = 1.0 when perfectly aligned (0° angle)
        # # dot = 0.0 when perpendicular (90° angle) 
        # # dot = -1.0 when opposite (180° angle)
        # dot_product = np.dot(unit_theta.T, ref)[0, 0]
        # dot_product_angle = math.acos(abs(dot_product))  # Always positive angle
        # print("Dot Product_angle: ", math.degrees(dot_product_angle))

        # # Convert to error: larger angle = larger error
        # # When aligned (dot=1): error = 0
        # # When perpendicular (dot=0): error = 1  
        # # When opposite (dot=-1): error = 2
        # e = dot_product_angle
        
        # # Determine direction using cross product for sign
        # unit_theta_1d = unit_theta.flatten()
        # ref_1d = ref.flatten()
        
        # # Cross product for direction (positive = turn left, negative = turn right)
        # # Fixed cross product formula: cos*ref_z - sin*ref_x
        # cross_product = (unit_theta_1d[0] * ref_1d[1]) - (unit_theta_1d[1] * ref_1d[0])
        
        # # Apply sign to error based on turn direction
        # if cross_product < 0:
        #     e = -e  # Need to turn right (negative steering)
        
        # print("e: ", e)
        

        # # Steering error using cross product (2D vectors)
        # # Convert to 1D arrays for cross product calculation
        # unit_theta_1d = unit_theta.flatten()  # [cos(theta), sin(theta)]
        # ref_1d = ref.flatten()                # [ref_x, ref_z]
        
        # # Cross product gives signed angle error
        # #Because +x is right, +z is down. +e is turn left, -e is turn right, e is magnitude of error 
        # #e = sin(theta) * ref_x - cos(theta) * ref_z
        # #e = (unit_theta_1d[1] * ref_1d[0]) - (unit_theta_1d[0] * ref_1d[1])
        # e = (unit_theta_1d[0] * ref_1d[1]) - (unit_theta_1d[1] * ref_1d[0])


        # Update the integral of the error
        e_int = prev_int + e * self.dt
        # This ensures it does not blow up
        e_int = max(min(e_int, 0.5), -0.5)  # Increased range for angle errors

        # Approximate the derivative of error
        e_der = (e - prev_e) / self.dt

        # Find the steering control (radians/s)
        # u_steer > 0: Steer LEFT (positive steering angle/angular velocity)
        # u_steer < 0: Steer RIGHT (negative steering angle/angular velocity)
        # u_steer magnitude: how much to turn (is scaled by PID gains)
        u_steer = self.steer_kp * e + self.steer_ki * e_int + self.steer_kd * e_der
        u_steer = -u_steer 
        
        # Return control value and updated error and error integral values
        return u_steer, e, e_int


    # def speedControl(self, x, x_d, prev_e, prev_int):
    #     # Heading vector (unit vector in yaw direction)
    #     unit_theta = np.array([
    #         [math.cos(x[2, 0])],
    #         [math.sin(x[2, 0])]
    #     ])

    #     # Vector to goal
    #     ref = x_d[:2] - x[:2]
    #     dist = np.linalg.norm(ref)
    #     if dist > 0:
    #         ref = ref / dist

    #     # Alignment = dot product of heading and goal vector
    #     alignment = np.dot(unit_theta.T, ref)[0, 0]
    #     e = alignment

    #     # Clamp to prevent weird negative values
    #     if e < 0.3:
    #         e = 0.3  # ensure some push even when misaligned

    #     # PID terms
    #     e_int = prev_int + e * self.dt
    #     e_int = max(min(e_int, 0.5), -0.5)
    #     e_der = (e - prev_e) / self.dt

    #     # Raw PID velocity
    #     u_vel = self.vel_kp * e + self.vel_ki * e_int + self.vel_kd * e_der

    #     # ✅ Scale by alignment and allow min forward motion
    #     min_speed = 0.05
    #     max_speed = 1.0
    #     alignment_scale = max(alignment, 0.0)  # prevent negative scale
    #     scaled_vel = u_vel * alignment_scale

    #     # Clip to ensure robot always moves, but not too fast
    #     scaled_vel = max(min_speed, min(scaled_vel, max_speed))

    #     print(f"[speedControl] align: {alignment:.2f}, u_vel: {u_vel:.2f}, scaled: {scaled_vel:.2f}")
    #     return scaled_vel, e, e_int

    def speedControl(self, x, x_d, heading_vector, prev_e, prev_int, steer_error):
        # Finds the velocity control

        # First find unit vector form robot in direction of heading
        unit_theta = heading_vector
        
        # Find vector from robot to desired point
        ref = x_d[:2]-x[:2]

        #Because Ref is not normalized, the error will give the distance also. 
        # Velocity error is the dot product
        e = np.dot(unit_theta.T, ref)[0, 0]

        # if steer_error >= 0.14 and e !=0:
        #     u_vel = 0.15
        #     return u_vel, e, 0

        # if e < 0:
        #     e = 0

        # # Base velocity error from alignment
        # alignment = np.dot(unit_theta.T, ref)[0, 0]
        # base_error = alignment  # Positive when aligned
        
        # # Penalize velocity based on steering error
        # # Large steering error = slow down significantly
        # steer_penalty = abs(steer_error)
        
        # # Combined error: good alignment AND low steering error = high speed
        # #The 2.0 multiplier controls how much steering error affects speed (adjust as needed)
        # multiplier = 1.1
        # e = base_error - (multiplier * steer_penalty) # Adjust multiplier as needed
        
        # Update the integral of the error
        e_int = prev_int + e * self.dt
        e_int = max(min(e_int, 1.0), -1.0)

        # Approximate the derivative of error
        e_der = (e - prev_e) / self.dt

        # Find the VELOCITY control
        u_vel = self.vel_kp * e + self.vel_ki * e_int + self.vel_kd * e_der

        # # Gaussian-like speed reduction based on steering error
        # steer_sigma = 0.1  # Controls how aggressively to slow down (tune this)
        # speed_factor = np.exp(-(steer_error**2) / (2 * steer_sigma**2))
        
        # # Apply speed modulation
        # u_vel = u_vel * speed_factor

        return u_vel, e, e_int

    def coordinatedControl(self, x, x_d, prev_steer_e, prev_steer_int, prev_vel_e, prev_vel_int):
        # Calculate heading and target vectors
        unit_theta = np.array([
            [math.cos(x[2, 0])],
            [math.sin(x[2, 0])]
        ])
        
        ref = x_d[:2] - x[:2]
        distance = np.linalg.norm(ref)
        
        if distance > 0:
            ref = ref / distance

        # print("Ref: " ref)
        
        # Calculate what angle the robot SHOULD be facing
        target_angle = math.atan2(ref[1, 0], ref[0, 0])
        
        # Calculate the current robot angle
        current_angle = x[2, 0]
        
        # Find the shortest angular difference
        steer_error = target_angle - current_angle
        
        # Normalize to [-π, π]
        while steer_error > math.pi:
            steer_error -= 2 * math.pi
        while steer_error < -math.pi:
            steer_error += 2 * math.pi
        
        # DEBUG PRINTS
        print(f"[DEBUG] Current angle: {math.degrees(current_angle):.1f}°")
        print(f"[DEBUG] Target angle: {math.degrees(target_angle):.1f}°")
        print(f"[DEBUG] Steer error: {math.degrees(steer_error):.1f}°")
        
        angular_threshold = 0.3  # radians (~17 degrees)
        abs_steer_error = abs(steer_error)
        
        if abs_steer_error > angular_threshold:
            steer_gain_multiplier = 1.0
            speed_limit = 0.05
            print(f"[TURNING MODE] Need to turn {math.degrees(steer_error):.1f}°")
        else:
            steer_gain_multiplier = 0.5
            speed_limit = 0.8
            print(f"[DRIVING MODE] Minor correction {math.degrees(steer_error):.1f}°")
        
        # STEERING CONTROL
        u_steer = steer_gain_multiplier * self.steer_kp * steer_error
        
        # VELOCITY CONTROL
        alignment = np.dot(unit_theta.T, ref)[0, 0]
        u_vel = speed_limit * max(alignment, 0.1)
        u_vel = max(min(u_vel, speed_limit), 0.02)
        
        return u_steer, u_vel, steer_error, prev_steer_int, u_vel, prev_vel_int