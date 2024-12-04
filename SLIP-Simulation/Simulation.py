# import Library
import pybullet as p
import pybullet_data
import numpy as np
import time
from Utils import *
import os

p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.81)
p.loadURDF("plane.urdf")

urdf_path = "urdf\SLIP_Model.urdf"

r, theta, phi, r_dot, theta_dot, phi_dot, r0 = InitialState()

mass_x, mass_y, mass_z = InitialMassPosition()

spring_x, spring_y, spring_z = InitialSpringPosion()

spring_id = p.loadURDF(urdf_path, basePosition=[spring_x, spring_y, spring_z])

#Change color of base
p.changeVisualShape(
    objectUniqueId=spring_id,
    linkIndex=-1,  # หรือกำหนด link index
    rgbaColor=[0, 0, 0, 1]  # กำหนดสีแดง
)

#Change color of mass
p.changeVisualShape(
    objectUniqueId=spring_id,
    linkIndex=0,  # หรือกำหนด link index
    rgbaColor=[1, 0, 0, 1] 
)

#Change color of spring
p.changeVisualShape(
    objectUniqueId=spring_id,
    linkIndex=1,  # หรือกำหนด link index
    rgbaColor=[1, 0, 0, 1] 
)

# Debug sliders
K_spring_id = p.addUserDebugParameter("K Constant", 0, 1000, 100)
m_mass_id = p.addUserDebugParameter("Mass", 0, 10, 1)

l_rest = r0
k_spring = p.readUserDebugParameter(K_spring_id)  # N/m

x_i, y_i, z_i = 0, 0, 0

# Constants for the simulation
g = -9.81          # Gravitational acceleration (m/s^2)
time_step = 1 / 240.0  # Simulation time step (seconds)
dx, dy, dz = 1, 0, 0
simulation_time = 0
state = 1
previous_r = r  # or use r0 if r0 is the initial spring length
T_fight = 2 * dz / -g
T_stance = 2*np.pi* np.sqrt(m_mass_id/k_spring)

# เริ่มสถานะใน Initial State
print("System initialized. Waiting for release...")
input("Press Enter to release the system.")
t_sim = 0

while True:

    # initial state
    if state == 1:
        phase = "stance"
        r, r_dot, theta, theta_dot, phi, phi_dot, mass_x, mass_y, mass_z = StancePhase(
            m_mass_id, k_spring, r0, g, time_step, r, r_dot, theta, theta_dot, phi, phi_dot
        )

        if r >= r0 :
            phase = "take-off"

            x_i, y_i, z_i, dx, dy, dz, spring_x, spring_y, spring_z, theta_i, T_fight, t_sim, state = StanceToFlight(
                mass_x, mass_y, mass_z, theta, phi, r_dot, r, theta_dot, phi_dot)
            
            phase = "fight"

    # Fight phase
    elif (state == 2) & (phase == "fight"):
        mass_x, mass_y, mass_z, spring_x, spring_y, spring_z, theta = FightPhase(
            x_i, y_i, z_i, r, theta, phi, theta_i, t_sim, g, dx, dy, dz, T_fight)
        
        t_sim += time_step

        if (spring_z <= 0) & (t_sim >= T_fight/2):
            phase = "touchdown"
            
            spring_z, x_i, y_i, z_i, dx, dy, dz, r_dot, phi_dot, theta_dot, theta, T_stance, t_sim, state = FightToStance(
                mass_x, mass_y, mass_z, r_dot, phi_dot, theta_dot, theta_i, m_mass_id ,k_spring
            )

            phase = "stance"

    # Stance phase
    elif (state == 3) & (phase == "stance"):
        r, r_dot, theta, theta_dot, phi, phi_dot, mass_x, mass_y, mass_z = StancePhase(
            m_mass_id, k_spring, r0, g, time_step, r, r_dot, theta, theta_dot, phi, phi_dot
        )
        t_sim += time_step
        if (r >= r0) & (t_sim >= T_stance/2):
            phase = "take-off"

            x_i, y_i, z_i, dx, dy, dz, spring_x, spring_y, spring_z, theta_i, T_fight, t_sim, state = StanceToFlight(
                mass_x, mass_y, mass_z, theta, phi, r_dot, r, theta_dot, phi_dot)
            
            phase = "fight"

    # Update simulation
    UpdateSimulation(spring_id, r, spring_x, spring_y, spring_z, theta, phi, theta_dot)

    print(f"Phase: {phase}")
    print(f"Mass Position: x={mass_x}, y={mass_y}, z={mass_z}", f"Spring Base: x={spring_x}, y={spring_y}, z={spring_z}")
    print(f"theta: {theta}")
    print(f"length spring: r= {r}")
    #print(f"time_phase: {t_sim}", f"T_fight: {T_fight}", f"T_stance: {T_stance}")
    print(f"Angulavelocity: r_dot={r_dot}, theta_dot={theta_dot}, phi_dot={phi_dot}")
    print("\n")
    # print(f"velocity: x_dot={dx}, y_dot={dy}, z_dot={dz}")
    
    simulation_time += time_step

    # Step simulation
    p.stepSimulation()
    time.sleep(time_step)
