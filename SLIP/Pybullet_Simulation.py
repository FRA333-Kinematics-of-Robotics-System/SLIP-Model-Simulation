import pybullet as p
import pybullet_data
import numpy as np
import time
from utils.Kinematics_utils import *

p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.81)
p.loadURDF("plane.urdf")

urdf_path = "urdf/SLIP_Model.urdf"

r, theta, phi, r_dot, theta_dot, phi_dot, r0, g = InitialState()

mass_x, mass_y, mass_z = InitialMassPosition()

spring_x, spring_y, spring_z = InitialSpringPosion()

spring_id = p.loadURDF(urdf_path, basePosition=[spring_x, spring_y, spring_z])

p.changeVisualShape(
    objectUniqueId=spring_id,
    linkIndex=-1,
    rgbaColor=[0, 0, 0, 1]
)

p.changeVisualShape(
    objectUniqueId=spring_id,
    linkIndex=0,
    rgbaColor=[1, 0, 0, 1] 
)

def UpdateSimulation(spring_id, r, spring_x, spring_y, spring_z, theta, phi):

    spring_index = 0

    new_position = r0 - r

    p.setJointMotorControl2(
        spring_id,
        spring_index,
        controlMode=p.POSITION_CONTROL,
        targetPosition=new_position,
    )

    p.resetBasePositionAndOrientation(
        spring_id,
        [spring_x , spring_y , spring_z],
        p.getQuaternionFromEuler([0, theta, phi])
    )

K_spring_id = p.addUserDebugParameter(" K Constant", 0, 1000, 100)
m_mass_id = p.addUserDebugParameter(" Mass", 0, 10, 1)

l_rest = r0
k_spring = p.readUserDebugParameter(K_spring_id)

x_i, y_i, z_i = mass_x, mass_y, mass_z
time_step = 1 / 700.0
dx, dy, dz = 1, 0, 0
simulation_time = 0
state = 1
previous_r = r
T_fight = 2 * dz / -g
T_stance = 2*np.pi* np.sqrt(m_mass_id/k_spring)

cameraDistance = 6
cameraYaw = 38
cameraPitch = -38
cameraTargetPosition = [0, 0, 0]

print("System initialized. Waiting for release...")

t_sim = 0
run = False

while True:
    keys = p.getKeyboardEvents()
    if p.B3G_RETURN in keys and keys[p.B3G_RETURN] & p.KEY_WAS_TRIGGERED:
        run = True
    
    if run:
        p.resetDebugVisualizerCamera(cameraDistance, cameraYaw, cameraPitch, cameraTargetPosition)
        if state == 1:
            phase = "stance"
            r, r_dot, theta, theta_dot, phi, phi_dot, mass_x, mass_y, mass_z = StancePhase(
                m_mass_id, k_spring, r0, g, time_step, r, r_dot, theta, theta_dot, phi, phi_dot ,spring_x, spring_y, spring_z
            )

            if r >= r0 :
                phase = "take-off"

                x_i, y_i, z_i, dx, dy, dz, spring_x, spring_y, spring_z, theta_i, T_fight, t_sim, state = StanceToFlight(
                    mass_x, mass_y, mass_z, theta, phi, r_dot, r, theta_dot, phi_dot)
                
                phase = "fight"

        elif (state == 2) & (phase == "fight"):
            mass_x, mass_y, mass_z, spring_x, spring_y, spring_z, theta = FlightPhase(
                x_i, y_i, z_i, r, theta, phi, theta_i, t_sim, g, dx, dy, dz, T_fight)
            
            t_sim += time_step

            if (spring_z <= 0) & (t_sim >= T_fight/2):
                phase = "touchdown"

                spring_z, x_i, y_i, z_i, dx, dy, dz, r_dot, phi_dot, theta_dot, T_stance, t_sim, state = FightToStance(
                    mass_x, mass_y, mass_z, r_dot, phi_dot, theta_dot, m_mass_id ,k_spring
                )

                phase = "stance"

        elif (state == 3) & (phase == "stance"):
            r, r_dot, theta, theta_dot, phi, phi_dot, mass_x, mass_y, mass_z = StancePhase(
                m_mass_id, k_spring, r0, g, time_step, r, r_dot, theta, theta_dot, phi, phi_dot, spring_x, spring_y, spring_z
            )
            t_sim += time_step
            if (r >= r0) & (t_sim >= T_stance/2):
                phase = "take-off"

                x_i, y_i, z_i, dx, dy, dz, spring_x, spring_y, spring_z, theta_i, T_fight, t_sim, state = StanceToFlight(
                    mass_x, mass_y, mass_z, theta, phi, r_dot, r, theta_dot, phi_dot)
                
                phase = "fight"

        UpdateSimulation(spring_id, r, spring_x, spring_y, spring_z, theta, phi)
        cameraTargetPosition = [spring_x, spring_y, 0]
        
        print(f"Phase: {phase}")
        print(f"Mass Position: x={mass_x}, y={mass_y}, z={mass_z}", f"Spring Base: x={spring_x}, y={spring_y}, z={spring_z}")
        print(f"theta: {theta}, phi: {phi}")
        print(f"length spring: r= {r}")
        #print(f"time_phase: {t_sim}", f"T_fight: {T_fight}", f"T_stance: {T_stance}")

        print(f"Angulavelocity: r_dot={r_dot}, theta_dot={theta_dot}, phi_dot={phi_dot}")
        print("\n")
        # print(f"velocity: x_dot={dx}, y_dot={dy}, z_dot={dz}")
        
        simulation_time += time_step

        p.stepSimulation()
        time.sleep(time_step)
