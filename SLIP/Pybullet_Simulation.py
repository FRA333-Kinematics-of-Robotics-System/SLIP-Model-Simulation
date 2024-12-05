import pybullet as p
import pybullet_data
import numpy as np
import time

from utils.Kinematics_utils import *
from utils.gui_utils import *

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


x_i, y_i, z_i = mass_x, mass_y, mass_z
time_step = 1 / 240.0
dx, dy, dz = 1, 0, 0
state = 1

theta_i = theta

cameraDistance = 6
cameraYaw = 38
cameraPitch = -38
cameraTargetPosition = [0, 0, 0]

print("System initialized. Waiting for release...")

Kp = 37
Kd = 20

t_sim = 0
count = 0
ui = UI()

run = False

while True:

    if not run:
        ui.update()

        m_value, k_value, r_dot_value, theta_dot_value, phi_dot_value, count_value = ui.get_slider_values()

        m, k, r_dot, theta_dot, phi_dot, count_max = m_value, k_value, r_dot_value, theta_dot_value, phi_dot_value, count_value

        T_flight = 2 * dz / -g
        T_stance = 2*np.pi* np.sqrt(m/k)

        print(f"Mass: {m_value}, Spring constant: {k_value}, dr: {r_dot_value}, dtheta: {theta_dot_value}, dphi: {phi_dot_value}, Count: {count_value}")
        
        mouse_click = pygame.mouse.get_pressed()

        keys = pygame.key.get_pressed()

        if ui.check_button_click(350, 80, 100, 40):
            ui.m_value = min(ui.m_value + ui.m_step, ui.m_max)
        if ui.check_button_click(50, 80, 100, 40):
            ui.m_value = max(ui.m_value - ui.m_step, ui.m_min)
        
        if ui.check_button_click(350, 180, 100, 40):
            ui.k_value = min(ui.k_value + ui.k_step, ui.k_max)
        if ui.check_button_click(50, 180, 100, 40):
            ui.k_value = max(ui.k_value - ui.k_step, ui.k_min)
        
        if ui.check_button_click(350, 280, 100, 40):
            ui.r_dot_value = min(ui.r_dot_value + ui.r_dot_step, ui.r_dot_max)
        if ui.check_button_click(50, 280, 100, 40):
            ui.r_dot_value = max(ui.r_dot_value - ui.r_dot_step, ui.r_dot_min)

        if ui.check_button_click(350, 380, 100, 40):
            ui.theta_dot_value = min(ui.theta_dot_value + ui.theta_dot_step, ui.theta_dot_max)
        if ui.check_button_click(50, 380, 100, 40):
            ui.theta_dot_value = max(ui.theta_dot_value - ui.theta_dot_step, ui.theta_dot_min)

        if ui.check_button_click(350, 480, 100, 40):
            ui.phi_dot_value = min(ui.phi_dot_value + ui.phi_dot_step, ui.phi_dot_max)
        if ui.check_button_click(50, 480, 100, 40):
            ui.phi_dot_value = max(ui.phi_dot_value - ui.phi_dot_step, ui.phi_dot_min)

        if ui.check_button_click(350, 580, 100, 40):
            ui.count_value = min(ui.count_value + ui.count_step, ui.count_max)
        if ui.check_button_click(50, 580, 100, 40):
            ui.count_value = max(ui.count_value - ui.count_step, ui.count_min)
        
        if ui.check_button_click(150, 650, 200, 40):
            print("Start button clicked!")
            run = True
        
        pygame.time.delay(50)
        
    
    if run:

        if state == 1:
            phase = "stance"
            r, r_dot, theta, theta_dot, phi, phi_dot, mass_x, mass_y, mass_z = StancePhase(
                m, k, r0, g, time_step, r, r_dot, theta, theta_dot, phi, phi_dot ,spring_x, spring_y, spring_z
            )

            if r >= r0 :
                phase = "take-off"

                target_theta = theta

                x_i, y_i, z_i, dx, dy, dz, spring_x, spring_y, spring_z, theta_i, T_flight, t_sim, state = StanceToFlight(
                    mass_x, mass_y, mass_z, theta, phi, r_dot, r, theta_dot, phi_dot)
                
                phase = "flight"

        elif (state == 2) & (phase == "flight"):
            mass_x, mass_y, mass_z, spring_x, spring_y, spring_z, theta = FlightPhase(
                x_i, y_i, z_i, r, theta, phi, theta_i, t_sim, g, dx, dy, dz, T_flight)
            
            t_sim += time_step

            if (spring_z <= 0) & (t_sim >= T_flight/2):
                phase = "touchdown"

                spring_z, x_i, y_i, z_i, dx, dy, dz, r_dot, phi_dot, theta_dot, T_stance, t_sim, state, theta_i = FlightToStance(
                    mass_x, mass_y, mass_z, r_dot, phi_dot, theta_dot, theta, m ,k
                )

                phase = "stance"

        elif (state == 3) & (phase == "stance"):
            r, r_dot, theta, theta_dot, phi, phi_dot, mass_x, mass_y, mass_z = StancePhaseControl(
                m, k, r0, g, time_step, r, r_dot, theta, target_theta, theta_dot, phi, phi_dot, spring_x, spring_y, spring_z, Kp, Kd
            )
            t_sim += time_step

            delta_x = r - r0

            force_in_spring = -k * delta_x

            if (abs(force_in_spring) < 1e-3) & (t_sim >= T_stance/2):
                phase = "take-off"

                count += 1

                x_i, y_i, z_i, dx, dy, dz, spring_x, spring_y, spring_z, theta_i, T_flight, t_sim, state = StanceToFlight(
                    mass_x, mass_y, mass_z, theta, phi, r_dot, r, theta_dot, phi_dot)
                
                phase = "flight"

        UpdateSimulation(spring_id, r, spring_x, spring_y, spring_z, theta, phi)
        cameraTargetPosition = [spring_x, spring_y, 0]
        p.resetDebugVisualizerCamera(cameraDistance, cameraYaw, cameraPitch, cameraTargetPosition)

        print(f"Phase: {phase}")
        print(f"Mass Position: x={mass_x}, y={mass_y}, z={mass_z}", f"Spring Base: x={spring_x}, y={spring_y}, z={spring_z}")
        print(f"theta: {theta}, theta_i = {theta_i},phi: {phi}")
        print(f"length spring: r= {r}")
        #print(f"time_phase: {t_sim}", f"T_fight: {T_fight}", f"T_stance: {T_stance}")

        print(f"Angulavelocity: r_dot={r_dot}, theta_dot={theta_dot}, phi_dot={phi_dot}")
        print(f"Count: {count}")
        if phase == "flight" or state == 3:
            print("target theta  ", target_theta)
        print("\n")
        # print(f"velocity: x_dot={dx}, y_dot={dy}, z_dot={dz}")
        
        p.stepSimulation()
        time.sleep(time_step)
    
    if count == count_max:
            is_running = False
            break