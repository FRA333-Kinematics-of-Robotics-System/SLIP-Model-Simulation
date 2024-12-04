# import Library
import pybullet as p
import pybullet_data
import numpy as np
import time
import os

p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.81)
p.loadURDF("plane.urdf")

# กำหนด path ของไฟล์ URDF
urdf_path = "urdf\SLIP_Model.urdf"

def InitialState():
    r0 = 1.0         # Natural spring length
    r = 0.3        # Compressed spring length (initial state)
    theta = np.pi/6  # Initial tilt angle (30 degrees)
    phi = 0.0        # Initial azimuthal angle
    r_dot = 0      # Radial velocity
    theta_dot = 0  # Polar angle velocity
    phi_dot = 0.0    # Azimuthal angle velocity

    return r, theta, phi, r_dot, theta_dot, phi_dot, r0

r, theta, phi, r_dot, theta_dot, phi_dot, r0 = InitialState()

# Start Position of mass
def InitialMassPosition():
    # คำนวณตำแหน่งของ Base (Mass) ที่ปลายบนของสปริง
    mass_x = r * np.sin(theta) * np.cos(phi)
    mass_y = r * np.sin(theta) * np.sin(phi)
    mass_z = r * np.cos(theta)
    return mass_x, mass_y, mass_z

mass_x, mass_y, mass_z = InitialMassPosition()

# Start position of Spring
def InitialSpringPosion():
    spring_x = mass_x - r * np.sin(theta) * np.cos(phi)
    spring_y = mass_y - r * np.sin(theta) * np.sin(phi)
    spring_z = mass_z - r * np.cos(theta) 
    return spring_x, spring_y, spring_z

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

# Function to update the spring length and position without flickering
def UpdateSimulation(spring_id, r, spring_x, spring_y, spring_z, theta, phi, theta_dot):
    # Assuming the prismatic joint is the first joint (adjust index accordingly)
    spring_index = 1  # Modify this if the prismatic joint is not the first joint

    # Calculate the new desired position based on the spring length (r)
    # You need to adjust the spring length here based on your simulation needs
    new_position = r

    # Set joint motor control to change the prismatic joint's position (spring length)
    p.setJointMotorControl2(
        spring_id,
        spring_index,
        controlMode=p.POSITION_CONTROL,
        targetPosition=new_position,
    )

    # Assuming you change the direction of theta_dot based on the phase
    if phase == 'fight':
        theta_dot = -theta_dot  # example value for fight
    elif phase == 'stance':
        theta_dot = theta_dot   # example value for stance

    # Smoothly update theta (angular position) based on theta_dot
    time_step = 0.01  # or the appropriate time step of your simulation
    theta = theta + theta_dot * time_step

    # Update the position and orientation of the spring and mass
    p.resetBasePositionAndOrientation(
        spring_id,
        [spring_x , spring_y , spring_z],  # Update the base position of the spring
        p.getQuaternionFromEuler([0, theta, phi])  # Update the spring's orientation
    )


# Debug sliders
K_spring_id = p.addUserDebugParameter("K Constant", 0, 1000, 100)
m_mass_id = p.addUserDebugParameter("Mass", 0, 10, 1)

l_rest = r0
k_spring = p.readUserDebugParameter(K_spring_id)  # N/m

def AngularToLinearVelocity(theta, phi, r_dot, r, theta_dot, phi_dot):
    # Calculate linear velocities
    dx = (
        np.sin(theta) * np.cos(phi) * r_dot +
        r * np.cos(theta) * np.cos(phi) * theta_dot -
        r * np.sin(theta) * np.sin(phi) * phi_dot
    )

    dy = (
        np.sin(theta) * np.sin(phi) * r_dot +
        r * np.cos(theta) * np.sin(phi) * theta_dot +
        r * np.sin(theta) * np.cos(phi) * phi_dot
    )

    dz = (
        np.cos(theta) * r_dot -
        r * np.sin(theta) * theta_dot
    )
    return dx, dy, dz

# Function for Fight Phase
def FightPhase(mass_x, mass_y, mass_z, r, theta, phi, theta_i, t, g, dx, dy, dz, T):

    # อัปเดตตำแหน่งของมวลในช่วง Fight phase
    mass_x = x_i + dx * t
    mass_y = y_i + dy * t
    mass_z = z_i + dz * t + 0.5 * g * t ** 2

    # R = np.sqrt(mass_x**2 + mass_y**2 + mass_z**2)

    # theta = np.arccos(mass_z / R)

    # คำนวณตำแหน่งของสปริงในช่วง Fight phase
    spring_x = mass_x - r * np.sin(theta) * np.cos(phi)
    spring_y = mass_y - r * np.sin(theta) * np.sin(phi)
    spring_z = mass_z - r * np.cos(theta)

    # คำนวณ theta โดยการลดลงจาก theta_i ไป -theta_i
    theta = theta_i - (2 * theta_i / T) * t

    return mass_x, mass_y, mass_z, spring_x, spring_y, spring_z, theta

# Function for Stance Phase
def StancePhase(m, k, r0, g, time_step, r, r_dot, theta, theta_dot, phi, phi_dot):
    # คำนวณ \ddot{r}(t) (Radial acceleration)
    r_ddot = (m * r * (theta_dot**2 + (np.sin(theta)**2) * (phi_dot**2)) +
              k * (r0 - r) - m * g * np.cos(theta)) / m

    # คำนวณ \ddot{\theta}(t) (Angular acceleration in theta)
    theta_ddot = (g * np.sin(theta) - 2 * r_dot * theta_dot +
                  0.5 * np.sin(2 * theta) * r * (phi_dot**2)) / r

    # คำนวณ \ddot{\phi}(t) (Angular acceleration in phi)
    if np.sin(theta) != 0:  # เพื่อหลีกเลี่ยงการหารด้วย 0
        phi_ddot = (-r * np.sin(2 * theta) * phi_dot * theta_dot -
                    2 * r_dot * np.sin(theta) * phi_dot) / (r * np.sin(theta))
    else:
        phi_ddot = 0  # หาก \sin(\theta) = 0

    #print(f"Angular Accerelation: r_ddot={r_ddot}, theta_ddot={theta_ddot}, phi_ddot={phi_ddot}")

    # อัปเดตค่าความเร็วเชิงมุมและมุม
    r_dot += r_ddot * time_step
    r += r_dot * time_step
    # if r <= 0:
    #     r = r0  # Reset to the natural length of the spring

    r = max(0, min(r, 2*r0))

    theta_dot += theta_ddot * time_step
    theta += theta_dot * time_step
    theta = theta % (2 * np.pi)  # Normalize theta

    phi_dot += phi_ddot * time_step
    phi += phi_dot * time_step
    phi = phi % (2 * np.pi)  # Normalize phi

    mass_x = r * np.sin(theta) * np.cos(phi)
    mass_y = r * np.sin(theta) * np.sin(phi)
    mass_z = r * np.cos(theta)

    return r, r_dot, theta, theta_dot, phi, phi_dot, mass_x, mass_y, mass_z

# Function for Touchdown (Fight to Stance)
def FightToStance(mass_x, mass_y, mass_z, r_dot, phi_dot, theta_dot, theta_i, m_mass_id ,k_spring):
    x_i = mass_x
    y_i = mass_y
    z_i = mass_z

    spring_z = 0

    T_stance = 2*np.pi* np.sqrt(m_mass_id/k_spring)
    t_sim = 0
    theta = -theta_i
    r_dot, phi_dot, theta_dot = -r_dot, -phi_dot, -theta_dot
    dx, dy, dz = 0, 0, 0

    state = 3

    return spring_z, x_i, y_i, z_i, dx, dy, dz, r_dot, phi_dot, theta_dot, theta, T_stance, t_sim, state


# Function for Take-off (Stance to Fight)
def StanceToFlight(mass_x, mass_y, mass_z, theta, phi, r_dot, r, theta_dot, phi_dot):
    x_i = mass_x
    y_i = mass_y
    z_i = mass_z

    dx, dy, dz = AngularToLinearVelocity(theta, phi, r_dot, r, theta_dot, phi_dot)
    theta_i = theta
    T_fight = 2 * dz / -g
    t_sim = 0
    state = 2

    spring_x = x_i - r * np.sin(theta) * np.cos(phi)
    spring_y = y_i - r * np.sin(theta) * np.sin(phi)
    spring_z = z_i - r * np.cos(theta)

    return x_i, y_i, z_i, dx, dy, dz, spring_x, spring_y, spring_z, theta_i, T_fight, t_sim, state


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
