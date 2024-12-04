import numpy as np

def InitialState(_r0=1.0, _r=0.3, _theta=np.pi/6, _phi=np.pi/6, _r_dot=0.0, _theta_dot=0.0, _phi_dot=0.0, _g=-9.81):
    r0 = _r0
    r = _r
    theta = _theta
    phi = _phi
    r_dot = _r_dot
    theta_dot = _theta_dot
    phi_dot = _phi_dot
    g = _g

    return r, theta, phi, r_dot, theta_dot, phi_dot, r0, g

r, theta, phi, r_dot, theta_dot, phi_dot, r0, g = InitialState()

phase = "Initial"
x_i, y_i, z_i = 0, 0, 0

def InitialMassPosition():
    mass_x = r * np.sin(theta) * np.cos(phi)
    mass_y = r * np.sin(theta) * np.sin(phi)
    mass_z = r * np.cos(theta)
    return mass_x, mass_y, mass_z

mass_x, mass_y, mass_z = InitialMassPosition()

def InitialSpringPosion():
    spring_x = mass_x - r * np.sin(theta) * np.cos(phi)
    spring_y = mass_y - r * np.sin(theta) * np.sin(phi)
    spring_z = mass_z - r * np.cos(theta) 
    return spring_x, spring_y, spring_z

spring_x, spring_y, spring_z = InitialSpringPosion()

def AngularToLinearVelocity(theta, phi, r_dot, r, theta_dot, phi_dot):
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

def FightPhase(x_i, y_i, z_i, r, theta, phi, theta_i, t, g, dx, dy, dz, T):

    mass_x = x_i + dx * t
    mass_y = y_i + dy * t
    mass_z = z_i + dz * t + 0.5 * g * t ** 2

    spring_x = mass_x - r * np.sin(theta) * np.cos(phi)
    spring_y = mass_y - r * np.sin(theta) * np.sin(phi)
    spring_z = mass_z - r * np.cos(theta)

    theta = theta_i - (2 * theta_i / T) * t

    return mass_x, mass_y, mass_z, spring_x, spring_y, spring_z, theta

def StancePhase(m, k, r0, g, time_step, r, r_dot, theta, theta_dot, phi, phi_dot ,spring_x, spring_y, spring_z):

    r_ddot = (m * r * (theta_dot**2 + (np.sin(theta)**2) * (phi_dot**2)) +
              k * (r0 - r) - m * g * np.cos(theta)) / m

    theta_ddot = (g * np.sin(theta) - 2 * r_dot * theta_dot +
                  0.5 * np.sin(2 * theta) * r * (phi_dot**2)) / r

    if np.sin(theta) != 0:
        phi_ddot = (-r * np.sin(2 * theta) * phi_dot * theta_dot -
                    2 * r_dot * np.sin(theta) * phi_dot) / (r * np.sin(theta))
    else:
        phi_ddot = 0

    r_dot += r_ddot * time_step
    r += r_dot * time_step

    r = max(0, min(r, r0))

    theta_dot += theta_ddot * time_step
    theta += theta_dot * time_step
    theta = (theta + np.pi) % (2 * np.pi) - np.pi

    phi_dot += phi_ddot * time_step
    phi += phi_dot * time_step
    phi = phi % (2 * np.pi)

    mass_x = spring_x + r * np.sin(theta) * np.cos(phi)
    mass_y = spring_y + r * np.sin(theta) * np.sin(phi)
    mass_z = spring_z + r * np.cos(theta)

    return r, r_dot, theta, theta_dot, phi, phi_dot, mass_x, mass_y, mass_z

def FightToStance(mass_x, mass_y, mass_z, r_dot, phi_dot, theta_dot, m_mass_id ,k_spring):
    x_i = mass_x
    y_i = mass_y
    z_i = mass_z

    spring_z = 0

    T_stance = 2*np.pi* np.sqrt(m_mass_id/k_spring)
    t_sim = 0
    
    r_dot, phi_dot, theta_dot = -r_dot, -phi_dot, -theta_dot
    dx, dy, dz = 0, 0, 0

    state = 3

    return spring_z, x_i, y_i, z_i, dx, dy, dz, r_dot, phi_dot, theta_dot, T_stance, t_sim, state

def StanceToFlight(mass_x, mass_y, mass_z, theta, phi, r_dot, r, theta_dot, phi_dot):
    x_i = mass_x
    y_i = mass_y
    z_i = mass_z

    dx, dy, dz = AngularToLinearVelocity(theta, phi, r_dot, r, theta_dot, phi_dot)
    theta_i = theta
    theta_i = (theta + np.pi) % (2 * np.pi) - np.pi
    T_fight = 2 * dz / -g
    t_sim = 0
    state = 2

    spring_x = x_i - r * np.sin(theta) * np.cos(phi)
    spring_y = y_i - r * np.sin(theta) * np.sin(phi)
    spring_z = z_i - r * np.cos(theta)

    return x_i, y_i, z_i, dx, dy, dz, spring_x, spring_y, spring_z, theta_i, T_fight, t_sim, state
