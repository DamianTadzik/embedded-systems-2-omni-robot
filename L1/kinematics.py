import numpy as np

def omni4_inverse_kinematics(vx, vy, omega, max_wheel_omega_rad_per_s=1.0):
    """
    Compute wheel speeds and directions for 4-wheel omni (mecanum) robot.

    vx, vy  : linear velocities [m/s]
    omega   : angular velocity [rad/s]
    v_max   : maximum linear velocity [m/s]
    omega_max : maximum wheel angular velocity [rad/s]
    """

    # Stałe geometryczne robota
    wheel_diameter_m = 0.08       # średnica koła
    wheel_base_width_m = 0.24     # szerokość robota
    wheel_base_length_m = 0.14    # długość robota

    # Parametry pomocnicze
    r = wheel_diameter_m / 2      # promień koła
    L = wheel_base_length_m / 2   # pół długości robota
    W = wheel_base_width_m / 2    # pół szerokości robota

    # Macierz kinematyki odwrotnej (dla standardowych kół Mecanum)
    M = np.array([
        [ 1, -1, -(L + W)],
        [ 1,  1,  (L + W)],
        [ 1,  1, -(L + W)],
        [ 1, -1,  (L + W)]
    ])

    # Obliczenie prędkości kątowych kół [rad/s]
    wheel_omegas = (1 / r) * M @ np.array([vx, vy, omega])

    # limit handling (scaling all wheels proportionally)
    max_omega = np.max(np.abs(wheel_omegas))
    if max_omega > max_wheel_omega_rad_per_s:
        scale = max_wheel_omega_rad_per_s / max_omega
        wheel_omegas *= scale

    return wheel_omegas
