import numpy as np

def omni4_inverse_kinematics(vx, vy, omega, v_max, omega_max):
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

    # Normalizacja prędkości, jeśli przekracza maksymalne wartości
    max_omega_val = np.max(np.abs(wheel_omegas))
    if max_omega_val > omega_max:
        wheel_omegas = wheel_omegas * (omega_max / max_omega_val)

    # Konwersja na sygnały sterujące (PWM, kierunek)
    wheel_cmds = []
    for w in wheel_omegas:
        direction = 1 if w >= 0 else 0
        pwm = int(np.clip(abs(w) / omega_max * 255, 0, 255))
        wheel_cmds.append((pwm, direction))

    return wheel_cmds, wheel_omegas  # [(pwm1, dir1), (pwm2, dir2), (pwm3, dir3), (pwm4, dir4)]