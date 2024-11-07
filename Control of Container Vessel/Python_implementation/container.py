import numpy as np

def container(x, ui):
    """
    Returns the speed U (m/s) and the time derivative of the state vector x for a container ship.
    
    Parameters:
    x : np.array
        State vector [u, v, r, x, y, psi, p, phi, delta, n].
    ui : np.array
        Input vector [delta_c, n_c].
        
    Returns:
    xdot : np.array
        Time derivative of the state vector.
    U : float
        Speed in m/s.
    """
    
    # Check input dimensions
    if len(x) != 10:
        raise ValueError("x-vector must have dimension 10!")
    if len(ui) != 2:
        raise ValueError("ui-vector must have dimension 2!")
    
    # Normalization variables
    L = 175.0  # length of ship (m)
    U = np.sqrt(x[0]**2 + x[1]**2)  # service speed (m/s)

    # Check service speed
    if U <= 0:
        raise ValueError("The ship must have speed greater than zero")
    if x[9] <= 0:
        raise ValueError("The propeller rpm must be greater than zero")
    
    # Constants
    delta_max = 30.0 * np.pi / 180  # max rudder angle (rad)
    Ddelta_max = 5.0 * np.pi / 180  # max rudder rate (rad/s)
    n_max = 160.0  # max shaft velocity (rpm)

    # Input variables
    delta_c = ui[0]
    n_c = ui[1] / 60.0 * L / U

    # States
    u = x[0] / U
    v = x[1] / U
    p = x[6] * L / U
    r = x[2] * L / U
    phi = x[7]
    psi = x[5]
    delta = x[8]
    n = x[9] / 60.0 * L / U

    # Parameters
    m = 0.00792
    mx = 0.000238
    my = 0.007049
    Ix = 0.0000176
    alphay = 0.05
    ly = 0.0313
    Iz = 0.000456
    Jx = 0.0000034
    Jz = 0.000419
    g = 9.81
    rho = 1025
    d = 8.5
    D = 6.533
    W = rho * g * 21222 / (rho * L**2 * U**2 / 2)

    # Hydrodynamic derivatives
    Xuu = -0.0004226
    Xvr = -0.00311
    Xrr = 0.00020
    Xphiphi = -0.00020
    Xvv = -0.00386

    Kv = 0.0003026
    Kr = -0.000063
    Kp = -0.0000075
    Kphi = -0.000021

    Yv = -0.0116
    Yr = 0.00242
    Np = 0.000213
    Nv = -0.0038545
    Nr = -0.00222

    # Rudder saturation and dynamics
    delta_c = np.clip(delta_c, -delta_max, delta_max)
    delta_dot = delta_c - delta
    delta_dot = np.clip(delta_dot, -Ddelta_max, Ddelta_max)

    # Shaft velocity saturation and dynamics
    n_c = np.clip(n_c * U / L, -n_max / 60.0, n_max / 60.0)
    if n > 0.3:
        Tm = 5.65 / n
    else:
        Tm = 18.83
    n_dot = (1 / Tm) * (n_c - n) * 60

    # Dimensional state derivatives
    detM = my * Ix * Iz - my**2 * Iz - my**2 * Ix
    X = Xuu * u**2 + (1 - 0.175) * T + Xvr * v * r + Xvv * v**2 + Xrr * r**2 + Xphiphi * phi**2
    Y = Yv * v + Yr * r + Kv * p + Kphi * phi
    K = Kv * v + Kr * r + Kp * p + Kphi * phi
    N = Nv * v + Nr * r + Np * p

    # Final state derivative vector
    xdot = np.array([
        X * (U**2 / L) / m,
        -((-Ix * Iz * Y + my * Iz * K + my * Ix * N) / detM) * (U**2 / L),
        ((-my * Ix * Y + my * my * K + N * my * Ix - N * my**2) / detM) * (U**2 / L**2),
        (np.cos(psi) * u - np.sin(psi) * np.cos(phi) * v) * U,
        (np.sin(psi) * u + np.cos(psi) * np.cos(phi) * v) * U,
        np.cos(phi) * r * (U / L),
        ((-my * Iz * Y + K * my * Iz - K * my**2 + my * my * N) / detM) * (U**2 / L**2),
        p * (U / L),
        delta_dot,
        n_dot
    ])
    
    return xdot, U
