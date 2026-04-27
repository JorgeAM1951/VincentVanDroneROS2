import numpy as np
import scipy.sparse as sparse
import osqp
import math

def get_poly_deriv_coeffs(t, derivative_order):
    """Devuelve los coeficientes de la derivada 'd' evaluada en 't'."""
    coeffs = np.zeros(8)
    for i in range(derivative_order, 8):
        val = 1
        for k in range(derivative_order):
            val *= (i - k)
        coeffs[i] = val * (t ** (i - derivative_order))
    return coeffs

def solve_minimum_snap_3d_projective(waypoints, period, camera_center):
    """
    Solucionador 3D acoplado con restricciones geométricas proyectivas.
    """
    n_segments = len(waypoints) - 1
    n_vars_1d = 8 * n_segments
    n_vars_total = 3 * n_vars_1d # 24N variables (X, Y, Z concatenados)
    
    # ---------------------------------------------------------
    # 1. MATRIZ DE COSTE (P) 
    # ---------------------------------------------------------
    P_1d = np.zeros((n_vars_1d, n_vars_1d))
    for seg in range(n_segments):
        T = period
        for i in range(4, 8):
            for j in range(4, 8):
                term_i = math.factorial(i) / math.factorial(i-4)
                term_j = math.factorial(j) / math.factorial(j-4)
                power = i + j - 7
                val = (term_i * term_j * (T ** power)) / power
                P_1d[8*seg+i, 8*seg+j] = val

    P = np.block([
        [P_1d, np.zeros_like(P_1d), np.zeros_like(P_1d)],
        [np.zeros_like(P_1d), P_1d, np.zeros_like(P_1d)],
        [np.zeros_like(P_1d), np.zeros_like(P_1d), P_1d]
    ])
    P += np.eye(n_vars_total) * 1e-6 

    # ---------------------------------------------------------
    # 2. RESTRICCIONES GEOMÉTRICAS (A, l, u)
    # ---------------------------------------------------------
    A_rows = []
    l_vals = []
    u_vals = []

    def add_constraint(row_x, row_y, row_z, l_val, u_val):
        """Añade una fila a la matriz A con límites inferior (l) y superior (u)"""
        row = np.zeros(n_vars_total)
        row[0 : n_vars_1d] = row_x
        row[n_vars_1d : 2*n_vars_1d] = row_y
        row[2*n_vars_1d : ] = row_z
        A_rows.append(row)
        l_vals.append(l_val)
        u_vals.append(u_val)

    # Coeficientes en los extremos del segmento
    c_start = get_poly_deriv_coeffs(0, 0)
    c_end = get_poly_deriv_coeffs(period, 0)

    # --- Restricción A: Fijar el inicio (WP0) para anclar el problema ---
    row_1d = np.zeros(n_vars_1d); row_1d[0:8] = c_start
    add_constraint(row_1d, np.zeros(n_vars_1d), np.zeros(n_vars_1d), waypoints[0,0], waypoints[0,0]) # X
    add_constraint(np.zeros(n_vars_1d), row_1d, np.zeros(n_vars_1d), waypoints[0,1], waypoints[0,1]) # Y
    add_constraint(np.zeros(n_vars_1d), np.zeros(n_vars_1d), row_1d, waypoints[0,2], waypoints[0,2]) # Z

    # --- Restricción B: Cruzar la línea epipolar al final de cada segmento ---
    for seg in range(n_segments):
        ray_vec = waypoints[seg+1] - camera_center
        
        # Vectores normales al rayo
        temp = np.array([1, 0, 0]) if abs(ray_vec[0]) < abs(ray_vec[1]) and abs(ray_vec[0]) < abs(ray_vec[2]) else (np.array([0, 1, 0]) if abs(ray_vec[1]) < abs(ray_vec[2]) else np.array([0, 0, 1]))
        n1 = np.cross(ray_vec, temp)
        n1 = n1 / np.linalg.norm(n1)
        n2 = np.cross(ray_vec, n1)
        n2 = n2 / np.linalg.norm(n2)

        # Imponer desvío cero en las normales (igualdad estricta)
        rx = np.zeros(n_vars_1d); rx[8*seg : 8*seg+8] = c_end * n1[0]
        ry = np.zeros(n_vars_1d); ry[8*seg : 8*seg+8] = c_end * n1[1]
        rz = np.zeros(n_vars_1d); rz[8*seg : 8*seg+8] = c_end * n1[2]
        add_constraint(rx, ry, rz, np.dot(n1, camera_center), np.dot(n1, camera_center))

        rx2 = np.zeros(n_vars_1d); rx2[8*seg : 8*seg+8] = c_end * n2[0]
        ry2 = np.zeros(n_vars_1d); ry2[8*seg : 8*seg+8] = c_end * n2[1]
        rz2 = np.zeros(n_vars_1d); rz2[8*seg : 8*seg+8] = c_end * n2[2]
        add_constraint(rx2, ry2, rz2, np.dot(n2, camera_center), np.dot(n2, camera_center))

    # --- Restricción C: Segmentos confinados en su plano epipolar ---
    for seg in range(n_segments):
        v1 = waypoints[seg] - camera_center
        v2 = waypoints[seg+1] - camera_center
        n_plane = np.cross(v1, v2)
        
        norm_plane = np.linalg.norm(n_plane)
        if norm_plane > 1e-6:
            n_plane = n_plane / norm_plane
            for k in range(8):
                rx = np.zeros(n_vars_1d); rx[8*seg + k] = n_plane[0]
                ry = np.zeros(n_vars_1d); ry[8*seg + k] = n_plane[1]
                rz = np.zeros(n_vars_1d); rz[8*seg + k] = n_plane[2]
                target_val = np.dot(n_plane, camera_center) if k == 0 else 0.0
                add_constraint(rx, ry, rz, target_val, target_val)

    # --- Restricción D: Continuidad (Igualdad estricta) ---
    for seg in range(n_segments - 1):
        for derivative in [0, 1, 2, 3]:
            c_end_deriv = get_poly_deriv_coeffs(period, derivative)
            c_start_deriv = -get_poly_deriv_coeffs(0, derivative)
            
            for axis in ['X', 'Y', 'Z']:
                rx, ry, rz = np.zeros(n_vars_1d), np.zeros(n_vars_1d), np.zeros(n_vars_1d)
                target_array = rx if axis == 'X' else (ry if axis == 'Y' else rz)
                target_array[8*seg : 8*seg+8] = c_end_deriv
                target_array[8*(seg+1) : 8*(seg+1)+8] = c_start_deriv
                add_constraint(rx, ry, rz, 0.0, 0.0)

    # --- Restricción E: CERCANÍA (Evitar el centro de la cámara) ---
    # Imponemos que la distancia proyectada a lo largo del rayo esté acotada
    for seg in range(n_segments):
        ray_vec = waypoints[seg+1] - camera_center
        dist_orig = np.linalg.norm(ray_vec)
        
        if dist_orig > 1e-6:
            u_ray = ray_vec / dist_orig # Vector unitario de la línea de visión
            
            rx = np.zeros(n_vars_1d); rx[8*seg : 8*seg+8] = c_end * u_ray[0]
            ry = np.zeros(n_vars_1d); ry[8*seg : 8*seg+8] = c_end * u_ray[1]
            rz = np.zeros(n_vars_1d); rz[8*seg : 8*seg+8] = c_end * u_ray[2]
            
            # --- Aquí defines el margen de libertad en el rayo ---
            # 0.5 = puede acercarse a la cámara hasta la mitad de la distancia original
            # 1.5 = puede alejarse de la cámara hasta un 50% más
            d_min = 0.5 * dist_orig 
            d_max = 1.5 * dist_orig 
            
            l_val = d_min + np.dot(u_ray, camera_center)
            u_val = d_max + np.dot(u_ray, camera_center)
            
            add_constraint(rx, ry, rz, l_val, u_val)

    # ---------------------------------------------------------
    # 3. SOLUCIÓN CON OSQP
    # ---------------------------------------------------------
    A_sparse = sparse.csc_matrix(np.array(A_rows))
    P_sparse = sparse.csc_matrix(P)
    q = np.zeros(n_vars_total)
    
    # Vectores de límites inferior y superior
    l_vec = np.array(l_vals)
    u_vec = np.array(u_vals)

    prob = osqp.OSQP()
    prob.setup(P_sparse, q, A_sparse, l=l_vec, u=u_vec, verbose=False)
    res = prob.solve()

    if res.info.status_val != 1:
        print("¡Advertencia! El solver proyectivo no convergió.")

    x_res = res.x
    cx = x_res[0 : n_vars_1d]
    cy = x_res[n_vars_1d : 2*n_vars_1d]
    cz = x_res[2*n_vars_1d : ]

    return cx, cy, cz