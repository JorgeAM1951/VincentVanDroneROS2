import numpy as np
import scipy.sparse as sparse
import scipy.io as sio
import clarabel
import math

def get_poly_deriv_coeffs(tau, derivative_order):
    # Forzar precisión extendida de 128 bits
    coeffs = np.zeros(8)
    for i in range(derivative_order, 8):
        val = 1
        for k in range(derivative_order):
            val *= (i - k)
        coeffs[i] = val * (tau ** (i - derivative_order))
    return coeffs

def remove_dependent_equalities(A_eq, b_eq, tol=1e-6):
    if len(A_eq) == 0:
        return np.array([]), np.array([])
    
    A_indep = []
    b_indep = []
    Q = None 
    
    for i in range(A_eq.shape[0]):
        row = A_eq[i, :]
        norm_row = np.linalg.norm(row)
        
        if norm_row < tol:
            continue 
            
        if Q is None:
            Q = (row / norm_row).reshape(1, -1)
            A_indep.append(row)
            b_indep.append(b_eq[i])
        else:
            proj = row @ Q.T
            residual = row - proj @ Q
            norm_res = np.linalg.norm(residual)
            
            if norm_res > tol:
                q_new = (residual / norm_res).reshape(1, -1)
                Q = np.vstack([Q, q_new])
                A_indep.append(row)
                b_indep.append(b_eq[i])
                
    print(f"[PRESOLVE] Igualdades originales: {A_eq.shape[0]} | Tras eliminar redundantes: {len(A_indep)}")
    return np.array(A_indep), np.array(b_indep)

def solve_minimum_snap_3d_projective(waypoints, period, camera_center=None, bbox_low=None, bbox_high=None):
    if camera_center is None:
        raise ValueError("Se necesita 'camera_center' para los planos epipolares.")

    # Asegurar precisión matemática de entrada
    waypoints = np.array(waypoints)
    camera_center = np.array(camera_center)

    n_segments = len(waypoints) - 1
    n_vars_1d = 8 * n_segments
    n_vars_total = 3 * n_vars_1d
    
    # Normalizamos el tiempo del optimizador a [0, 1]
    T = period
    tau_end = 1.0
    
    # 0. Normales calculadas en float128
    normals = []
    for i in range(n_segments):
        ray1 = waypoints[i] - camera_center
        ray2 = waypoints[i+1] - camera_center
        normal = np.cross(ray1, ray2)
        norm_length = np.linalg.norm(normal)
        if norm_length > 1e-6:
            normal = normal / norm_length
        normals.append(normal)

    # 1. Matriz de Coste P en float128
    P_1d = np.zeros((n_vars_1d, n_vars_1d))
    for seg in range(n_segments):
        for i in range(4, 8):
            for j in range(4, 8):
                term_i = math.factorial(i) / math.factorial(i - 4)
                term_j = math.factorial(j) / math.factorial(j - 4)
                power = i + j - 7
                val = (term_i * term_j * (tau_end ** power)) / power
                P_1d[8 * seg + i, 8 * seg + j] = val / (T ** 7)

    P_sparse = sparse.block_diag([P_1d, P_1d, P_1d], format='csc')
    
    # NOTA: Si persiste el "InsufficientProgress", puedes subir este 1e-12 a 1e-9 o 1e-6
    P_sparse += sparse.eye(n_vars_total) * 1e-12

    Aeq_rows, beq_vals = [], []
    Aineq_rows, bineq_vals = [], []

    def add_eq(coeffs_x, coeffs_y, coeffs_z, val):
        Aeq_rows.append(np.concatenate([coeffs_x, coeffs_y, coeffs_z]))
        beq_vals.append(val)
        
    def add_ineq(coeffs_x, coeffs_y, coeffs_z, low, up):
        # Ax <= up
        Aineq_rows.append(np.concatenate([coeffs_x, coeffs_y, coeffs_z]))
        bineq_vals.append(up)
        # -Ax <= -low
        Aineq_rows.append(-np.concatenate([coeffs_x, coeffs_y, coeffs_z]))
        bineq_vals.append(-low)

    # =========================================================
    # 2. IGUALDADES (float128)
    # =========================================================
    
    # [1] pos continuity
    for seg in range(n_segments - 1):
        c_left = get_poly_deriv_coeffs(tau_end, 0); c_right = get_poly_deriv_coeffs(0.0, 0)
        for ax in [0, 1, 2]:
            t_vec = np.zeros(n_vars_total)
            t_vec[ax*n_vars_1d + 8*seg : ax*n_vars_1d + 8*seg+8] = c_left
            t_vec[ax*n_vars_1d + 8*(seg+1) : ax*n_vars_1d + 8*(seg+1)+8] = -c_right
            add_eq(t_vec[0:n_vars_1d], t_vec[n_vars_1d:2*n_vars_1d], t_vec[2*n_vars_1d:3*n_vars_1d], 0.0)

    # [2] vel continuity
    for seg in range(n_segments - 1):
        c_left = get_poly_deriv_coeffs(tau_end, 1); c_right = get_poly_deriv_coeffs(0.0, 1)
        for ax in [0, 1, 2]:
            t_vec = np.zeros(n_vars_total)
            t_vec[ax*n_vars_1d + 8*seg : ax*n_vars_1d + 8*seg+8] = c_left
            t_vec[ax*n_vars_1d + 8*(seg+1) : ax*n_vars_1d + 8*(seg+1)+8] = -c_right
            add_eq(t_vec[0:n_vars_1d], t_vec[n_vars_1d:2*n_vars_1d], t_vec[2*n_vars_1d:3*n_vars_1d], 0.0)

    # [3] acc continuity
    for seg in range(n_segments - 1):
        c_left = get_poly_deriv_coeffs(tau_end, 2); c_right = get_poly_deriv_coeffs(0.0, 2)
        for ax in [0, 1, 2]:
            t_vec = np.zeros(n_vars_total)
            t_vec[ax*n_vars_1d + 8*seg : ax*n_vars_1d + 8*seg+8] = c_left
            t_vec[ax*n_vars_1d + 8*(seg+1) : ax*n_vars_1d + 8*(seg+1)+8] = -c_right
            add_eq(t_vec[0:n_vars_1d], t_vec[n_vars_1d:2*n_vars_1d], t_vec[2*n_vars_1d:3*n_vars_1d], 0.0)

    # [5] pos initial
    c_0 = get_poly_deriv_coeffs(0.0, 0)
    for ax in [0, 1, 2]:
        t_vec = np.zeros(n_vars_total)
        t_vec[ax*n_vars_1d : ax*n_vars_1d + 8] = c_0
        add_eq(t_vec[0:n_vars_1d], t_vec[n_vars_1d:2*n_vars_1d], t_vec[2*n_vars_1d:3*n_vars_1d], waypoints[0][ax])

    # [6] pos final
    c_T = get_poly_deriv_coeffs(tau_end, 0)
    idx = 8 * (n_segments - 1)
    for ax in [0, 1, 2]:
        t_vec = np.zeros(n_vars_total)
        t_vec[ax*n_vars_1d + idx : ax*n_vars_1d + idx + 8] = c_T
        add_eq(t_vec[0:n_vars_1d], t_vec[n_vars_1d:2*n_vars_1d], t_vec[2*n_vars_1d:3*n_vars_1d], waypoints[-1][ax])

    # [7] vel initial
    cv_0 = get_poly_deriv_coeffs(0.0, 1)
    for ax in [0, 1, 2]:
        t_vec = np.zeros(n_vars_total)
        t_vec[ax*n_vars_1d : ax*n_vars_1d + 8] = cv_0
        add_eq(t_vec[0:n_vars_1d], t_vec[n_vars_1d:2*n_vars_1d], t_vec[2*n_vars_1d:3*n_vars_1d], 0.0)

    # [8] vel final
    cv_T = get_poly_deriv_coeffs(tau_end, 1)
    for ax in [0, 1, 2]:
        t_vec = np.zeros(n_vars_total)
        t_vec[ax*n_vars_1d + idx : ax*n_vars_1d + idx + 8] = cv_T
        add_eq(t_vec[0:n_vars_1d], t_vec[n_vars_1d:2*n_vars_1d], t_vec[2*n_vars_1d:3*n_vars_1d], 0.0)

    # [4] normals
    for seg in range(n_segments):
        nx, ny, nz = normals[seg]
        d_plane = np.dot(normals[seg], camera_center)
        for p in range(8):
            rx = np.zeros(n_vars_1d)
            ry = np.zeros(n_vars_1d)
            rz = np.zeros(n_vars_1d)
            rx[8*seg + p] = nx; ry[8*seg + p] = ny; rz[8*seg + p] = nz
            val = d_plane if p == 0 else 0.0
            add_eq(rx, ry, rz, val)

    # [15] jerk cont
    for seg in range(n_segments - 1):
        c_left = get_poly_deriv_coeffs(tau_end, 3); c_right = get_poly_deriv_coeffs(0.0, 3)
        for ax in [0, 1, 2]:
            t_vec = np.zeros(n_vars_total)
            t_vec[ax*n_vars_1d + 8*seg : ax*n_vars_1d + 8*seg+8] = c_left
            t_vec[ax*n_vars_1d + 8*(seg+1) : ax*n_vars_1d + 8*(seg+1)+8] = -c_right
            add_eq(t_vec[0:n_vars_1d], t_vec[n_vars_1d:2*n_vars_1d], t_vec[2*n_vars_1d:3*n_vars_1d], 0.0)

    # Conversión a arrays de numpy explícitos en float128
    Aeq_raw = np.array(Aeq_rows)
    beq_raw = np.array(beq_vals)
    Aineq_raw = np.array(Aineq_rows) if len(Aineq_rows) > 0 else np.empty((0, n_vars_total))
    bineq_raw = np.array(bineq_vals) if len(bineq_vals) > 0 else np.array([])
    
    # Cast preventivo a float64 solo para guardar el .mat (evita que crashée scipy.io)
    sio.savemat('matrices_python_bruto.mat', {
        'P_py': P_sparse.astype(np.float64), 
        'q_py': np.zeros(n_vars_total, dtype=np.float64), 
        'Aeq_py': Aeq_raw.astype(np.float64),
        'beq_py': beq_raw.astype(np.float64),
        'Aineq_py': Aineq_raw.astype(np.float64), 
        'bineq_py': bineq_raw.astype(np.float64)
    })

    # => FILTRO PRESOLVE (Procesa internamente en float128)
    Aeq_mat, beq_vec = remove_dependent_equalities(Aeq_raw, beq_raw)
    
    # =========================================================
    # 3. DESIGUALDADES (LÍMITES FÍSICOS ESCALADOS)
    # =========================================================
    min_x, max_x = -2.1658, 2.1618
    min_y, max_y = 2.5000, 3.4172
    min_z, max_z = 0.4947, 2.8152

    min_vx, max_vx = -0.4289 * T, 0.3987 * T
    min_vy, max_vy = -0.2036 * T, 0.1780 * T
    min_vz, max_vz = -0.4112 * T, 0.4226 * T

    min_ax, max_ax = -0.1224 * (T**2), 0.2733 * (T**2)
    min_ay, max_ay = -0.0843 * (T**2), 0.0888 * (T**2)
    min_az, max_az = -0.1291 * (T**2), 0.2716 * (T**2)

    t_samples = np.linspace(0, tau_end, 10, )
    for seg in range(n_segments):
        for t_eval in t_samples:
            # POS
            c_pos = get_poly_deriv_coeffs(t_eval, 0)
            rx = np.zeros(n_vars_1d, )
            rx[8*seg:8*seg+8] = c_pos; add_ineq(rx, np.zeros(n_vars_1d), np.zeros(n_vars_1d, ), min_x, max_x)
            ry = np.zeros(n_vars_1d)
            ry[8*seg:8*seg+8] = c_pos; add_ineq(np.zeros(n_vars_1d), ry, np.zeros(n_vars_1d), min_y, max_y)
            rz = np.zeros(n_vars_1d)
            rz[8*seg:8*seg+8] = c_pos; add_ineq(np.zeros(n_vars_1d), np.zeros(n_vars_1d), rz, min_z, max_z)

            # VEL
            c_vel = get_poly_deriv_coeffs(t_eval, 1)
            rx = np.zeros(n_vars_1d)
            rx[8*seg:8*seg+8] = c_vel; add_ineq(rx, np.zeros(n_vars_1d), np.zeros(n_vars_1d), min_vx, max_vx)
            ry = np.zeros(n_vars_1d)
            ry[8*seg:8*seg+8] = c_vel; add_ineq(np.zeros(n_vars_1d), ry, np.zeros(n_vars_1d), min_vy, max_vy)
            rz = np.zeros(n_vars_1d)
            rz[8*seg:8*seg+8] = c_vel; add_ineq(np.zeros(n_vars_1d), np.zeros(n_vars_1d), rz, min_vz, max_vz)

            # ACC
            c_acc = get_poly_deriv_coeffs(t_eval, 2)
            rx = np.zeros(n_vars_1d)
            rx[8*seg:8*seg+8] = c_acc; add_ineq(rx, np.zeros(n_vars_1d), np.zeros(n_vars_1d), min_ax, max_ax)
            ry = np.zeros(n_vars_1d)
            ry[8*seg:8*seg+8] = c_acc; add_ineq(np.zeros(n_vars_1d), ry, np.zeros(n_vars_1d), min_ay, max_ay)
            rz = np.zeros(n_vars_1d)
            rz[8*seg:8*seg+8] = c_acc; add_ineq(np.zeros(n_vars_1d), np.zeros(n_vars_1d), rz, min_az, max_az)

    # =========================================================
    # 4. RESOLVER CON CLARABEL
    # =========================================================
    A_final = np.vstack([Aeq_mat, np.array(Aineq_rows)]) if len(Aineq_rows) > 0 else Aeq_mat
    b_final = np.concatenate([beq_vec, np.array(bineq_vals)]) if len(bineq_vals) > 0 else beq_vec

    A_sparse = sparse.csc_matrix(A_final)
    q = np.zeros(n_vars_total)

    cones = []
    if len(beq_vec) > 0:
        cones.append(clarabel.ZeroConeT(len(beq_vec)))
    if len(bineq_vals) > 0:
        cones.append(clarabel.NonnegativeConeT(len(bineq_vals)))

    settings = clarabel.DefaultSettings()
    settings.verbose = True
    settings.tol_gap_abs = 1e-8
    settings.tol_gap_rel = 1e-8
    settings.tol_feas = 1e-8

    # Iniciar Solver cediendo los datos finales a float64 para Clarabel
    solver = clarabel.DefaultSolver(
        P_sparse.astype(np.float64), 
        q.astype(np.float64), 
        A_sparse.astype(np.float64), 
        b_final.astype(np.float64), 
        cones, 
        settings
    )
    res = solver.solve()

    if str(res.status) != "Solved":
        print(f"Clarabel advierte un estatus diferente a 'Solved': {res.status}")
        if str(res.status) not in ["Solved", "AlmostSolved"]:
            return None, None, None

    # Extraer solución y devolver en float128
    sol = res.x
    cx = np.array(sol[0 : n_vars_1d])
    cy = np.array(sol[n_vars_1d : 2*n_vars_1d])
    cz = np.array(sol[2*n_vars_1d : 3*n_vars_1d])
    
    # Deshacer la normalización temporal para devolver el tiempo a real
    for seg in range(n_segments):
        for i in range(8):
            scale = T ** i
            cx[8*seg + i] /= scale
            cy[8*seg + i] /= scale
            cz[8*seg + i] /= scale
            
    return cx, cy, cz