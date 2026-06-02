# import numpy as np
# import scipy.sparse as sparse
# import osqp
# import math

# def get_poly_deriv_coeffs(t, derivative_order):
#     coeffs = np.zeros(8)
#     for i in range(derivative_order, 8):
#         val = 1
#         for k in range(derivative_order):
#             val *= (i - k)
#         coeffs[i] = val * (t ** (i - derivative_order))
#     return coeffs

# def remove_dependent_equalities(A_eq, b_eq, tol=1e-7):
#     """
#     Emula el 'Presolve' de MATLAB.
#     Escanea las restricciones de arriba a abajo y elimina las que son 
#     linealmente dependientes (redundantes) respecto a las anteriores.
#     """
#     A_indep = []
#     b_indep = []
#     Q = None # Matriz de base ortogonal
    
#     for i in range(A_eq.shape[0]):
#         row = A_eq[i, :]
#         norm_row = np.linalg.norm(row)
        
#         if norm_row < tol:
#             continue # Ignorar filas vacías
            
#         if Q is None:
#             Q = (row / norm_row).reshape(1, -1)
#             A_indep.append(row)
#             b_indep.append(b_eq[i])
#         else:
#             # Proyectar sobre las filas anteriores para ver si es redundante
#             proj = row @ Q.T
#             residual = row - proj @ Q
#             norm_res = np.linalg.norm(residual)
            
#             # Si el residuo es grande, aporta información nueva (es independiente)
#             if norm_res > tol:
#                 q_new = (residual / norm_res).reshape(1, -1)
#                 Q = np.vstack([Q, q_new])
#                 A_indep.append(row)
#                 b_indep.append(b_eq[i])
                
#     print(f"[PRESOLVE] Igualdades originales: {A_eq.shape[0]} | Igualdades tras eliminar redundantes: {len(A_indep)}")
#     return np.array(A_indep), np.array(b_indep)

# def solve_minimum_snap_3d_projective(waypoints, period, camera_center=None, bbox_low=None, bbox_high=None):
#     if camera_center is None:
#         raise ValueError("Se necesita 'camera_center' para los planos epipolares.")

#     n_segments = len(waypoints) - 1
#     n_vars_1d = 8 * n_segments
#     n_vars_total = 3 * n_vars_1d
#     T = period
    
#     # 0. Normales
#     normals = []
#     for i in range(n_segments):
#         ray1 = waypoints[i] - camera_center
#         ray2 = waypoints[i+1] - camera_center
#         normal = np.cross(ray1, ray2)
#         norm_length = np.linalg.norm(normal)
#         if norm_length > 1e-6:
#             normal = normal / norm_length
#         normals.append(normal)

#     # 1. Matriz de Coste P
#     P_1d = np.zeros((n_vars_1d, n_vars_1d))
#     for seg in range(n_segments):
#         for i in range(4, 8):
#             for j in range(4, 8):
#                 term_i = math.factorial(i) / math.factorial(i - 4)
#                 term_j = math.factorial(j) / math.factorial(j - 4)
#                 power = i + j - 7
#                 P_1d[8 * seg + i, 8 * seg + j] = (term_i * term_j * (T ** power)) / power

#     P_sparse = sparse.block_diag([P_1d, P_1d, P_1d], format='csc')
#     P_sparse += sparse.eye(n_vars_total) * 1e-9

#     # Separamos Igualdades (Aeq) y Desigualdades (Aineq)
#     Aeq_rows, beq_vals = [], []
#     Aineq_rows, lineq_vals, uineq_vals = [], [], []

#     def add_eq(coeffs_x, coeffs_y, coeffs_z, val):
#         Aeq_rows.append(np.concatenate([coeffs_x, coeffs_y, coeffs_z]))
#         beq_vals.append(val)
        
#     def add_ineq(coeffs_x, coeffs_y, coeffs_z, low, up):
#         Aineq_rows.append(np.concatenate([coeffs_x, coeffs_y, coeffs_z]))
#         lineq_vals.append(low)
#         uineq_vals.append(up)

#     # =========================================================
#     # 2. IGUALDADES (EN ORDEN EXACTO MATLAB PARA EL PRESOLVE)
#     # =========================================================
#     # [1] pos continuity
#     for seg in range(n_segments - 1):
#         c_left = get_poly_deriv_coeffs(T, 0); c_right = get_poly_deriv_coeffs(0, 0)
#         for ax in [0, 1, 2]:
#             t_vec = np.zeros(n_vars_total)
#             t_vec[ax*n_vars_1d + 8*seg : ax*n_vars_1d + 8*seg+8] = c_left
#             t_vec[ax*n_vars_1d + 8*(seg+1) : ax*n_vars_1d + 8*(seg+1)+8] = -c_right
#             add_eq(t_vec[0:n_vars_1d], t_vec[n_vars_1d:2*n_vars_1d], t_vec[2*n_vars_1d:3*n_vars_1d], 0.0)

#     # [2] vel continuity
#     for seg in range(n_segments - 1):
#         c_left = get_poly_deriv_coeffs(T, 1); c_right = get_poly_deriv_coeffs(0, 1)
#         for ax in [0, 1, 2]:
#             t_vec = np.zeros(n_vars_total)
#             t_vec[ax*n_vars_1d + 8*seg : ax*n_vars_1d + 8*seg+8] = c_left
#             t_vec[ax*n_vars_1d + 8*(seg+1) : ax*n_vars_1d + 8*(seg+1)+8] = -c_right
#             add_eq(t_vec[0:n_vars_1d], t_vec[n_vars_1d:2*n_vars_1d], t_vec[2*n_vars_1d:3*n_vars_1d], 0.0)

#     # [3] acc continuity
#     for seg in range(n_segments - 1):
#         c_left = get_poly_deriv_coeffs(T, 2); c_right = get_poly_deriv_coeffs(0, 2)
#         for ax in [0, 1, 2]:
#             t_vec = np.zeros(n_vars_total)
#             t_vec[ax*n_vars_1d + 8*seg : ax*n_vars_1d + 8*seg+8] = c_left
#             t_vec[ax*n_vars_1d + 8*(seg+1) : ax*n_vars_1d + 8*(seg+1)+8] = -c_right
#             add_eq(t_vec[0:n_vars_1d], t_vec[n_vars_1d:2*n_vars_1d], t_vec[2*n_vars_1d:3*n_vars_1d], 0.0)

#     # [5] pos initial
#     c_0 = get_poly_deriv_coeffs(0, 0)
#     for ax in [0, 1, 2]:
#         t_vec = np.zeros(n_vars_total)
#         t_vec[ax*n_vars_1d : ax*n_vars_1d + 8] = c_0
#         add_eq(t_vec[0:n_vars_1d], t_vec[n_vars_1d:2*n_vars_1d], t_vec[2*n_vars_1d:3*n_vars_1d], waypoints[0][ax])

#     # [6] pos final
#     c_T = get_poly_deriv_coeffs(T, 0)
#     idx = 8 * (n_segments - 1)
#     for ax in [0, 1, 2]:
#         t_vec = np.zeros(n_vars_total)
#         t_vec[ax*n_vars_1d + idx : ax*n_vars_1d + idx + 8] = c_T
#         add_eq(t_vec[0:n_vars_1d], t_vec[n_vars_1d:2*n_vars_1d], t_vec[2*n_vars_1d:3*n_vars_1d], waypoints[-1][ax])

#     # [7] vel initial
#     cv_0 = get_poly_deriv_coeffs(0, 1)
#     for ax in [0, 1, 2]:
#         t_vec = np.zeros(n_vars_total)
#         t_vec[ax*n_vars_1d : ax*n_vars_1d + 8] = cv_0
#         add_eq(t_vec[0:n_vars_1d], t_vec[n_vars_1d:2*n_vars_1d], t_vec[2*n_vars_1d:3*n_vars_1d], 0.0)

#     # [8] vel final
#     cv_T = get_poly_deriv_coeffs(T, 1)
#     for ax in [0, 1, 2]:
#         t_vec = np.zeros(n_vars_total)
#         t_vec[ax*n_vars_1d + idx : ax*n_vars_1d + idx + 8] = cv_T
#         add_eq(t_vec[0:n_vars_1d], t_vec[n_vars_1d:2*n_vars_1d], t_vec[2*n_vars_1d:3*n_vars_1d], 0.0)

#     # [4] normals (Planos Epipolares)
#     # Ahora que pasamos el filtro, podemos pedir igualdad exacta estricta
#     for seg in range(n_segments):
#         nx, ny, nz = normals[seg]
#         d_plane = np.dot(normals[seg], camera_center)
#         for p in range(8):
#             rx, ry, rz = np.zeros(n_vars_1d), np.zeros(n_vars_1d), np.zeros(n_vars_1d)
#             rx[8*seg + p] = nx
#             ry[8*seg + p] = ny
#             rz[8*seg + p] = nz
#             val = d_plane if p == 0 else 0.0
#             add_eq(rx, ry, rz, val)

#     # [15] jerk cont
#     for seg in range(n_segments - 1):
#         c_left = get_poly_deriv_coeffs(T, 3); c_right = get_poly_deriv_coeffs(0, 3)
#         for ax in [0, 1, 2]:
#             t_vec = np.zeros(n_vars_total)
#             t_vec[ax*n_vars_1d + 8*seg : ax*n_vars_1d + 8*seg+8] = c_left
#             t_vec[ax*n_vars_1d + 8*(seg+1) : ax*n_vars_1d + 8*(seg+1)+8] = -c_right
#             add_eq(t_vec[0:n_vars_1d], t_vec[n_vars_1d:2*n_vars_1d], t_vec[2*n_vars_1d:3*n_vars_1d], 0.0)

#     # => APLICAR FILTRO PRESOLVE (Solo a las igualdades, respetando orden)
#     Aeq_mat, beq_vec = remove_dependent_equalities(np.array(Aeq_rows), np.array(beq_vals))

#     # =========================================================
#     # 3. DESIGUALDADES (LÍMITES FÍSICOS - NO SE FILTRAN)
#     # =========================================================
#     min_x, max_x = -1.1658, 1.1618
#     min_y, max_y =  2.5000, 3.4172
#     min_z, max_z =  0.4947, 2.8152

#     min_vx, max_vx = -0.4289, 0.3987
#     min_vy, max_vy = -0.2036, 0.1780
#     min_vz, max_vz = -0.4112, 0.4226

#     min_ax, max_ax = -0.1224, 0.2733
#     min_ay, max_ay = -0.0843, 0.0888
#     min_az, max_az = -0.1291, 0.2716

#     t_samples = np.linspace(0, T, 10)
#     for seg in range(n_segments):
#         for t_eval in t_samples:
#             # POS
#             c_pos = get_poly_deriv_coeffs(t_eval, 0)
#             rx, ry, rz = np.zeros(n_vars_1d), np.zeros(n_vars_1d), np.zeros(n_vars_1d)
#             rx[8*seg:8*seg+8] = c_pos; add_ineq(rx, np.zeros(n_vars_1d), np.zeros(n_vars_1d), min_x, max_x)
            
#             rx, ry, rz = np.zeros(n_vars_1d), np.zeros(n_vars_1d), np.zeros(n_vars_1d)
#             ry[8*seg:8*seg+8] = c_pos; add_ineq(np.zeros(n_vars_1d), ry, np.zeros(n_vars_1d), min_y, max_y)
            
#             rx, ry, rz = np.zeros(n_vars_1d), np.zeros(n_vars_1d), np.zeros(n_vars_1d)
#             rz[8*seg:8*seg+8] = c_pos; add_ineq(np.zeros(n_vars_1d), np.zeros(n_vars_1d), rz, min_z, max_z)

#             # VEL
#             c_vel = get_poly_deriv_coeffs(t_eval, 1)
#             rx, ry, rz = np.zeros(n_vars_1d), np.zeros(n_vars_1d), np.zeros(n_vars_1d)
#             rx[8*seg:8*seg+8] = c_vel; add_ineq(rx, np.zeros(n_vars_1d), np.zeros(n_vars_1d), min_vx, max_vx)
            
#             rx, ry, rz = np.zeros(n_vars_1d), np.zeros(n_vars_1d), np.zeros(n_vars_1d)
#             ry[8*seg:8*seg+8] = c_vel; add_ineq(np.zeros(n_vars_1d), ry, np.zeros(n_vars_1d), min_vy, max_vy)
            
#             rx, ry, rz = np.zeros(n_vars_1d), np.zeros(n_vars_1d), np.zeros(n_vars_1d)
#             rz[8*seg:8*seg+8] = c_vel; add_ineq(np.zeros(n_vars_1d), np.zeros(n_vars_1d), rz, min_vz, max_vz)

#             # ACC
#             c_acc = get_poly_deriv_coeffs(t_eval, 2)
#             rx, ry, rz = np.zeros(n_vars_1d), np.zeros(n_vars_1d), np.zeros(n_vars_1d)
#             rx[8*seg:8*seg+8] = c_acc; add_ineq(rx, np.zeros(n_vars_1d), np.zeros(n_vars_1d), min_ax, max_ax)
            
#             rx, ry, rz = np.zeros(n_vars_1d), np.zeros(n_vars_1d), np.zeros(n_vars_1d)
#             ry[8*seg:8*seg+8] = c_acc; add_ineq(np.zeros(n_vars_1d), ry, np.zeros(n_vars_1d), min_ay, max_ay)
            
#             rx, ry, rz = np.zeros(n_vars_1d), np.zeros(n_vars_1d), np.zeros(n_vars_1d)
#             rz[8*seg:8*seg+8] = c_acc; add_ineq(np.zeros(n_vars_1d), np.zeros(n_vars_1d), rz, min_az, max_az)

#     # =========================================================
#     # 4. ENSAMBLAR TODO Y RESOLVER
#     # =========================================================
#     A_final = np.vstack([Aeq_mat, np.array(Aineq_rows)])
#     l_final = np.concatenate([beq_vec, np.array(lineq_vals)])
#     u_final = np.concatenate([beq_vec, np.array(uineq_vals)])

#     A_sparse = sparse.csc_matrix(A_final)
#     q = np.zeros(n_vars_total)

#     prob = osqp.OSQP()
#     # Aumentamos max_iter a 100000 ya que OSQP es extremadamente rápido
#     prob.setup(P_sparse, q, A_sparse, l_final, u_final, verbose=True, eps_abs=1e-4, eps_rel=1e-4, max_iter=100000)
#     res = prob.solve()

#     # Añadimos 'maximum iterations reached' a la lista de estatus válidos
#     estatus_validos = ['solved', 'solved inaccurate', 'maximum iterations reached']
    
#     if res.info.status not in estatus_validos:
#         print(f"OSQP falló con estatus crítico: {res.info.status}")
#         return None, None, None

#     if res.info.status != 'solved':
#         print(f"Aviso: OSQP terminó con estatus '{res.info.status}'. Se usará la mejor aproximación encontrada.")

#     sol = res.x
#     cx = sol[0 : n_vars_1d]
#     cy = sol[n_vars_1d : 2*n_vars_1d]
#     cz = sol[2*n_vars_1d : 3*n_vars_1d]
    
#     return cx, cy, cz

##################### OJITO; TIME NORMALIZED PENDIENTE #########################33
import numpy as np
import scipy.sparse as sparse
import osqp
import math

def get_poly_deriv_coeffs(tau, derivative_order):
    coeffs = np.zeros(8)
    for i in range(derivative_order, 8):
        val = 1
        for k in range(derivative_order):
            val *= (i - k)
        coeffs[i] = val * (tau ** (i - derivative_order))
    return coeffs

def remove_dependent_equalities(A_eq, b_eq, tol=1e-6):
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

    n_segments = len(waypoints) - 1
    n_vars_1d = 8 * n_segments
    n_vars_total = 3 * n_vars_1d
    
    # TRUCO DE MATLAB: Normalizamos el tiempo del optimizador a [0, 1]
    T = period
    tau_end = 1.0  
    
    # 0. Normales
    normals = []
    for i in range(n_segments):
        ray1 = waypoints[i] - camera_center
        ray2 = waypoints[i+1] - camera_center
        normal = np.cross(ray1, ray2)
        norm_length = np.linalg.norm(normal)
        if norm_length > 1e-6:
            normal = normal / norm_length
        normals.append(normal)

    # 1. Matriz de Coste P (Evaluada siempre con tiempo = 1.0)
    P_1d = np.zeros((n_vars_1d, n_vars_1d))
    for seg in range(n_segments):
        for i in range(4, 8):
            for j in range(4, 8):
                term_i = math.factorial(i) / math.factorial(i - 4)
                term_j = math.factorial(j) / math.factorial(j - 4)
                power = i + j - 7
                P_1d[8 * seg + i, 8 * seg + j] = (term_i * term_j * (tau_end ** power)) / power

    P_sparse = sparse.block_diag([P_1d, P_1d, P_1d], format='csc')
    P_sparse += sparse.eye(n_vars_total) * 1e-9

    Aeq_rows, beq_vals = [], []
    Aineq_rows, lineq_vals, uineq_vals = [], [], []

    def add_eq(coeffs_x, coeffs_y, coeffs_z, val):
        Aeq_rows.append(np.concatenate([coeffs_x, coeffs_y, coeffs_z]))
        beq_vals.append(val)
        
    def add_ineq(coeffs_x, coeffs_y, coeffs_z, low, up):
        Aineq_rows.append(np.concatenate([coeffs_x, coeffs_y, coeffs_z]))
        lineq_vals.append(low)
        uineq_vals.append(up)

    # =========================================================
    # 2. IGUALDADES (Evaluadas en tau_end = 1.0)
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
            rx, ry, rz = np.zeros(n_vars_1d), np.zeros(n_vars_1d), np.zeros(n_vars_1d)
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

    # => FILTRO PRESOLVE (Notarás que al estar la matriz mejor condicionada no suelta error)
    Aeq_mat, beq_vec = remove_dependent_equalities(np.array(Aeq_rows), np.array(beq_vals))

    # =========================================================
    # 3. DESIGUALDADES (LÍMITES FÍSICOS ESCALADOS)
    # =========================================================
    # Al derivar en [0,1], V_real = V_norm / T. Por tanto, límite V_norm = Límite_Real * T
    min_x, max_x = -1.1658, 1.1618
    min_y, max_y =  2.5000, 3.4172
    min_z, max_z =  0.4947, 2.8152

    min_vx, max_vx = -0.4289 * T, 0.3987 * T
    min_vy, max_vy = -0.2036 * T, 0.1780 * T
    min_vz, max_vz = -0.4112 * T, 0.4226 * T

    min_ax, max_ax = -0.1224 * (T**2), 0.2733 * (T**2)
    min_ay, max_ay = -0.0843 * (T**2), 0.0888 * (T**2)
    min_az, max_az = -0.1291 * (T**2), 0.2716 * (T**2)

    t_samples = np.linspace(0, tau_end, 10)
    for seg in range(n_segments):
        for t_eval in t_samples:
            # POS
            c_pos = get_poly_deriv_coeffs(t_eval, 0)
            rx, ry, rz = np.zeros(n_vars_1d), np.zeros(n_vars_1d), np.zeros(n_vars_1d)
            rx[8*seg:8*seg+8] = c_pos; add_ineq(rx, np.zeros(n_vars_1d), np.zeros(n_vars_1d), min_x, max_x)
            rx, ry, rz = np.zeros(n_vars_1d), np.zeros(n_vars_1d), np.zeros(n_vars_1d)
            ry[8*seg:8*seg+8] = c_pos; add_ineq(np.zeros(n_vars_1d), ry, np.zeros(n_vars_1d), min_y, max_y)
            rx, ry, rz = np.zeros(n_vars_1d), np.zeros(n_vars_1d), np.zeros(n_vars_1d)
            rz[8*seg:8*seg+8] = c_pos; add_ineq(np.zeros(n_vars_1d), np.zeros(n_vars_1d), rz, min_z, max_z)

            # VEL
            c_vel = get_poly_deriv_coeffs(t_eval, 1)
            rx, ry, rz = np.zeros(n_vars_1d), np.zeros(n_vars_1d), np.zeros(n_vars_1d)
            rx[8*seg:8*seg+8] = c_vel; add_ineq(rx, np.zeros(n_vars_1d), np.zeros(n_vars_1d), min_vx, max_vx)
            rx, ry, rz = np.zeros(n_vars_1d), np.zeros(n_vars_1d), np.zeros(n_vars_1d)
            ry[8*seg:8*seg+8] = c_vel; add_ineq(np.zeros(n_vars_1d), ry, np.zeros(n_vars_1d), min_vy, max_vy)
            rx, ry, rz = np.zeros(n_vars_1d), np.zeros(n_vars_1d), np.zeros(n_vars_1d)
            rz[8*seg:8*seg+8] = c_vel; add_ineq(np.zeros(n_vars_1d), np.zeros(n_vars_1d), rz, min_vz, max_vz)

            # ACC
            c_acc = get_poly_deriv_coeffs(t_eval, 2)
            rx, ry, rz = np.zeros(n_vars_1d), np.zeros(n_vars_1d), np.zeros(n_vars_1d)
            rx[8*seg:8*seg+8] = c_acc; add_ineq(rx, np.zeros(n_vars_1d), np.zeros(n_vars_1d), min_ax, max_ax)
            rx, ry, rz = np.zeros(n_vars_1d), np.zeros(n_vars_1d), np.zeros(n_vars_1d)
            ry[8*seg:8*seg+8] = c_acc; add_ineq(np.zeros(n_vars_1d), ry, np.zeros(n_vars_1d), min_ay, max_ay)
            rx, ry, rz = np.zeros(n_vars_1d), np.zeros(n_vars_1d), np.zeros(n_vars_1d)
            rz[8*seg:8*seg+8] = c_acc; add_ineq(np.zeros(n_vars_1d), np.zeros(n_vars_1d), rz, min_az, max_az)

    # =========================================================
    # 4. RESOLVER
    # =========================================================
    A_final = np.vstack([Aeq_mat, np.array(Aineq_rows)])
    l_final = np.concatenate([beq_vec, np.array(lineq_vals)])
    u_final = np.concatenate([beq_vec, np.array(uineq_vals)])

    A_sparse = sparse.csc_matrix(A_final)
    q = np.zeros(n_vars_total)

    prob = osqp.OSQP()
    prob.setup(
        P_sparse, q, A_sparse, l=l_final, u=u_final, 
        verbose=True, 
        eps_abs=1e-6,         # Mayor precisión
        eps_rel=1e-6, 
        max_iter=100000, 
        adaptive_rho=True,    # Ayuda al solver a no estancarse
        polish=True           # <--- LA CLAVE: Fuerza una solución exacta al final (como MATLAB)
    )
    res = prob.solve()

    estatus_validos = ['solved', 'solved inaccurate', 'maximum iterations reached']
    if res.info.status not in estatus_validos:
        print(f"OSQP falló con estatus crítico: {res.info.status}")
        return None, None, None

    # Extraemos coeficientes "Normalizados"
    sol = res.x
    cx = sol[0 : n_vars_1d]
    cy = sol[n_vars_1d : 2*n_vars_1d]
    cz = sol[2*n_vars_1d : 3*n_vars_1d]
    
    # DESHACER TRUCO: Transformamos los coeficientes a tiempo real
    # para que tu main.py no tenga que enterarse de nada y calcule bien.
    for seg in range(n_segments):
        for i in range(8):
            scale = T ** i
            cx[8*seg + i] /= scale
            cy[8*seg + i] /= scale
            cz[8*seg + i] /= scale
            
    return cx, cy, cz