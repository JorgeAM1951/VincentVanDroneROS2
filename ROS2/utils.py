import os
import numpy as np
import math

def load_trajectory_from_txt(folder_path):
    """
    Lee los archivos .txt de posición, velocidad y aceleración generados
    por el optimizador de trayectoria (Minimum Snap) en Python/MATLAB.
    
    Args:
        folder_path (str): Ruta absoluta o relativa a la carpeta con los .txt
        
    Returns:
        dict: Diccionario con las matrices 'pos', 'vel', 'acc' y 'times'.
              Si falla, devuelve None.
    """
    try:
        q_file = os.path.join(folder_path, "q.txt")
        qd_file = os.path.join(folder_path, "qd.txt")
        qdd_file = os.path.join(folder_path, "qdd.txt")
        time_file = os.path.join(folder_path, "tvec.txt") # O el nombre que uses para el vector de tiempos

        # Cargar matrices (asumiendo delimitador por coma, cámbialo a ' ' si usas espacios)
        q_mat = np.loadtxt(q_file, delimiter=',')
        qd_mat = np.loadtxt(qd_file, delimiter=',')
        qdd_mat = np.loadtxt(qdd_file, delimiter=',')
        
        # Cargar tiempos (si no existe, lo calculamos asumiendo una frecuencia constante)
        if os.path.exists(time_file):
            t_mat = np.loadtxt(time_file, delimiter=',')
        else:
            t_mat = None

        # Truco de seguridad: Si se guardaron transpuestas (3 filas x N columnas),
        # las rotamos para que sean (N_puntos, 3_coordenadas) como espera ROS2.
        if len(q_mat.shape) == 2 and q_mat.shape[0] == 3 and q_mat.shape[1] > 3:
            q_mat = q_mat.T
            qd_mat = qd_mat.T
            qdd_mat = qdd_mat.T

        print(f"[utils] Trayectoria cargada con éxito: {q_mat.shape[0]} puntos.")
        
        return {
            'pos': q_mat,
            'vel': qd_mat,
            'acc': qdd_mat,
            'times': t_mat
        }

    except Exception as e:
        print(f"[utils] ERROR al cargar los archivos txt desde {folder_path}: {e}")
        return None

def calculate_3d_distance(current_pos, target_pos):
    """
    Calcula la distancia euclidiana entre dos puntos 3D.
    Muy útil para saber si el dron ha llegado a un waypoint.
    
    Args:
        current_pos (list/tuple/Pose): [x, y, z] actual
        target_pos (list/tuple): [x, y, z] objetivo
        
    Returns:
        float: Distancia en metros.
    """
    # Si le pasas directamente el objeto de ROS (PoseStamped), lo convertimos
    if hasattr(current_pos, 'pose'):
        cx = current_pos.pose.position.x
        cy = current_pos.pose.position.y
        cz = current_pos.pose.position.z
    else:
        cx, cy, cz = current_pos[0], current_pos[1], current_pos[2]

    tx, ty, tz = target_pos[0], target_pos[1], target_pos[2]

    dx = cx - tx
    dy = cy - ty
    dz = cz - tz
    
    return math.sqrt(dx**2 + dy**2 + dz**2)

def calculate_trajectory_rate(times_array, default_rate=25.0):
    """
    Calcula a qué frecuencia (Hz) debe publicar el nodo de ROS2
    leyendo la diferencia de tiempo entre dos puntos de la trayectoria.
    
    Args:
        times_array (np.array): Vector de tiempos.
        default_rate (float): Frecuencia de seguridad si no hay tiempos.
        
    Returns:
        float: Frecuencia en Hz calculada (calc_rate).
    """
    if times_array is None or len(times_array) < 2:
        return default_rate
        
    dt = times_array[1] - times_array[0]
    
    if dt <= 0.0:
        return default_rate
        
    hz = 1.0 / dt
    return round(hz, 2)