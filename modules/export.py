import os
import numpy as np
from datetime import datetime

def export_trajectory_for_ros2(q, qd, qdd, tvec, base_path="trayectorias", prefix="export_"):
    """
    Exporta las matrices de la trayectoria (posición, velocidad, aceleración y tiempo)
    a archivos .txt formateados con comas, listos para ser leídos por el nodo de ROS2.
    
    Args:
        q (np.ndarray): Matriz de posiciones (N, 3) o (3, N)
        qd (np.ndarray): Matriz de velocidades (N, 3) o (3, N)
        qdd (np.ndarray): Matriz de aceleraciones (N, 3) o (3, N)
        tvec (np.ndarray): Vector de tiempos (N,)
        base_path (str): Carpeta raíz donde se guardarán los experimentos.
        prefix (str): Prefijo opcional para los archivos.
    """
    # 1. Asegurar el formato (N, 3) - Si viene como (3, N) transponemos
    if q.shape[0] == 3 and q.shape[1] > 3:
        q = q.T
    if qd.shape[0] == 3 and qd.shape[1] > 3:
        qd = qd.T
    if qdd.shape[0] == 3 and qdd.shape[1] > 3:
        qdd = qdd.T
        
    # Asegurar que tvec sea unidimensional o columna plana
    tvec = tvec.flatten()

    # 2. Crear una carpeta única con la fecha y hora actual (estilo tu MATLAB anterior)
    timestamp = datetime.now().strftime("%Y%m%d-%H%M%S")
    folder_name = f"{timestamp}_trajectory_ros2"
    full_folder_path = os.path.join(base_path, folder_name)
    
    os.makedirs(full_folder_path, exist_ok=True)
    
    # 3. Definir las rutas completas de los archivos .txt
    q_file = os.path.join(full_folder_path, f"{prefix}q.txt")
    qd_file = os.path.join(full_folder_path, f"{prefix}qd.txt")
    qdd_file = os.path.join(full_folder_path, f"{prefix}qdd.txt")
    tvec_file = os.path.join(full_folder_path, f"{prefix}tvec.txt")
    
    # También creamos copias limpias sin prefijo para que utils.py las lea directamente
    q_clean = os.path.join(full_folder_path, "q.txt")
    qd_clean = os.path.join(full_folder_path, "qd.txt")
    qdd_clean = os.path.join(full_folder_path, "qdd.txt")
    tvec_clean = os.path.join(full_folder_path, "tvec.txt")

    try:
        # 4. Guardar usando numpy con delimitador por coma (fmt='%.6f' para alta precisión)
        np.savetxt(q_clean, q, delimiter=',', fmt='%.6f')
        np.savetxt(qd_clean, qd, delimiter=',', fmt='%.6f')
        np.savetxt(qdd_clean, qdd, delimiter=',', fmt='%.6f')
        np.savetxt(tvec_clean, tvec, delimiter=',', fmt='%.6f')
        
        # Guardar las versiones con prefijo por compatibilidad histórica
        np.savetxt(q_file, q, delimiter=',', fmt='%.6f')
        np.savetxt(qd_file, qd, delimiter=',', fmt='%.6f')
        np.savetxt(qdd_file, qdd, delimiter=',', fmt='%.6f')
        np.savetxt(tvec_file, tvec, delimiter=',', fmt='%.6f')

        print("\n" + "="*60)
        print(f"[EXPORT] ¡Trayectoria exportada con ÉXITO para ROS2!")
        print(f"[EXPORT] Carpeta creada: {full_folder_path}")
        print(f"[EXPORT] Puntos guardados: {q.shape[0]}")
        print("="*60 + "\n")
        
        return full_folder_path

    except Exception as e:
        print(f"[EXPORT] ERROR crítico al exportar la trayectoria: {e}")
        return None