# Load data
import os
import time
import numpy as np
import matplotlib.pyplot as plt
from modules.camera import Camera
from modules.visualization import Scene
from modules.trajectory import Trajectory
from modules.solvers import quadprog as qp
from modules.solvers import clarabel as cl
from modules.export import export_trajectory_for_ros2


def main():
    # --- Build a simple demo camera ---
    cam = Camera(
        pose        = [0.0, 0.0, 1.5],
        orientation = [0, -90, 0],          # roll, pitch, yaw (deg)
        fov         = [60, 45],
        focal       = [0.006, 0.006],
        pixelSize   = [4e-6, 4e-6],
        resolution  = [1456, 1088],
    )

    traj = Trajectory(np.array([[-1.049, 2.89, 0.652],[-2.0, 2.99, 1.698] ,[-1.049, 3.08, 2.744], [1.049, 3.08, 2.744], [2.0, 2.99, 1.698],  
                                [1.049, 2.89, 0.652], [-1.049, 2.89, 0.652]]), period=10.0)

    # AQUÍ TENGO QUE DIBUJAR LA TRAYECTORIA USANDO LOS WP
    # --- Build a square + diagonals as the desired drawing ---
    sq = np.array([
        [-1.0491,  2, 0.6522], [ 1.0491,  2, 0.6522], [ 1.0491,  2, 2.5], [-1.0491,  2, 2.5], [-1.0491,  2, 0.6522],
        [ 1.0491,  2, 2.5], [-1.0491, 2, 2.5], [ 1.0491, 2, 0.6522],    # diagonals
        [-1.0491, 2, 2.5],  # back to top-left
    ])

    # --- Waypoints at the square corners ---
    # ESTO ME LO TENGO QUE TRAER DEL OBJETO
    wps = traj.waypoints
    T = traj.period

    # --- Simple sinusoidal trajectory ---
    #AQUÍ VA EL SOLVER (PREGUNTAR A JUAN O A EDU POR LAS MATES)
    # --- Extraer pose de la cámara como array ---
    camera_center = np.array(cam.pose)

    # --- Llamada al NUEVO solver proyectivo gigante ---
    # Le pasamos la pose de la cámara como punto focal
    cx, cy, cz = qp.solve_minimum_snap_3d_projective(wps, T, camera_center)
    #cx, cy, cz = cl.solve_minimum_snap_3d_projective(wps, T, camera_center)
    
    # Extraer puntos para dibujar la trayectoria
    trayectoria_x, trayectoria_y, trayectoria_z = [], [], []
    
    n_seg = wps.shape[0] - 1 # Corregido: número real de segmentos
    for seg in range(n_seg):
        c_x = cx[8*seg : 8*seg+8]
        c_y = cy[8*seg : 8*seg+8]
        c_z = cz[8*seg : 8*seg+8]

        t_puntos = np.linspace(0, T, int(T * 25.0))
        if seg < n_seg - 1:
            t_puntos = t_puntos[:-1]
        
        for t in t_puntos:
            trayectoria_x.append(sum(c_x[i] * (t**i) for i in range(8)))
            trayectoria_y.append(sum(c_y[i] * (t**i) for i in range(8)))
            trayectoria_z.append(sum(c_z[i] * (t**i) for i in range(8)))

    trayectoria = np.column_stack((trayectoria_x, trayectoria_y, trayectoria_z))
    # --- Assemble scene ---
    scene = Scene(bounds=[[-3, 3], [2.5, 4], [0.1, 4.8]])
    
    # Añadimos la cámara y los waypoints (los puntos rojos '+')
    scene.addCamera(cam, label='Camera 0')
    scene.addWaypoints(wps)
    
    # --- MODIFICACIÓN AQUÍ ---
    # En lugar de pasarle una variable antigua o un cuadrado hardcodeado, 
    # le pasamos directamente 'wps' para que dibuje la línea verde sobre ellos.
    scene.addDesiredDrawing(wps)
    
    # Añadimos la trayectoria calculada por tu solver (Naranja)
    scene.addTrajectory(trayectoria, label='Minimum Snap')


    # --- Render ---
    #Datos del cuadrado
    #ruta_matlab = "/home/jorge/Downloads/VINCENT_JORGEARMAS/VINCENT_JORGEARMAS/00_VINCENT_VAN_DRONE/99_export/00_testsv8_2026/20260528-140103/02_export" # <-- Rellena esto luego
    #Datos del hexagono
    ruta_matlab = "/home/jorge/Downloads/VINCENT_JORGEARMAS/VINCENT_JORGEARMAS/00_VINCENT_VAN_DRONE/99_export/00_testsv8_2026/20260611-000498/02_export"
    
    try:
        t_mat = np.loadtxt(os.path.join(ruta_matlab, "tvec.txt"), delimiter=',')
        q_mat = np.loadtxt(os.path.join(ruta_matlab, "q.txt"), delimiter=',')
        qd_mat = np.loadtxt(os.path.join(ruta_matlab, "qd.txt"), delimiter=',')
        qdd_mat = np.loadtxt(os.path.join(ruta_matlab, "qdd.txt"), delimiter=',')
        qddd_mat = np.loadtxt(os.path.join(ruta_matlab, "qddd.txt"), delimiter=',')
        qdddd_mat = np.loadtxt(os.path.join(ruta_matlab, "qdddd.txt"), delimiter=',')
        
        # Truco: Si MATLAB guardó las matrices al revés (3 filas x N columnas)
        # transponemos para que sean (N, 3) como espera Python
        if q_mat.shape[0] == 3 and q_mat.shape[1] > 3:
            q_mat, qd_mat, qdd_mat = q_mat.T, qd_mat.T, qdd_mat.T
            qddd_mat, qdddd_mat = qddd_mat.T, qdddd_mat.T
            
        datos_matlab = [q_mat, qd_mat, qdd_mat, qddd_mat, qdddd_mat]
        
        # Superponemos la trayectoria de MATLAB en la vista 3D
        scene.addTrajectory(q_mat, label='MATLAB', color='cyan')
        
    except Exception as e:
        print(f"No se pudieron cargar los txt de MATLAB: {e}")
        datos_matlab = None
        t_mat = None

    # --- Render ---
    fig3d, _  = scene.plot3D(title='3D World')
    figCam, _ = scene.plotCameraView(camera_idx=0, title='Camera Frame View')
    
    # Pasamos explícitamente los tiempos de los waypoints: 0, 10, 20, 30, 40
    # (4 segmentos × 10 s/segmento)
    total_time = 40.0
    
    # 2. Generamos los 7 puntos de tiempo (t=0, t=6.66, ..., t=40)
    # Esto distribuye equitativamente los 40s entre los 6 segmentos de tu trayectoria
    num_wps = wps.shape[0]
    tiempos_wps = np.linspace(0, total_time, num_wps).tolist()
    figDer, _ = scene.plotDerivatives(
        trajectory_idx=0, 
        times=None,
        matlab_data=datos_matlab, 
        matlab_times=t_mat,
        waypoint_times=tiempos_wps
    )

    plt.show()

    # =======================================================================
    # EXPORTACIÓN DE LA TRAYECTORIA DE PYTHON PARA ROS2
    # =======================================================================
    q_py, qd_py, qdd_py, tvec_py = [], [], [], []
    t_acumulado = 0.0

    for seg in range(n_seg):
        # 1. Extraer coeficientes de posición del segmento actual
        c_x = cx[8*seg : 8*seg+8]
        c_y = cy[8*seg : 8*seg+8]
        c_z = cz[8*seg : 8*seg+8]
        
        # 2. Derivar coeficientes para la Velocidad
        cv_x = [c_x[i] * i for i in range(1, 8)]
        cv_y = [c_y[i] * i for i in range(1, 8)]
        cv_z = [c_z[i] * i for i in range(1, 8)]
        
        # 3. Derivar coeficientes para la Aceleración
        ca_x = [cv_x[i] * i for i in range(1, 7)]
        ca_y = [cv_y[i] * i for i in range(1, 7)]
        ca_z = [cv_z[i] * i for i in range(1, 7)]

        # 4. Generar el tiempo de este segmento
        t_puntos = np.linspace(0, T, 50)
        if seg < n_seg - 1:
            t_puntos = t_puntos[:-1]
            
        for t in t_puntos:
            # Tiempo global
            tvec_py.append(t_acumulado + t)
            
            # Evaluar Posición
            px = sum(c_x[i] * (t**i) for i in range(8))
            py = sum(c_y[i] * (t**i) for i in range(8))
            pz = sum(c_z[i] * (t**i) for i in range(8))
            q_py.append([px, py, pz])
            
            # Evaluar Velocidad
            vx = sum(cv_x[i] * (t**i) for i in range(7))
            vy = sum(cv_y[i] * (t**i) for i in range(7))
            vz = sum(cv_z[i] * (t**i) for i in range(7))
            qd_py.append([vx, vy, vz])
            
            # Evaluar Aceleración
            ax = sum(ca_x[i] * (t**i) for i in range(6))
            ay = sum(ca_y[i] * (t**i) for i in range(6))
            az = sum(ca_z[i] * (t**i) for i in range(6))
            qdd_py.append([ax, ay, az])
            
        t_acumulado += T

    # 5. Exportar a ROS2 usando los datos coherentes
    carpeta_guardado = export_trajectory_for_ros2(
        q = np.array(q_py), 
        qd = np.array(qd_py), 
        qdd = np.array(qdd_py), 
        tvec = np.array(tvec_py), 
        base_path = "./trayectorias_exportadas"
    )
    print(f"Trayectoria de Python exportada con éxito en: {carpeta_guardado}")


if __name__ == "__main__":
    main()