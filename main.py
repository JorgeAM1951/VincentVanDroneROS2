# Load data
import os
import time
import numpy as np
import matplotlib.pyplot as plt
from modules.camera import Camera
from modules.visualization import Scene
from modules.trajectory import Trajectory
from modules.solvers import quadprog as qp


def main():
    #ts = time.millis()
        #               #
        # Load configs  #
        #               #

    
        #  01) PREPROCESS INFO PRIOR TO TRAJECTORY GENERATION
        # % aa) - LOAD WAYPOINTS 2D UV PIXEL COORDINATES FROM IMAGE AND PLOT
        # % bb) - PLOT CAMERA in 3D WORLD
        # % cc) - SEGMENT MANAGEMENT AND SPLITTER AND SAVE 2D/3D POINTS """

        #         % 02) SOLVERS SECTION

        # % 03) OTHER STUFF
        #     % 01 - SAVING SOLVER RESULTS
        #     % 02 - LEGENDS FOR FIGURES
        #     % 03 - COMPARE LENGTHS
        #     % 04 - SAVING FIGURES
        #     % 05 - EXITING
     # Lazy import to avoid circular dependency
    

    # --- Build a simple demo camera ---
    cam = Camera(
        pose        = [0.0, 0.0, 1.5],
        orientation = [0, -90, 0],          # roll, pitch, yaw (deg)
        fov         = [60, 45],
        focal       = [0.006, 0.006],
        pixelSize   = [4e-6, 4e-6],
        resolution  = [1456, 1088],
    )

    traj = Trajectory(np.array([[-1.04, 2.89, 0.65], [-1.04, 3.08, 2.74], [1.04, 3.08, 2.74], 
                                [1.04, 2.89, 0.65], [-1.04, 2.89, 0.65]]), period=10.0)

    # AQUÍ TENGO QUE DIBUJAR LA TRAYECTORIA USANDO LOS WP
    # --- Build a square + diagonals as the desired drawing ---
    sq = np.array([
        [-1.04,  2, 0.65], [ 1.04,  2, 0.65], [ 1.04,  2, 2.5], [-1.04,  2, 2.5], [-1.04,  2, 0.65],
        [ 1.04,  2, 2.5], [-1.04, 2, 2.5], [ 1.04, 2, 0.65],    # diagonals
        [-1.04, 2, 2.5],  # back to top-left
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
    
    # Extraer puntos para dibujar la trayectoria
    trayectoria_x, trayectoria_y, trayectoria_z = [], [], []
    
    n_seg = wps.shape[0] - 1 # Corregido: número real de segmentos
    for seg in range(n_seg):
        c_x = cx[8*seg : 8*seg+8]
        c_y = cy[8*seg : 8*seg+8]
        c_z = cz[8*seg : 8*seg+8]

        t_puntos = np.linspace(0, T, 50)
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
    ruta_matlab = "/home/jorge/Downloads/VINCENT_JORGEARMAS/VINCENT_JORGEARMAS/00_VINCENT_VAN_DRONE/99_export/00_testsv8_2026/20260528-140103/02_export" # <-- Rellena esto luego
    
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
    figDer, _ = scene.plotDerivatives(
        trajectory_idx=0, 
        times=None,
        matlab_data=datos_matlab, 
        matlab_times=t_mat,
        waypoint_times=[0, 10, 20, 30, 40]
    )

    plt.show()


if __name__ == "__main__":
    main()