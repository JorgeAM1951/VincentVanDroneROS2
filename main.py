# Load data
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

    traj = Trajectory(np.array([[-1.5, 2, 1], [1.5, 2, 1], [1.5, 2, 2.5], [-1.5, 2, 2.5], [-1.5, 2, 1]]), period=2.0)

    # AQUÍ TENGO QUE DIBUJAR LA TRAYECTORIA USANDO LOS WP
    # --- Build a square + diagonals as the desired drawing ---
    sq = np.array([
        [-1.5,  2, 1], [ 1.5,  2, 1], [ 1.5,  2, 2.5], [-1.5,  2, 2.5], [-1.5,  2, 1],
        [ 1.5,  2, 2.5], [-1.5, 2, 2.5], [ 1.5, 2, 1],    # diagonals
        [-1.5, 2, 2.5],  # back to top-left
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
        
        for t in t_puntos:
            trayectoria_x.append(sum(c_x[i] * (t**i) for i in range(8)))
            trayectoria_y.append(sum(c_y[i] * (t**i) for i in range(8)))
            trayectoria_z.append(sum(c_z[i] * (t**i) for i in range(8)))

    trayectoria = np.column_stack((trayectoria_x, trayectoria_y, trayectoria_z))
    # --- Assemble scene ---
    scene = Scene(bounds=[[-3, 3], [0, 6], [0, 5]])
    scene.addCamera(cam, label='{camera}')
    scene.addDesiredDrawing(sq)
    scene.addTrajectory(trayectoria, label='Trajectory QP')
    scene.addWaypoints(wps)

    # --- Render ---
    fig3d, _  = scene.plot3D(title='3D World')
    figCam, _ = scene.plotCameraView(camera_idx=0, title='Camera Frame View')
    figDer, _ = scene.plotDerivatives(trajectory_idx=0)


    plt.show()


if __name__ == "__main__":
    main()