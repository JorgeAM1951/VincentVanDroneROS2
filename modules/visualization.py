import numpy as np
import matplotlib.pyplot as plt
import matplotlib.lines as mlines
import matplotlib.patches as mpatches
from mpl_toolkits.mplot3d import Axes3D
from scipy.spatial.transform import Rotation as R

# --- NUEVO: Configuración global para legibilidad en Ubuntu (HiDPI) ---
plt.rcParams.update({
    'font.size': 12,
    'axes.labelsize': 12,
    'xtick.labelsize': 11,
    'ytick.labelsize': 11,
    'axes.titlesize': 14,
    'legend.fontsize': 11,
    'figure.dpi': 120  # Aumenta esto a 150 o 200 si tienes monitor 4K
})


class Scene:
    def __init__(self, bounds=None):
        """
        3D visualization scene, mimicking MATLAB-style camera/trajectory plots.
        """
        self.cameras        = []    
        self.cameraLabels   = []    
        self.waypoints      = []    
        self.desiredDrawings = []   
        self.trajectories   = []    
        self.bounds         = bounds

    # ------------------------------------------------------------------ #
    #  Scene population
    # ------------------------------------------------------------------ #

    def addCamera(self, camera, label='{camera}'):
        self.cameras.append(camera)
        self.cameraLabels.append(label)

    def addWaypoints(self, waypoints):
        self.waypoints.append(np.asarray(waypoints, dtype=float))

    def addDesiredDrawing(self, path):
        self.desiredDrawings.append(np.asarray(path, dtype=float))

    def addTrajectory(self, points, label='Trajectory QP', color='orange'):
        self.trajectories.append({
            'points': np.asarray(points, dtype=float),
            'label' : label,
            'color' : color   # <-- Guardamos el color
        })

    # ------------------------------------------------------------------ #
    #  Private helpers
    # ------------------------------------------------------------------ #

    @staticmethod
    def _buildRotationMatrix(orientation):
        RX = R.from_euler('x', orientation[1], degrees=True).as_matrix()   
        RY = R.from_euler('y', orientation[0], degrees=True).as_matrix()   
        RZ = R.from_euler('z', orientation[2], degrees=True).as_matrix()   
        return RZ @ RY @ RX

    def _getCameraFrustumVertices(self, camera, scale=0.5):
        center = np.asarray(camera.pose, dtype=float)
        RWC    = self._buildRotationMatrix(camera.orientation)

        hx = np.tan(np.radians(camera.fovX / 2.0)) * scale
        hy = np.tan(np.radians(camera.fovY / 2.0)) * scale

        corners_cam = np.array([
            [ hx,  hy, scale],
            [-hx,  hy, scale],
            [-hx, -hy, scale],
            [ hx, -hy, scale],
        ])

        corners_world = (RWC @ corners_cam.T).T + center
        return center, corners_world

    @staticmethod
    def _drawCameraFrustum(ax, center, corners, color='red', label=None):
        for corner in corners:
            ax.plot([center[0], corner[0]],
                    [center[1], corner[1]],
                    [center[2], corner[2]],
                    color=color, linewidth=1.5)

        rect = np.vstack([corners, corners[0]])
        ax.plot(rect[:, 0], rect[:, 1], rect[:, 2],
                color=color, linewidth=1.5)

        if label:
            # Tamaño de fuente aumentado
            ax.text(center[0] + 0.05, center[1] + 0.05, center[2] - 0.15,
                    label, color=color, fontsize=10)

    @staticmethod
    def _drawAxesFrame(ax, origin, rotation_matrix, scale=0.4, label=None):
        colors = ['red', 'green', 'blue']
        for i, color in enumerate(colors):
            direction = rotation_matrix[:, i] * scale
            ax.quiver(origin[0], origin[1], origin[2],
                      direction[0], direction[1], direction[2],
                      color=color, linewidth=1.5, arrow_length_ratio=0.3)
        if label:
            ax.text(origin[0], origin[1], origin[2] - 0.2,
                    label, color='white', fontsize=10)

    @staticmethod
    def _drawBoundingBox(ax, bounds, color='limegreen', linestyle='--'):
        (x0, x1), (y0, y1), (z0, z1) = bounds
        edges = [
            ([x0,x1],[y0,y0],[z0,z0]), ([x1,x1],[y0,y1],[z0,z0]),
            ([x1,x0],[y1,y1],[z0,z0]), ([x0,x0],[y1,y0],[z0,z0]),
            ([x0,x1],[y0,y0],[z1,z1]), ([x1,x1],[y0,y1],[z1,z1]),
            ([x1,x0],[y1,y1],[z1,z1]), ([x0,x0],[y1,y0],[z1,z1]),
            ([x0,x0],[y0,y0],[z0,z1]), ([x1,x1],[y0,y0],[z0,z1]),
            ([x1,x1],[y1,y1],[z0,z1]), ([x0,x0],[y1,y1],[z0,z1]),
        ]
        for ex, ey, ez in edges:
            ax.plot(ex, ey, ez, color=color, linestyle=linestyle, linewidth=1.0)

    @staticmethod
    def _applyDarkStyle(ax):
        ax.set_facecolor('black')
        for pane in (ax.xaxis.pane, ax.yaxis.pane, ax.zaxis.pane):
            pane.fill = False
            pane.set_edgecolor('#444444')
        
        # Tamaño de fuente de los ticks aumentado a 10
        ax.tick_params(colors='white', labelsize=10)
        ax.xaxis.label.set_color('white')
        ax.yaxis.label.set_color('white')
        ax.zaxis.label.set_color('white')
        ax.title.set_color('white')
        ax.grid(True, color='#333333', linewidth=0.5)

    # ------------------------------------------------------------------ #
    #  Public plot methods
    # ------------------------------------------------------------------ #

    def plot3D(self, figsize=(10, 8), title='3D World', cameraScale=0.4):
        fig = plt.figure(figsize=figsize, facecolor='black')
        ax  = fig.add_subplot(111, projection='3d')
        self._applyDarkStyle(ax)

        legend_handles = []
        all_points_for_bounds = [] # NUEVO: Almacenar puntos para bounds globales

        # ---- Bounding box ----
        if self.bounds is not None:
            self._drawBoundingBox(ax, self.bounds)
            all_points_for_bounds.extend([[self.bounds[0][0], self.bounds[1][0], self.bounds[2][0]], 
                                          [self.bounds[0][1], self.bounds[1][1], self.bounds[2][1]]])

        # ---- Desired drawings ----
        for i, drawing in enumerate(self.desiredDrawings):
            ax.plot(drawing[:, 0], drawing[:, 1], drawing[:, 2],
                    color='limegreen', linewidth=1.5)
            all_points_for_bounds.extend(drawing)
        if self.desiredDrawings:
            legend_handles.append(mlines.Line2D([], [], color='limegreen', linewidth=1.5, label='desired drawing'))

        # ---- Trajectories ----
        # ---- Trajectories ----
        for traj in self.trajectories:
            pts = traj['points']
            c = traj.get('color', 'orange')
            ax.plot(pts[:, 0], pts[:, 1], pts[:, 2], color=c, linewidth=1.5)
            all_points_for_bounds.extend(pts)
            # Añadimos la leyenda individual para cada curva
            legend_handles.append(mlines.Line2D([], [], color=c, linewidth=1.5, label=traj['label']))
        # ---- Waypoints ----
        for wps in self.waypoints:
            ax.scatter(wps[:, 0], wps[:, 1], wps[:, 2], color='red', marker='+', s=120, linewidths=2, depthshade=False)
            all_points_for_bounds.extend(wps)
        if self.waypoints:
            legend_handles.append(mlines.Line2D([], [], color='red', marker='+', linestyle='None', markersize=10, markeredgewidth=2, label='Wps'))

        # ---- Cameras ----
        for cam, lbl in zip(self.cameras, self.cameraLabels):
            center, corners = self._getCameraFrustumVertices(cam, scale=cameraScale)
            self._drawCameraFrustum(ax, center, corners, color='red', label=lbl)
            RWC = self._buildRotationMatrix(cam.orientation)
            self._drawAxesFrame(ax, center, RWC, scale=cameraScale * 0.7)
            all_points_for_bounds.append(center)
            all_points_for_bounds.extend(corners)

        # ---- World origin frame ----
        self._drawAxesFrame(ax, [0, 0, 0], np.eye(3), scale=0.5, label='{world}')
        all_points_for_bounds.append([0,0,0])

        # NUEVO: Igualar los límites de X, Y, Z para que mantenga proporción
        if all_points_for_bounds:
            arr_pts = np.array(all_points_for_bounds)
            min_val = np.min(arr_pts)
            max_val = np.max(arr_pts)
            margen = (max_val - min_val) * 0.1  # 10% de margen
            
            ax.set_xlim([min_val - margen, max_val + margen])
            ax.set_ylim([min_val - margen, max_val + margen])
            ax.set_zlim([min_val - margen, max_val + margen])

        ax.set_xlabel('X [m]')
        ax.set_ylabel('Y [m]')
        ax.set_zlabel('Z [m]')
        ax.set_title(title, fontsize=16)

        ax.legend(handles=legend_handles, facecolor='#111111', edgecolor='#555555',
                  labelcolor='white', loc='upper right', fontsize=11)

        fig.tight_layout()
        return fig, ax

    def plotCameraView(self, camera_idx=0, figsize=(7, 6), title='Camera Frame View'):
        if camera_idx >= len(self.cameras):
            raise IndexError(f"Camera index {camera_idx} is out of range.")

        cam = self.cameras[camera_idx]
        P   = cam.getP()
        res = cam.resolution

        fig, ax = plt.subplots(figsize=figsize)
        ax.set_facecolor('#909090')
        fig.patch.set_facecolor('#909090')

        legend_handles = []

        def project(pts_3d):
            pts_h = np.hstack([pts_3d, np.ones((len(pts_3d), 1))])  
            proj  = (P @ pts_h.T).T                                  
            proj /= proj[:, 2:3]                                     
            return proj[:, :2]

        for i, drawing in enumerate(self.desiredDrawings):
            uv = project(drawing)
            ax.plot(uv[:, 0], uv[:, 1], color='limegreen', linewidth=1.5)
        if self.desiredDrawings:
            legend_handles.append(mlines.Line2D([], [], color='limegreen', linewidth=1.5, label='desired drawing'))

        for i, traj in enumerate(self.trajectories):
            uv = project(traj['points'])
            ax.scatter(uv[:, 0], uv[:, 1], color='yellow', edgecolors='orange', s=18, linewidths=0.5, zorder=5)
        if self.trajectories:
            legend_handles.append(mlines.Line2D([], [], color='orange', marker='o', linestyle='None', markersize=6, markerfacecolor='yellow', label=self.trajectories[0]['label']))

        for wps in self.waypoints:
            uv = project(wps)
            ax.scatter(uv[:, 0], uv[:, 1], color='red', marker='+', s=220, linewidths=2, zorder=6)
        if self.waypoints:
            legend_handles.append(mlines.Line2D([], [], color='red', marker='+', linestyle='None', markersize=12, markeredgewidth=2, label='wps'))

        cx, cy = cam.imageCenter
        ax.axhline(cy, color='royalblue', linewidth=1, alpha=0.8)
        ax.axvline(cx, color='royalblue', linewidth=1, alpha=0.8)

        ax.set_xlim(0, res[0])
        ax.set_ylim(res[1], 0)          
        ax.xaxis.tick_top()
        ax.xaxis.set_label_position('top')
        ax.set_xlabel('U', fontsize=12)
        ax.set_ylabel('V', fontsize=12)
        ax.set_title(title, fontsize=14, y=-0.06)

        ax.legend(handles=legend_handles, facecolor='white', edgecolor='gray',
                  loc='upper right', fontsize=11)

        fig.tight_layout()
        return fig, ax

    def plotDerivatives(self, trajectory_idx=0, times=None, figsize=(18, 9), bounds=None, 
                        matlab_data=None, matlab_times=None, waypoint_times=None):
        if trajectory_idx >= len(self.trajectories):
            raise IndexError(f"Trajectory index {trajectory_idx} out of range.")

        pts = self.trajectories[trajectory_idx]['points']   
        lbl = self.trajectories[trajectory_idx]['label']

        if times is None:
            times = np.linspace(0, 40, len(pts))
        times = np.asarray(times, dtype=float)

        vel  = np.gradient(pts,  times, axis=0)
        acc  = np.gradient(vel,  times, axis=0)
        jerk = np.gradient(acc,  times, axis=0)
        snap = np.gradient(jerk, times, axis=0)

        default_bounds = {'Position': 4.0, 'Velocity': 10.0, 'Acceleration': 1.0, 'Jerk': 0.5, 'Snap': 2.0}
        if bounds: default_bounds.update(bounds)

        axis_labels  = ['X', 'Y', 'Z']
        col_labels   = ['Position', 'Velocity', 'Acceleration', 'Jerk', 'Snap']
        data_list    = [pts, vel, acc, jerk, snap]

        # Extraer los waypoints directamente de la escena si existen
        wps_array = self.waypoints[0] if len(self.waypoints) > 0 else None

        fig, axes = plt.subplots(3, 5, figsize=figsize, facecolor='black')
        fig.patch.set_facecolor('black')

        for col, (col_lbl, data) in enumerate(zip(col_labels, data_list)):
            # Cálculo de límites
            col_min, col_max = np.min(data), np.max(data)
            if matlab_data is not None:
                col_min = min(col_min, np.min(matlab_data[col]))
                col_max = max(col_max, np.max(matlab_data[col]))
            if col_lbl == 'Position' and wps_array is not None:
                col_min = min(col_min, np.min(wps_array))
                col_max = max(col_max, np.max(wps_array))

            margen_col = (col_max - col_min) * 0.1 if col_max != col_min else 1.0
            
            for row, axis_lbl in enumerate(axis_labels):
                ax = axes[row, col]
                ax.set_facecolor('black')
                
                # 1. Curva Python (Naranja)
                ax.plot(times, data[:, row], color='orange', linewidth=1.5) 
                
                # 2. Curva MATLAB (Cian discontinua)
                if matlab_data is not None and matlab_times is not None:
                    ax.plot(matlab_times, matlab_data[col][:, row], color='cyan', linestyle='--', linewidth=1.5, alpha=0.7)

                # 3. WAYPOINTS (Puntos rojos)
                if col_lbl == 'Position' and wps_array is not None and waypoint_times is not None:
                    # Nos aseguramos de no intentar dibujar más puntos que tiempos dados
                    n_puntos = min(len(waypoint_times), wps_array.shape[0])
                    ax.plot(waypoint_times[:n_puntos], wps_array[:n_puntos, row], 'ro', markersize=6, zorder=5)

                # Límites y formato
                limit = default_bounds.get(col_lbl)
                if limit is not None:
                    ax.axhline(limit, color='limegreen', linestyle='-.', linewidth=1, alpha=0.5)
                    ax.axhline(-limit, color='limegreen', linestyle='-.', linewidth=1, alpha=0.5)

                ax.set_ylim([col_min - margen_col, col_max + margen_col])
                ax.set_ylabel(f'{col_lbl}-{axis_lbl}', color='white', fontsize=10)
                ax.tick_params(colors='white', labelsize=9)
                for spine in ax.spines.values(): spine.set_edgecolor('#444444')

        # Leyenda
        handles = [plt.Line2D([0], [0], color='orange', label='Python'),
                   plt.Line2D([0], [0], color='cyan', linestyle='--', label='MATLAB')]
        if wps_array is not None:
            handles.append(plt.Line2D([0], [0], color='red', marker='o', linestyle='', label='Waypoints'))
        
        axes[0, 4].legend(handles=handles, facecolor='black', edgecolor='#555555', labelcolor='white', loc='upper right')

        fig.tight_layout()
        return fig, axes


# ------------------------------------------------------------------ #
#  Quick demo
# ------------------------------------------------------------------ #
if __name__ == '__main__':
    from modules.camera import Camera

    cam = Camera(
        pose        = [1.5, 0.0, 0.5],
        orientation = [0, -15, 0],          
        fov         = [60, 45],
        focal       = [0.006, 0.006],
        pixelSize   = [4e-6, 4e-6],
        resolution  = [1456, 1088],
    )

    sq = np.array([
        [-1.5,  2, 1], [ 1.5,  2, 1], [ 1.5,  2, 2.5], [-1.5,  2, 2.5], [-1.5,  2, 1],
        [ 1.5,  2, 2.5], [-1.5, 2, 2.5], [ 1.5, 2, 1],    
        [-1.5, 2, 2.5],  
    ])

    t    = np.linspace(0, 2 * np.pi, 200)
    traj = np.column_stack([
        np.sin(t) * 1.5,
        np.ones(200) * 2.0,
        np.cos(t) * 0.75 + 1.75,
    ])

    wps = np.array([
        [-1.5, 2, 1], [1.5, 2, 1], [1.5, 2, 2.5], [-1.5, 2, 2.5]
    ])

    scene = Scene(bounds=[[-3, 3], [0, 6], [0, 5]])
    scene.addCamera(cam, label='{camera}')
    scene.addDesiredDrawing(sq)
    scene.addTrajectory(traj, label='Trajectory QP')
    scene.addWaypoints(wps)

    fig3d, _  = scene.plot3D(title='3D World')
    figCam, _ = scene.plotCameraView(camera_idx=0, title='Camera Frame View')
    figDer, _ = scene.plotDerivatives(trajectory_idx=0)

    plt.show()