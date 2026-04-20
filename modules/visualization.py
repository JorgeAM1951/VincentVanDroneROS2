import numpy as np
import matplotlib.pyplot as plt
import matplotlib.lines as mlines
import matplotlib.patches as mpatches
from mpl_toolkits.mplot3d import Axes3D
from scipy.spatial.transform import Rotation as R


class Scene:
    def __init__(self, bounds=None):
        """
        3D visualization scene, mimicking MATLAB-style camera/trajectory plots.

        Parameters
        ----------
        bounds : list[list[float]] | None
            Bounding box [[xmin, xmax], [ymin, ymax], [zmin, zmax]].
            If provided, a dashed green box is drawn in the 3D world view.
        """
        self.cameras        = []    # List of Camera objects
        self.cameraLabels   = []    # Label per camera, e.g. '{camera}'
        self.waypoints      = []    # List of Nx3 np.arrays
        self.desiredDrawings = []   # List of Nx3 np.arrays  (green paths)
        self.trajectories   = []    # List of dicts {'points': Nx3, 'label': str}
        self.bounds         = bounds

    # ------------------------------------------------------------------ #
    #  Scene population
    # ------------------------------------------------------------------ #

    def addCamera(self, camera, label='{camera}'):
        """Add a Camera (from cameraConfigs.py) to the scene."""
        self.cameras.append(camera)
        self.cameraLabels.append(label)

    def addWaypoints(self, waypoints):
        """
        Add waypoints shown as red '+' markers.

        Parameters
        ----------
        waypoints : array-like, shape (N, 3)
        """
        self.waypoints.append(np.asarray(waypoints, dtype=float))

    def addDesiredDrawing(self, path):
        """
        Add a desired drawing path (green solid line).

        Parameters
        ----------
        path : array-like, shape (N, 3)
        """
        self.desiredDrawings.append(np.asarray(path, dtype=float))

    def addTrajectory(self, points, label='Trajectory QP'):
        """
        Add an optimised trajectory (orange line / dots).

        Parameters
        ----------
        points : array-like, shape (N, 3)
        label  : str
        """
        self.trajectories.append({
            'points': np.asarray(points, dtype=float),
            'label' : label
        })

    # ------------------------------------------------------------------ #
    #  Private helpers
    # ------------------------------------------------------------------ #

    @staticmethod
    def _buildRotationMatrix(orientation):
        """
        Build world-to-camera rotation from [roll, pitch, yaw] (degrees).
        Matches the convention used in Camera.getP().
        """
        RX = R.from_euler('x', orientation[1], degrees=True).as_matrix()   # pitch
        RY = R.from_euler('y', orientation[0], degrees=True).as_matrix()   # roll
        RZ = R.from_euler('z', orientation[2], degrees=True).as_matrix()   # yaw
        return RZ @ RY @ RX

    def _getCameraFrustumVertices(self, camera, scale=0.5):
        """
        Returns (center, corners) – the apex and the four image-plane
        corners of the camera frustum in world coordinates.
        """
        center = np.asarray(camera.pose, dtype=float)
        RWC    = self._buildRotationMatrix(camera.orientation)

        hx = np.tan(np.radians(camera.fovX / 2.0)) * scale
        hy = np.tan(np.radians(camera.fovY / 2.0)) * scale

        # Four corners in camera space (looking along +Z)
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
        """Draw pyramid lines + image-plane rectangle on a 3D axes."""
        for corner in corners:
            ax.plot([center[0], corner[0]],
                    [center[1], corner[1]],
                    [center[2], corner[2]],
                    color=color, linewidth=1.5)

        # Close the image-plane rectangle
        rect = np.vstack([corners, corners[0]])
        ax.plot(rect[:, 0], rect[:, 1], rect[:, 2],
                color=color, linewidth=1.5)

        if label:
            ax.text(center[0] + 0.05, center[1] + 0.05, center[2] - 0.15,
                    label, color=color, fontsize=8)

    @staticmethod
    def _drawAxesFrame(ax, origin, rotation_matrix, scale=0.4, label=None):
        """Draw an RGB XYZ coordinate frame at a given origin."""
        colors = ['red', 'green', 'blue']
        for i, color in enumerate(colors):
            direction = rotation_matrix[:, i] * scale
            ax.quiver(origin[0], origin[1], origin[2],
                      direction[0], direction[1], direction[2],
                      color=color, linewidth=1.5, arrow_length_ratio=0.3)
        if label:
            ax.text(origin[0], origin[1], origin[2] - 0.2,
                    label, color='white', fontsize=7)

    @staticmethod
    def _drawBoundingBox(ax, bounds, color='limegreen', linestyle='--'):
        """Draw the 12 dashed edges of an axis-aligned bounding box."""
        (x0, x1), (y0, y1), (z0, z1) = bounds

        edges = [
            # bottom face
            ([x0,x1],[y0,y0],[z0,z0]), ([x1,x1],[y0,y1],[z0,z0]),
            ([x1,x0],[y1,y1],[z0,z0]), ([x0,x0],[y1,y0],[z0,z0]),
            # top face
            ([x0,x1],[y0,y0],[z1,z1]), ([x1,x1],[y0,y1],[z1,z1]),
            ([x1,x0],[y1,y1],[z1,z1]), ([x0,x0],[y1,y0],[z1,z1]),
            # vertical edges
            ([x0,x0],[y0,y0],[z0,z1]), ([x1,x1],[y0,y0],[z0,z1]),
            ([x1,x1],[y1,y1],[z0,z1]), ([x0,x0],[y1,y1],[z0,z1]),
        ]
        for ex, ey, ez in edges:
            ax.plot(ex, ey, ez, color=color, linestyle=linestyle, linewidth=1.0)

    @staticmethod
    def _applyDarkStyle(ax):
        """Apply the black-background MATLAB-like style to a 3D axes."""
        ax.set_facecolor('black')
        for pane in (ax.xaxis.pane, ax.yaxis.pane, ax.zaxis.pane):
            pane.fill = False
            pane.set_edgecolor('#444444')
        ax.tick_params(colors='white', labelsize=8)
        ax.xaxis.label.set_color('white')
        ax.yaxis.label.set_color('white')
        ax.zaxis.label.set_color('white')
        ax.title.set_color('white')
        ax.grid(True, color='#333333', linewidth=0.5)

    # ------------------------------------------------------------------ #
    #  Public plot methods
    # ------------------------------------------------------------------ #

    def plot3D(self, figsize=(10, 8), title='3D World', cameraScale=0.4):
        """
        Render the 3D world scene.

        Returns
        -------
        fig, ax : matplotlib Figure and Axes3D
        """
        fig = plt.figure(figsize=figsize, facecolor='black')
        ax  = fig.add_subplot(111, projection='3d')
        self._applyDarkStyle(ax)

        legend_handles = []

        # ---- Bounding box ----
        if self.bounds is not None:
            self._drawBoundingBox(ax, self.bounds)

        # ---- Desired drawings (green solid) ----
        for i, drawing in enumerate(self.desiredDrawings):
            ax.plot(drawing[:, 0], drawing[:, 1], drawing[:, 2],
                    color='limegreen', linewidth=1.5)
        if self.desiredDrawings:
            legend_handles.append(
                mlines.Line2D([], [], color='limegreen', linewidth=1.5,
                              label='desired drawing'))

        # ---- Trajectories (orange) ----
        for i, traj in enumerate(self.trajectories):
            pts = traj['points']
            ax.plot(pts[:, 0], pts[:, 1], pts[:, 2],
                    color='orange', linewidth=1.5)
        if self.trajectories:
            legend_handles.append(
                mlines.Line2D([], [], color='orange', linewidth=1.5,
                              label=self.trajectories[0]['label']))

        # ---- Waypoints (red +) ----
        for wps in self.waypoints:
            ax.scatter(wps[:, 0], wps[:, 1], wps[:, 2],
                       color='red', marker='+', s=120, linewidths=2, depthshade=False)
        if self.waypoints:
            legend_handles.append(
                mlines.Line2D([], [], color='red', marker='+',
                              linestyle='None', markersize=10,
                              markeredgewidth=2, label='Wps'))

        # ---- Cameras (red frustum + frame axes) ----
        for cam, lbl in zip(self.cameras, self.cameraLabels):
            center, corners = self._getCameraFrustumVertices(cam, scale=cameraScale)
            self._drawCameraFrustum(ax, center, corners, color='red', label=lbl)
            RWC = self._buildRotationMatrix(cam.orientation)
            self._drawAxesFrame(ax, center, RWC, scale=cameraScale * 0.7)

        # ---- World origin frame ----
        self._drawAxesFrame(ax, [0, 0, 0], np.eye(3), scale=0.5, label='{world}')

        ax.set_xlabel('X [m]')
        ax.set_ylabel('Y [m]')
        ax.set_zlabel('Z [m]')
        ax.set_title(title, fontsize=14)

        ax.legend(handles=legend_handles,
                  facecolor='#111111', edgecolor='#555555',
                  labelcolor='white', loc='upper right', fontsize=9)

        fig.tight_layout()
        return fig, ax

    def plotCameraView(self, camera_idx=0, figsize=(7, 6),
                       title='Camera Frame View'):
        """
        Render the 2D camera-projection view (image-space).

        Parameters
        ----------
        camera_idx : int
            Index of the camera to use for projection.

        Returns
        -------
        fig, ax : matplotlib Figure and Axes
        """
        if camera_idx >= len(self.cameras):
            raise IndexError(f"Camera index {camera_idx} is out of range "
                             f"(scene has {len(self.cameras)} camera(s)).")

        cam = self.cameras[camera_idx]
        P   = cam.getP()
        res = cam.resolution

        fig, ax = plt.subplots(figsize=figsize)
        ax.set_facecolor('#909090')
        fig.patch.set_facecolor('#909090')

        legend_handles = []

        def project(pts_3d):
            """Project Nx3 world points to Nx2 pixel coordinates via P."""
            pts_h = np.hstack([pts_3d, np.ones((len(pts_3d), 1))])  # Nx4
            proj  = (P @ pts_h.T).T                                  # Nx3
            proj /= proj[:, 2:3]                                     # normalise
            return proj[:, :2]

        # ---- Desired drawings (green) ----
        for i, drawing in enumerate(self.desiredDrawings):
            uv = project(drawing)
            ax.plot(uv[:, 0], uv[:, 1], color='limegreen', linewidth=1.5)
        if self.desiredDrawings:
            legend_handles.append(
                mlines.Line2D([], [], color='limegreen', linewidth=1.5,
                              label='desired drawing'))

        # ---- Trajectories (yellow-orange dots) ----
        for i, traj in enumerate(self.trajectories):
            uv = project(traj['points'])
            ax.scatter(uv[:, 0], uv[:, 1],
                       color='yellow', edgecolors='orange',
                       s=18, linewidths=0.5, zorder=5)
        if self.trajectories:
            legend_handles.append(
                mlines.Line2D([], [], color='orange', marker='o',
                              linestyle='None', markersize=6,
                              markerfacecolor='yellow',
                              label=self.trajectories[0]['label']))

        # ---- Waypoints (red +) ----
        for wps in self.waypoints:
            uv = project(wps)
            ax.scatter(uv[:, 0], uv[:, 1],
                       color='red', marker='+', s=220,
                       linewidths=2, zorder=6)
        if self.waypoints:
            legend_handles.append(
                mlines.Line2D([], [], color='red', marker='+',
                              linestyle='None', markersize=12,
                              markeredgewidth=2, label='wps'))

        # ---- Image-centre crosshair (blue) ----
        cx, cy = cam.imageCenter
        ax.axhline(cy, color='royalblue', linewidth=1, alpha=0.8)
        ax.axvline(cx, color='royalblue', linewidth=1, alpha=0.8)

        # ---- Axis / legend formatting ----
        ax.set_xlim(0, res[0])
        ax.set_ylim(res[1], 0)          # flip Y – image convention
        ax.xaxis.tick_top()
        ax.xaxis.set_label_position('top')
        ax.set_xlabel('U', fontsize=10)
        ax.set_ylabel('V', fontsize=10)
        ax.set_title(title, fontsize=11, y=-0.06)

        ax.legend(handles=legend_handles,
                  facecolor='white', edgecolor='gray',
                  loc='upper right', fontsize=9)

        fig.tight_layout()
        return fig, ax

    def plotDerivatives(self, trajectory_idx=0, times=None, figsize=(18, 9),
                        bounds=None):
        """
        Plot position, velocity, acceleration, jerk, and snap for X, Y, Z –
        matching the right-hand panel in the reference image.

        Parameters
        ----------
        trajectory_idx : int
            Which trajectory to differentiate.
        times : array-like | None
            Time vector of length N. Defaults to linspace(0, 40, N).
        bounds : dict | None
            Optional green dashed limits per derivative, e.g.
            {'Position': 4, 'Velocity': 10, 'Acceleration': 1}.

        Returns
        -------
        fig, axes : matplotlib Figure and 3×5 axes array
        """
        if trajectory_idx >= len(self.trajectories):
            raise IndexError(f"Trajectory index {trajectory_idx} out of range.")

        pts = self.trajectories[trajectory_idx]['points']   # Nx3
        lbl = self.trajectories[trajectory_idx]['label']

        if times is None:
            times = np.linspace(0, 40, len(pts))
        times = np.asarray(times, dtype=float)

        # Numerical derivatives
        vel  = np.gradient(pts,  times, axis=0)
        acc  = np.gradient(vel,  times, axis=0)
        jerk = np.gradient(acc,  times, axis=0)
        snap = np.gradient(jerk, times, axis=0)

        default_bounds = {
            'Position':     4.0,
            'Velocity':     10.0,
            'Acceleration': 1.0,
            'Jerk':         0.5,
            'Snap':         2.0,
        }
        if bounds:
            default_bounds.update(bounds)

        axis_labels  = ['X', 'Y', 'Z']
        col_labels   = ['Position', 'Velocity', 'Acceleration', 'Jerk', 'Snap']
        data_list    = [pts, vel, acc, jerk, snap]

        fig, axes = plt.subplots(3, 5, figsize=figsize, facecolor='black')
        fig.patch.set_facecolor('black')

        for row, axis_lbl in enumerate(axis_labels):
            for col, (col_lbl, data) in enumerate(zip(col_labels, data_list)):
                ax = axes[row, col]
                ax.set_facecolor('black')

                ax.plot(times, data[:, row], color='orange', linewidth=1.0)

                # Green dashed limit lines
                limit = default_bounds.get(col_lbl)
                if limit is not None:
                    ax.axhline( limit, color='limegreen', linestyle='-.', linewidth=1)
                    ax.axhline(-limit, color='limegreen', linestyle='-.', linewidth=1)

                ax.set_xlabel('t', color='white', fontsize=8)
                ax.set_ylabel(f'{col_lbl}-{axis_lbl}', color='white', fontsize=8)
                ax.tick_params(colors='white', labelsize=7)
                for spine in ax.spines.values():
                    spine.set_edgecolor('#444444')
                ax.grid(False)

        # Legend on the last cell
        axes[2, 4].plot([], [], color='orange', linewidth=1.5, label=lbl)
        axes[2, 4].legend(facecolor='black', edgecolor='#555555',
                          labelcolor='white', loc='upper right', fontsize=8)

        fig.tight_layout(pad=0.5)
        return fig, axes


# ------------------------------------------------------------------ #
#  Quick demo – remove or guard with __main__ in production
# ------------------------------------------------------------------ #
if __name__ == '__main__':
    # Lazy import to avoid circular dependency
    from camera import Camera

    # --- Build a simple demo camera ---
    cam = Camera(
        pose        = [1.5, 0.0, 0.5],
        orientation = [0, -15, 0],          # roll, pitch, yaw (deg)
        fov         = [60, 45],
        focal       = [0.006, 0.006],
        pixelSize   = [4e-6, 4e-6],
        resolution  = [1456, 1088],
    )

    # --- Build a square + diagonals as the desired drawing ---
    sq = np.array([
        [-1.5,  2, 1], [ 1.5,  2, 1], [ 1.5,  2, 2.5], [-1.5,  2, 2.5], [-1.5,  2, 1],
        [ 1.5,  2, 2.5], [-1.5, 2, 2.5], [ 1.5, 2, 1],    # diagonals
        [-1.5, 2, 2.5],  # back to top-left
    ])

    # --- Simple sinusoidal trajectory ---
    t    = np.linspace(0, 2 * np.pi, 200)
    traj = np.column_stack([
        np.sin(t) * 1.5,
        np.ones(200) * 2.0,
        np.cos(t) * 0.75 + 1.75,
    ])

    # --- Waypoints at the square corners ---
    wps = np.array([
        [-1.5, 2, 1], [1.5, 2, 1], [1.5, 2, 2.5], [-1.5, 2, 2.5]
    ])

    # --- Assemble scene ---
    scene = Scene(bounds=[[-3, 3], [0, 6], [0, 5]])
    scene.addCamera(cam, label='{camera}')
    scene.addDesiredDrawing(sq)
    scene.addTrajectory(traj, label='Trajectory QP')
    scene.addWaypoints(wps)

    # --- Render ---
    fig3d, _  = scene.plot3D(title='3D World')
    figCam, _ = scene.plotCameraView(camera_idx=0, title='Camera Frame View')
    figDer, _ = scene.plotDerivatives(trajectory_idx=0)

    plt.show()