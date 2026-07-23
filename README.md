
```markdown
``
# VincentVanDrone ROS 2

This repo contains the tools for trajectory generation (Minimum Snap) and the ROS 2 (Jazzy) node execution via MAVROS. This is a port of the original VincentVanDrone project. 

To make sure all the math libraries (Clarabel, OSQP, Numpy, Scipy) and ROS 2 dependencies work smoothly, we provide a Docker container.

---

## Prerequisites

Before you start, make sure you have the following installed and updated:
*   **Docker** (to build and run the container).
---

## Linux Guide (Ubuntu/Debian)

Linux is the native OS for this project, so you should expect this to work.

### Step 1: Clone the repo
Open a terminal and download the code to your machine:
```bash
git clone https://github.com/JorgeAM1951/VincentVanDroneROS2 vincent_van_drone
cd vincent_van_drone

```

### Step 2: Allow GUI forwarding (X11)

To let gazebo and Rviz2 create windows in your computer, run this command on your host machine:

```bash
xhost +local:root

```

### Step 3: Build the Docker image

Make sure you're in the same directory as the `Dockerfile` and run:

```bash
docker build -t vincent_van_drone:jazzy .

```

*(Note: This might take a few minutes the first time, as it has to download ROS 2 Jazzy and compile dependencies).*

### Step 4: Run the container

Spin up the container by mounting your local code directly into the ROS 2 workspace and connecting your display:

```bash
docker run -it --rm \
  --net=host \
  -e DISPLAY=$DISPLAY \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -v $(pwd):/root/drones_unizar_ws/src \
  vincent_van_drone:jazzy

```

---

## Windows 10/11 Guide

To run Linux GUI apps and ROS 2 on Windows, we will be using **WSL2** along with an X11 server.

### Step 1: Environment setup (First time only)

1. Install [Docker Desktop for Windows](https://docs.docker.com/desktop/install/windows-install/) and make sure WSL2 integration is turned on in the settings (`Settings > General > Use the WSL 2 based engine`).
2. Install an X11 server like [VcXsrv (Xming)](https://sourceforge.net/projects/vcxsrv/).
3. Run the **XLaunch** shortcut that was created when you installed VcXsrv.
4. Leave the default settings as they are and keep clicking *Next*. However, when you get to the **Extra settings** window, make sure to check **"Disable access control"**. Then click *Finish*.

### Step 2: Clone the repo

Open a **PowerShell** or **WSL** terminal and run:

```bash
git clone <YOUR_REPO_URL> vincent_van_drone
cd vincent_van_drone

```

### Step 3: Build the Docker image

In that same terminal, inside the project folder, run:

```bash
docker build -t vincent_van_drone:jazzy .

```

### Step 4: Run the container

On Windows, we need to tell the container to send the graphics to the Windows host IP through Docker. Run:

```powershell
docker run -it --rm `
  -e DISPLAY=host.docker.internal:0.0 `
  -v ${PWD}:/root/drones_unizar_ws/src `
  vincent_van_drone:jazzy

```

*(If you're using Git Bash or WSL instead of PowerShell, swap `${PWD}` for `$(pwd)`).*

---

## Usage and Testing 

Once you've run Step 4 on your respective OS, your terminal will change and you will be logged in as the `root` user inside `/root/drones_unizar_ws`. The entire ROS 2 environment will load automatically.

### 1. Quick build test

Check that the math libraries are installed properly:

```bash
python3 -c "import numpy, scipy, matplotlib, osqp, clarabel; print('\n>>> Python Environment OK <<<')"

```

### 2. Generate the 3D trajectory

Navigate to the codebase folder and run the main script:

```bash
cd src
python3 main.py

```

**What should happen?**

1. The solver will calculate the math coefficients and print them in the terminal.
2. Graphic windows will pop up showing the drone's path along with velocity, acceleration, jerk, and snap plots.
3. A `trayectorias_exportadas/` folder (which will also show up on your host machine) will be generated, containing the text files ready for MAVROS to read.

### 3. Launch the ROS 2 node

*(Make sure you have a SITL simulator like PX4 running and accessible).*
Run your node to send the generated trajectory to the drone:

```bash
cd src/ROS2
python3 main_node.py

```
