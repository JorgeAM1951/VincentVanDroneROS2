Aquí tienes el archivo completo listo para que lo copies y lo guardes directamente como `README.md` en la raíz de tu repositorio:

```markdown
# VincentVanDrone ROS 2

Este repositorio contiene las herramientas de generación de trayectorias (Minimum Snap) y la ejecución del nodo de ROS 2 (Jazzy) mediante MAVROS para dibujar figuras geométricas complejas (como hexágonos proyectados) usando un dron.

Para garantizar que todas las librerías matemáticas (Clarabel, OSQP, Numpy, Scipy) y las dependencias de ROS 2 funcionen a la primera, el proyecto está completamente dockerizado.

---

## Requisitos Previos

Antes de empezar, asegúrate de tener instalado en tu sistema:
*   **Git** (para clonar el repositorio).
*   **Docker** (para construir y ejecutar el contenedor).

---

## Guía de Instalación y Uso para Linux (Ubuntu/Debian)

Linux es el entorno nativo para este proyecto y la ejecución es muy directa.

### Paso 1: Clonar el repositorio
Abre una terminal y descarga el código en tu máquina:
```bash
git clone <URL_DE_TU_REPOSITORIO> vincent_van_drone
cd vincent_van_drone

```

### Paso 2: Dar permisos a la interfaz gráfica (X11)

Para que los scripts de Python puedan abrir las ventanas 3D de Matplotlib en tu pantalla, ejecuta este comando en tu ordenador anfitrión:

```bash
xhost +local:root

```

### Paso 3: Construir la imagen Docker

Asegúrate de estar en el directorio donde se encuentra el `Dockerfile` y ejecuta:

```bash
docker build -t vincent_van_drone:jazzy .

```

*(Nota: Este paso puede tardar unos minutos la primera vez, ya que descarga ROS 2 Jazzy y compila las dependencias).*

### Paso 4: Lanzar el contenedor

Arranca el contenedor montando el código de tu ordenador directamente en el workspace de ROS 2 y conectando la pantalla:

```bash
docker run -it --rm \
  --net=host \
  -e DISPLAY=$DISPLAY \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -v $(pwd):/root/drones_unizar_ws/src \
  vincent_van_drone:jazzy

```

---

## Guía de Instalación y Uso para Windows 10/11

Para usar aplicaciones con interfaz gráfica de Linux (GUI) y ROS 2 en Windows, utilizaremos **WSL2** (Windows Subsystem for Linux) y un servidor X11.

### Paso 1: Preparar el entorno (Solo la primera vez)

1. Instala [Docker Desktop para Windows](https://docs.docker.com/desktop/install/windows-install/) y asegúrate de que la integración con WSL2 esté activada en los ajustes (`Settings > General > Use the WSL 2 based engine`).
2. Instala un servidor X11 como [VcXsrv (Xming)](https://sourceforge.net/projects/vcxsrv/).
3. Ejecuta el acceso directo **XLaunch** que se ha creado al instalar VcXsrv.
4. En la configuración inicial, deja todo por defecto dando a *Next*, pero en la ventana **Extra settings**, asegúrate de marcar la casilla **"Disable access control"**. Haz clic en *Finish*.

### Paso 2: Clonar el repositorio

Abre una terminal de **PowerShell** o **WSL** y ejecuta:

```bash
git clone <URL_DE_TU_REPOSITORIO> vincent_van_drone
cd vincent_van_drone

```

### Paso 3: Construir la imagen Docker

Dentro de la misma terminal, en la carpeta del proyecto, ejecuta:

```bash
docker build -t vincent_van_drone:jazzy .

```

### Paso 4: Lanzar el contenedor (con gráficos)

En Windows, necesitamos decirle al contenedor que envíe la imagen gráfica a la IP del host de Windows a través de Docker. Ejecuta:

```powershell
docker run -it --rm `
  -e DISPLAY=host.docker.internal:0.0 `
  -v ${PWD}:/root/drones_unizar_ws/src `
  vincent_van_drone:jazzy

```

*(Si usas Git Bash o WSL en lugar de PowerShell, cambia `${PWD}` por `$(pwd)`).*

---

## Uso y Comprobación (Dentro del Docker)

Una vez hayas ejecutado el paso 4 en tu respectivo sistema operativo, tu terminal cambiará y serás el usuario `root` dentro de `/root/drones_unizar_ws`. Todo el entorno de ROS 2 se cargará automáticamente.

### 1. Comprobación rápida (Smoke Test)

Verifica que las librerías matemáticas se han instalado correctamente:

```bash
python3 -c "import numpy, scipy, matplotlib, osqp, clarabel; print('\n>>> Entorno Python OK <<<')"

```

### 2. Generar la trayectoria 3D

Navega a la carpeta de tu código base y ejecuta el script principal:

```bash
cd src
python3 main.py

```

**¿Qué debería ocurrir?**

1. El solver calculará los coeficientes matemáticos y los imprimirá en la terminal.
2. Se abrirán ventanas gráficas mostrando la trayectoria del dron (Hexágono) y las gráficas de velocidad, aceleración, jerk y snap.
3. Se generará una carpeta `trayectorias_exportadas/` (que también aparecerá en tu ordenador host) con los archivos de texto listos para ser leídos por MAVROS.

### 3. Lanzar el nodo de ROS 2

*(Asegúrate de tener un simulador SITL como PX4 corriendo y accesible).*
Ejecuta tu nodo para enviar la trayectoria generada al dron:

```bash
cd src/ROS2
python3 main_node.py

```

```

```