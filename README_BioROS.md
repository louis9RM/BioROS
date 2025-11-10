# BioROS Setup Guide

Este documento detalla **paso a paso** la configuración completa del entorno Docker con ROS 2 Humble en Windows, conexión al Arduino (con sensor AD8232), y preparación para desarrollo y visualización con Foxglove Studio.

---

## 🧩 1. Estructura del proyecto

```
C:\Users\EVER\BioROS
│
└─── ws/
     └─── src/
```

`ws/src` es el workspace donde se almacenan los paquetes ROS 2.  
Este directorio se comparte con el contenedor Docker para que los cambios sean persistentes.

---

## 🐳 2. Instalación y configuración de Docker Desktop

1. Instalar **Docker Desktop for Windows**.  
2. Asegurarse de que esté usando **WSL2** como backend.  
3. Confirmar que funciona ejecutando:
   ```powershell
   docker --version
   ```

---

## ⚙️ 3. Descargar imagen base de ROS 2 Humble

```powershell
docker pull ros:humble-ros-base
```

---

## 🌐 4. Crear red Docker dedicada

```powershell
docker network create rosnet
```

Esto permite comunicación estable entre contenedores (por ejemplo, con Foxglove Studio).

---

## 🐋 5. Crear y ejecutar el contenedor inicial

```powershell
docker run -it --name ros2bio `
  --hostname ros2bio `
  --network rosnet `
  -v "$PWD\ws:/root/ws" `
  -p 8765:8765 `
  -e TZ=America/Lima `
  ros:humble-ros-base bash
```

Una vez dentro del contenedor (`root@ros2bio:/#`), instalar herramientas básicas:

```bash
apt update && apt install -y   python3-pip python3-colcon-common-extensions git curl   nano less iproute2 net-tools
```

Configurar entorno ROS 2 permanente:

```bash
echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc
source /opt/ros/humble/setup.bash
```

Verificar:
```bash
ros2 topic list
```

---

## 🔌 6. Conectar el Arduino (AD8232)

### a) Conectar Arduino y listar dispositivos USB

En PowerShell (normal):
```powershell
usbipd list
```
Ejemplo de salida:
```
BUSID  VID:PID    DEVICE
2-3    2341:8036  Arduino Leonardo (COM4)
```

### b) Conceder acceso a WSL (usar PowerShell como Administrador)

```powershell
usbipd bind --busid 2-3
usbipd attach --wsl --busid 2-3
```

Verificar que aparece como `Attached`:
```powershell
usbipd list
```

### c) Comprobar que Linux ve el dispositivo

```powershell
docker run --rm -it --privileged ubuntu:22.04 bash
```
Dentro:
```bash
apt update && apt install -y usbutils
ls /dev/ttyACM* /dev/ttyUSB* 2>/dev/null || echo "No aparece"
```
Salida esperada:
```
/dev/ttyACM0
```

Salir con `exit`.

---

## 🧠 7. Relanzar el contenedor ROS 2 con Arduino conectado

Primero eliminar el anterior contenedor (solo el contenedor, no los datos):

```powershell
docker rm -f ros2bio
```

Luego relanzar con el dispositivo mapeado:

```powershell
docker run -it --name ros2bio `
  --hostname ros2bio `
  --network rosnet `
  --device=/dev/ttyACM0 `
  -v "$PWD\ws:/root/ws" `
  -p 8765:8765 `
  -e TZ=America/Lima `
  ros:humble-ros-base bash
```

Verificar dentro del contenedor:
```bash
ls -l /dev/ttyACM0
```
Salida esperada:
```
crw------- 1 root root 166, 0 Nov 10 18:53 /dev/ttyACM0
```

---

## ✅ Estado actual

| Elemento | Estado |
|-----------|--------|
| Docker Desktop | ✔️ |
| ROS 2 Humble | ✔️ |
| Red rosnet | ✔️ |
| Arduino Leonardo conectado | ✔️ |
| Puerto /dev/ttyACM0 visible | ✔️ |
| Workspace ws/src persistente | ✔️ |

---

## 🚀 Próximo paso

**Paso 3:** Crear un nodo ROS 2 (Python) que lea los datos del AD8232 vía `/dev/ttyACM0` y publique en un tópico `/ecg_raw`.  
Luego se visualizará en **Foxglove Studio (web)** a través del puerto `8765` usando `rosbridge` o `foxglove_bridge`.

---

Autor: EVER  
Fecha: 10-Nov-2025  
Versión: 1.0
