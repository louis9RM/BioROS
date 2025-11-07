# BioROS

**Sistema de adquisición y procesamiento de señales electrocardiográficas en entorno Docker utilizando ROS2.**

---

## 📖 Descripción general
BioROS es un proyecto académico que integra hardware biomédico (sensor AD8232 + Arduino) con un entorno de software basado en **ROS2** y **Docker**.  
Su objetivo es permitir la adquisición, procesamiento y visualización de señales ECG de manera modular y portátil.

---

## ⚙️ Arquitectura del sistema
**Componentes principales:**
- **Sensor AD8232:** Captura la señal electrocardiográfica analógica.  
- **Arduino:** Convierte la señal analógica en digital y la envía por puerto serial.  
- **Docker + ROS2:** Contenedor que aloja los nodos ROS para adquisición, filtrado y análisis.  
- **PC (Windows):** Ejecuta el entorno Docker y visualiza los resultados.

---

## 🧩 Estructura del repositorio
```
BioROS/
├── docker/
│   ├── Dockerfile
│   └── ros_entrypoint.sh
├── src/
│   ├── ecg_serial/
│   ├── ecg_proc/
│   ├── ecg_viz/
│   └── ecg_logger/
├── launch/
│   └── bio_ros.launch.py
├── docs/
│   └── diagramas/
└── README.md
```

---

## 🚀 Instalación y uso

### 1. Clonar el repositorio
```bash
git clone https://github.com/louis9RM/BioROS.git
cd BioROS
```

### 2. Construir el contenedor Docker
```bash
docker build -t bioros:latest ./docker
```

### 3. Ejecutar el contenedor
```bash
docker run -it --rm --device=/dev/ttyUSB0 bioros:latest
```

### 4. Visualizar los tópicos ROS2
```bash
ros2 topic list
```

---

## 🧠 Nodos principales
| Nodo | Función |
|------|----------|
| `ecg_serial` | Lee datos desde Arduino y publica `/ecg/raw` |
| `ecg_proc` | Filtra la señal y calcula HR y HRV |
| `ecg_viz` | Muestra datos en tiempo real |
| `ecg_logger` | Guarda registros en rosbag o CSV |

---

## 🧪 Hardware utilizado
- Arduino Uno / Nano  
- Sensor AD8232 (ECG)  
- Electrodos y cables ECG  
- PC con Windows / Linux + Docker

---

## 🧑‍💻 Autor
**Luis R.M.**  
Proyecto académico para la universidad — 2025.

---

## 🏷️ Licencia
Este proyecto se distribuye bajo la licencia MIT.
