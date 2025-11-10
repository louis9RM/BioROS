@"
# BioROS

Workspace ROS 2 Humble para leer ECG (AD8232 vía Arduino), publicar en ROS y visualizar en Foxglove Studio.

## Estructura
- ws/src: paquetes ROS 2 (código fuente)
- ws/build, ws/install, ws/log: generados por colcon (ignorados)

## Uso rápido
1. docker pull ros:humble-ros-base
2. docker run ... -v `"$PWD\ws:/root/ws`" ...
3. Dentro del contenedor: \`source /opt/ros/humble/setup.bash\` y \`colcon build\`

"@ | Out-File -Encoding utf8 README.md
