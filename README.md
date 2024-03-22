# Pick_Place
Paquete del ejercicio Pick_Place (Contiene PLugin Grasp) para desarrollo de DockerFile

# Clonar los repositorios desde GitHub en el directorio catkin_ws/src
RUN git clone https://github.com/adrielmicom/Pick_Place.git /catkin_ws/src/Pick_Place \
    && git clone https://github.com/JenniferBuehler/general-message-pkgs.git /catkin_ws/src/Pick_Place/general-message-pkgs \
    && git clone https://github.com/JenniferBuehler/gazebo-pkgs.git /catkin_ws/src/Pick_Place/gazebo-pkgs





# USO 
Permitir salida grafica en ubuntu, en una consola ->  xhost +

Descargar imagen seminario ->   docker pull adrielmicom/pick_place_seminario:tag

LANZAR CONTENEDOR ->
docker run -it --gpus all   -e DISPLAY=$DISPLAY   -v /tmp/.X11-unix:/tmp/.X11-unix   -v $HOME/.Xauthority:/root/.Xauthority   -e XAUTHORITY=/root/.Xauthority   --name seminario adrielmicom/pick_place_seminario:1

Abrir terminales de tu contenedor ->  docker exec -it seminario /bin/bash


roslaunch irb120_robotiq85_gazebo warehouse.launch
rosrun irb120_robotiq85_gazebo FK.py

Ruta codigos /catkin_ws/src/Pick_Place/irb120_robotiq85/irb120_robotiq85_gazebo/src
