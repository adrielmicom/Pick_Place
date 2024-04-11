# Pick_Place
Paquete del ejercicio Pick_Place (Contiene PLugin Grasp) para desarrollo de DockerFile.
Utilizar junto a devcontainer de VScode
# USO 
Permitir salida grafica en ubuntu, en una consola

	xhost +
 
# IMAGEN adrielmicom/pick_place_seminario:3

Descargar imagen seminario

	docker pull adrielmicom/pick_place_seminario:3

LANZAR CONTENEDOR

	docker run -it --gpus all -e DISPLAY=$DISPLAY   -v /tmp/.X11-unix:/tmp/.X11-unix   -v $HOME/.Xauthority:/root/.Xauthority   -e XAUTHORITY=/root/.Xauthority   --name seminario adrielmicom/pick_place_seminario:3
	
Abrir terminales de tu contenedor

	docker exec -it seminario /bin/bash


# Las aplicaciones se abren cuando se lo pides en su terminal.
abrira gazebo y Rviz

	roslaunch irb120_robotiq85_gazebo warehouse.launch	

SI TE APETECE LANZAR MI CODIGO

	rosrun irb120_robotiq85_gazebo spawn_model.py

	cd /catkin_ws/src/Pick_Place/irb120_robotiq85/irb120_robotiq85_gazebo/src

	python3 Pick3.py

lanzar un codigo demo

	rosrun irb120_robotiq85_gazebo FK.py

Ruta codigos
	/catkin_ws/src/Pick_Place/irb120_robotiq85/irb120_robotiq85_gazebo/src


#  Uso junto DevCOntainer
Utilizar junto a devContainer

Instalar complementos 

	code --install-extension oderwat.indent-rainbow ms-python.python PKief.material-icon-theme usernamehw.errorlens christian-kohler.path-intellisense ms-azuretools.vscode-docker donjayamanne.python-extension-pack ms-iot.vscode-ros





# EXPLICAICON parametros lanzamiento 


-it: Este parámetro indica a Docker que el contenedor se ejecutará en modo interactivo y con una terminal de pseudo-TTY asignada. Esto es útil para interactuar con el contenedor.

--gpus all: Este parámetro indica a Docker que debe asignar todos los recursos de GPU disponibles en el sistema al contenedor. Esto es útil si tu aplicación dentro del contenedor requiere aceleración de GPU.

-e DISPLAY=$DISPLAY: Este parámetro establece la variable de entorno DISPLAY dentro del contenedor para que coincida con la variable DISPLAY del host. Esto es necesario para que las aplicaciones dentro del contenedor puedan conectarse al servidor X del host y mostrar gráficos.

-v /tmp/.X11-unix:/tmp/.X11-unix: Este parámetro monta el socket de comunicación del servidor X del host en el mismo lugar dentro del contenedor. Esto permite que las aplicaciones dentro del contenedor se comuniquen con el servidor X del host para mostrar gráficos.

-v $HOME/.Xauthority:/root/.Xauthority: Este parámetro monta el archivo de autoridad del servidor X del host en el mismo lugar dentro del contenedor. Esto es necesario para que las aplicaciones dentro del contenedor puedan autenticarse correctamente con el servidor X del host.

-e XAUTHORITY=/root/.Xauthority: Este parámetro establece la variable de entorno XAUTHORITY dentro del contenedor para que coincida con el archivo de autoridad del servidor X del host. Esto es necesario para que las aplicaciones dentro del contenedor puedan autenticarse correctamente con el servidor X del host.



# NO NECESARIO

Creacion paquete pre lanzamiento docker 
Creacion de una carpeta docker_shared

	mkdir docker_shared
	cd docker_shared 
	catkin_create_pkg mipaquete
	
Dentro del contenedor

	cd workspace
	catkin init
	catkin_make
	source workspace/devel/setup.bash
	rosrun mipaquete Pick3.py
		

# Clonar los repositorios desde GitHub en el directorio catkin_ws/src
RUN git clone https://github.com/adrielmicom/Pick_Place.git /catkin_ws/src/Pick_Place \
    && git clone https://github.com/JenniferBuehler/general-message-pkgs.git /catkin_ws/src/Pick_Place/general-message-pkgs \
    && git clone https://github.com/JenniferBuehler/gazebo-pkgs.git /catkin_ws/src/Pick_Place/gazebo-pkgs
