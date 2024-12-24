# Docker manual

### Create an updated image (if Dockerfile have updates)

```bash
docker-compose build
```

### Create an instance of a container from image:
```
docker run -it --privileged --ipc=host --net=host \
-v /tmp/.X11-unix:/tmp/.X11-unix:rw \
-v ~/.Xauthority:/home/sim/.Xauthority \
-v /home/vs/uav-flight-simulation-master:/home/sim/uav-flight-simulation-master:rw \
-e DISPLAY=$DISPLAY -p 14570:14570/udp --name=px4 udh2025_robotics-drone_sim:latest bash
```

### In docker 

```
cd ~/PX4-Autopilot
sudo make px4_sitl gz_x500
```

### Allow Connections from Docker
```
xhost +local:docker
```

## Useful commands

#### Run a particular command in container (outside it):
```
docker exec -it <CONTAINER_NAME> <CMD>
docker exec -it <CONTAINER_NAME> bash

# example 
docker exec -it px4 bash
```

#### Run to spawm a drone: (under preporation)
```

PX4_SYS_AUTOSTART=4001 PX4_GZ_MODEL_POSE="0,0" PX4_SIM_MODEL=gz_x500 ./build/px4_sitl_default/bin/px4 -i 0
PX4_SYS_AUTOSTART=4001 PX4_GZ_MODEL_POSE="0,1" PX4_SIM_MODEL=gz_x500 ./build/px4_sitl_default/bin/px4 -i 1
PX4_SYS_AUTOSTART=4001 PX4_GZ_MODEL_POSE="0,2" PX4_SIM_MODEL=gz_x500 ./build/px4_sitl_default/bin/px4 -i 7

```

# Working with containers

## _docker-compose_ additional commands
Under the project directory execute:

Show all docker-compose containers:
```
docker-compose ps -a
```

Show running docker-compose containers:
```
docker-compose ps
```

Remove docker-compose containers:
```
docker-compose rm
```

# Working with docker

#### Start docker containers:
```
sudo docker start <CONTAINER_NAME>
```

#### Stop a container
```
sudo docker kill <CONTAINER_NAME>
```

#### Container management commands
```
# Show all
sudo docker ps -a

# Show running
sudo docker ps

# Remove
sudo docker rm <CONTAINER_NAME>
```

#### Images management commands
```
# Show all
sudo docker images

# Remove
sudo docker rmi udh2025_robotics-drone_sim
```