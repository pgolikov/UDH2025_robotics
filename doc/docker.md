# Docker manual

### Create an updated image (if Dockerfile have updates)

```bash
docker-compose build
```

### Allow docker xhost for gui interfaces
``` bash
xhost +local:docker
```

### Create an instance of a container from image:
``` bash
docker run -it --privileged --ipc=host --net=host \
-v /tmp/.X11-unix:/tmp/.X11-unix:rw \
-v ~/.Xauthority:/home/sim/.Xauthority \
-v ./:/home/sim/UDH2025_robotics:rw \
-e DISPLAY=$DISPLAY -p 14570:14570/udp --name=px4 udh2025_robotics-drone_sim:latest bash
```

### In docker, build px4 after instaltaion of doker.
``` bash
cd ~/PX4-Autopilot
sudo make px4_sitl gz_x500
```

#### To enter the docker use:
``` bash
docker exec -it px4 bash
```