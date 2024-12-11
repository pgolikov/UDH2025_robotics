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
    -e DISPLAY=$DISPLAY -p 14570:14570/udp --name=px4 uav-flight-simulation-master-drone_sim:latest bash
```

### In docker 

```
cd ~/PX4-Autopilot
make px4_sitl gz_x500
```