# Docker manual

### Create an updated image (if Dockerfile have updates)

```bash
docker-compose build
```

### Create an instance of a container from image:
``` bash
docker run -it --privileged --ipc=host --net=host \
-v /tmp/.X11-unix:/tmp/.X11-unix:rw \
-v ~/.Xauthority:/home/sim/.Xauthority \
-v /home/vs/drone_hack/UDH2025_robotics:/home/sim/UDH2025_robotics:rw \
-e DISPLAY=$DISPLAY -p 14570:14570/udp --name=px4 udh2025_robotics-drone_sim:latest bash
```


### In docker 

``` bash
cd ~/PX4-Autopilot
make px4_sitl gz_x500
```
