## Useful commands

#### Run a particular command in container (outside it):
```
docker exec -it <CONTAINER_NAME> <CMD>
docker exec -it <CONTAINER_NAME> bash

# example 
docker exec -it px4 bash
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

# Remove px4
sudo docker rm px4
```

#### Images management commands
```
# Show all
sudo docker images

# Remove
sudo docker rmi udh2025_robotics-drone_sim
```