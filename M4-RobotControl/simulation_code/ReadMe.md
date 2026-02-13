### Lab 1

## How to use docker
1. Open terminal navigate to the directory where the docker-compose.yml file is located:

```
cd M4-RobotControl/simulation_code
```

then run the following command to build and start the docker container:

```
docker compose up --build -d
```

Verify that the container is running by executing:

```
docker ps
```

2. To access the container's terminal, use the following command:

```
docker exec -it so101_container bash
```

3. Give permission to the X server to allow the container to access the display:

```
xhost +local:docker
```

4. Inside the container, set the DISPLAY environment variable:

```
export DISPLAY=:0
```

5. Now you can run the simulation code inside the container. For example, to run a Python script, use:

```
python3 run_mujoco_simulation.py &
```

