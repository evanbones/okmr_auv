# okmr_stonefish
step to run the simulator: 
1. make sure git submodules are initialized
2. compile stonefish inside the 3rd party folder
3. build stonefish_ros2 using colcon like normal
4. I had to run the following to get the stonefish_ros2 package to find the stonefish install

```bash
export LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH
```

I'd reccomend adding the export command to your ~/.bashrc or equivlant 

5. build okmr_stonefish using colcon build
6. launch the simulation using the desired .scn (scenario) file

```bash
ros2 launch okmr_stonefish sim.launch.py scenario_name:=simple.scn 
```
