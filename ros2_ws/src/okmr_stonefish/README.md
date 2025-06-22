

step to run the simulator: 
1. make sure git submodules are initialized
2. compile stonefish inside the okmr_auv/simulation folder
3. build stonefish_ros2 using colcon like normal
4. I had to run the following to get the stonefish_ros2 package to find the stonefish install

'''bash
export LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH
''' 

I'd reccomend adding the export command to your ~/.bashrc or equivlant 

5. TBD
