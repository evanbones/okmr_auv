# 08-11-25 controls test
1. launch corresponding launch file
```ros2 launch ogopogo pool_tests/08-11-25-controls.launch.py```
2. ensure all nodes are launched
    - dead reckoning
    - all control layers (pose, vel, accel, thrust alloc)
    - all hardware interface nodes
    - camera nodes

3. Ensure that the thrust allocator has the correct paramters loaded
4. send wrench commands 
