float64[8] thrust
#simulation setup:
# thrust[0] = fli (front left inner motor)
# thrust[1] = fri (front right inner motor)
# thrust[2] = bli (back left inner motor)
# thrust[3] = bri (back right inner motor)
# thrust[4] = flo (front left outer motor)
# thrust[5] = fro (front right outer motor)
# thrust[6] = blo (back left outer motor)
# thrust[7] = bro (back right outer motor)
#
# In the real AUV, the order of the thrusters needs to match what the esp32 is expecting, as well as
# the thrust allocation matrix (since that determines what thrust we send to each motor)
std_msgs/Header header
