bool integrate
# integrate field defines if request wants to 
# actually track that distance covered = duration * velocity 
# if false, we just send velocity setpoint, and wait for duration

float32 duration

geometry_msgs/Twist twist

std_msgs/Header header
