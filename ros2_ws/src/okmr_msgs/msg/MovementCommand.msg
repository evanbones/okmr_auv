int32 command
#defines which of the commands defined below to execute

geometry_msgs/Vector3 translation 
geometry_msgs/Vector3 rotation 

float32 radius_of_acceptance 
#determines how close we have to be to the goal before its considered done
#used in rel move and abs move commands

float32 rotation_speed 
#for spin commands, degree / sec

float32 duration
#can be used for throttle and speed based commands to determine how long a specific command should be executed
#ideally throttle and speed commands get a maximum duration around 
#10 seconds since they are less reliable than position based commands
#can also be used to specify max timeout for relative and absolute move commands

okmr_msgs/GoalPose goal_pose #for absolute move commands

std_msgs/Header header

int32 FREEZE = 0
    #hold current pose, stop all movements going on
    #this will automatically set all pid systems into position mode

int32 MOVE_RELATIVE = 1 
    #does a movement relative to the current (at time received) pose, 
    #ex. move 5m forward, rotate 90 degrees cw, etc
    #
    #translation (x, y, z), defines how many meters to move
    #rotation (r, p, y) in degrees NOT radians
    
    #NOTE: command is relative to the pose the auv was in when the command was received by the action server

int32 MOVE_ABSOLUTE = 2
    #move to the absolute position defined by the goal_pose field
    #can also define translation field, which will move the goal position accordingly
    #direction of translation can be defined by rotation field
    #will be useful for aligning type movements
    #can figure out a surface normal for the gate, slolumn, torpedo, then base rotation field off those
    #ALGORITHM ROUGH IDEA
    #take position from goal pose, then rotate a point (translation distance away) by roll pitch yaw of the rotation field
    #essentially setting goal pose position as the origin, placing the point defined by the translation field
    #then rotating that point around the origin (goal position) by the amount defined by rotation
    #this plan defintely has some flaws and will need to be better defined
    #maybe this method will never need to be implemented who knows


int32 SPIN_YAW = 3
    #spin yaw at the spin speed defined by rotation_speed field
    #turns the rotational pid system into speed mode
    #should keep translation pid in position mode

int32 BARREL_ROLL = 4
    #spin roll at the speed defined by rotation_speed field

int32 DO_A_FLIP = 5
    #spin pitch at the speed defined by rotation_speed field

#applies to both pitch and roll:
#will need to selectively enable pid systems 
#(disable all translation, disable other rotational PID systems, disable dead reckoning)
    #NEEDS TESTING
    #REMEBER, ROTATION DEAD RECKONING DOESNT WORK OUTSIDE OF NORMAL UPRIGHT OPERATION
    #CANNOT BE TRANSLATIONAL, MUST BE SPEED BASED

int32 LOOK_AT = 6
    #could be used for looking at a specific location
    #location to look at defined by "pose"
    #possibly useful for gripper, torpedo, dropper tasks
    #how to do this kind of thing with the bottom facing camera?

int32 SET_DEPTH = 7
    #go down dont #sink




