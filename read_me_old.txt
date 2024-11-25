Jurgen Famula
A.I. Robotics Section 4704
Project 3 task 2
5-14-2022
Read_me

Description:
This package contains all of the source code, executables, and cmake files needed to
launch the wall follow simulation environment and execute the wall_follow behaivor.
Once the simulation enviroment has been initialized, running the Wall_Follow program
will direct the robot to follow the left hand wall. The program accomplishes this by
subscribing to the robot's laser scan topic and descritizing a 180 degree arc, starting
at 340 degrees and ending at 160 degrees, into 3 60 degree arcs. The scan values in each
arc are then averaged and assigned one of 4 distance states based on the average distance between
the wall and the robot with a value of 3 being the ideal distance. These three lazer state values
are then used to determine the overall state of the robot. There are 8 possible states, 7 valid states 
that the robot will encounter and 1 invalid state will propt the simulation to reset, that the 
robot can be in at any given time. These 7 states and the 3 possible actions that the robot can take
are then used to create 3 2D arraylists: the q_reward_table, the q_matrix, and the state_transition table.
1. The q_reward_table contains the reward values for the state transitions between two state given an action
2. The q_matrix contains the current q_value for the state transitions between two states given an action
3. The state_transition table contains the expected next state of the given an action and its current state.
   This is used to determine the expected next reward for q learning.
	The state_transition is intialized based on the expected state transitions before the first episode
	and the table is dynamically updated if the action results in an unexpected state.
Once the states have been assigned, the current state is then passed to the q_follow function which iteritively determines
the robot's next action based on the reward for the current value and the next action, selected via a greed algorith with random
select if there are multiple actions with the same reward value, and the max reward of the expected future state and the future
action. This reward is then saved in the q_matrix.

Launch Instructions:
1. open 2 terminals and navigate to your catkin workspace
2. run roslaunch du_jurgen_famula_p3 wallfollow to launch the gazebo simulation evnironment
3. run rosrun du_jurgen_famula_p3 Wall_Follow to launch the q table based wall following program
