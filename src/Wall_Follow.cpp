/*
* Jurgen Famula
* A.I. Robotics section 4704
* Project 3: Wall_Follow.cpp
*
*/
#include "ros/ros.h"
//libraries used
#include "geometry_msgs/PointStamped.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/message_filter.h"
#include "message_filters/subscriber.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Twist.h"
#include "std_msgs/String.h"
#include "std_msgs/Empty.h"
#include <gazebo_msgs/SetModelState.h>
#include <map>
#include <random>
#include <sstream>
#include <iostream>
#include <fstream>


//structures
struct point {
  float x = 0;
  float y = 0;
  float z = 0;
};

struct action_space {
  int type = 0;
  float x = 0;
  float w = 0;
  float t = 0.1;

  inline bool operator==(const action_space& a) {
       if (a.x==x && a.w== w)
          return true;
       else
          return false;
    }
};

struct touple{
  int reward = INT_MIN;
  int action = -1;

  inline bool operator<=(const touple& a) {
       if (reward<=a.reward)
          return true;
       else
          return false;
    }
};

//enumerator that represents possible states of the robot as it traverses the loop.
enum robo_state{
  HEAD_ON_COLLISION,
  PARALLEL_COLLISION,
  APPROCHING_90_DEGREE_TURN,
  AT_EDGE,
  NEAR_WALL,
  IDEAL_DISTANCE,
  TOO_FAR,
  INVALID_STATE
};

//Wall_Follow class definition
class Wall_Follow_Pub_Sub
{
  private:
    ros::NodeHandle n_;
    ros::Publisher cmd_pub_;
    ros::Subscriber cmd_sub_;
    ros::Subscriber laser_sub_;
    ros::ServiceClient gazebo_client_;
    tf2_ros::Buffer tfBuffer;

    //The distance from the wall that robot will maintain in meters
    float d_wall = 0.5;
    int scan_interval = 120;
    int min_angle = 350;
    int max_angle = 110;
    //the number of possible states and actions (for part 1, 3 actions and 4 states)
    int actions = 3;
    int laser_states = 4;
    int robo_states = 7;
    int descretization_size = 40;

    //The action spaces that the robot will execute
    action_space forward;
    action_space turn_left;
    action_space turn_right;

    //This laserscan object holds the value of the current LaserScan
    sensor_msgs::LaserScan scan;

    //the static reward table for part 1 (The indices represent the possible states by the possible actions,
    // and the values are the rewards)
    std::vector<int> q_reward_table;

    //The dynamic q matrix for q_learning
    std::vector<int> q_matrix;

    //A static table of state transitions for the robot.
    std::vector<int> state_transitions;

    robo_state cur_state = INVALID_STATE;
    ros::Time cur_state_start;
    int last_action;
    touple q_value;

    geometry_msgs::Pose start_pose;
    geometry_msgs::Twist start_twist;


    //callback functions
    void get_laser_message(const sensor_msgs::LaserScan::ConstPtr& msg)
    {
      scan = *msg;
    }

    void cmd_callback(const geometry_msgs::Twist::ConstPtr& msg)
    {
      //ROS_INFO("The current velocity in the x direction is %g", msg->linear.x);
      //ROS_INFO("The current velocity in the y direction is %g", msg->linear.y);
    }

    //init functions
    /* Init_q_tables: This function populates the q_reward_table and the q_matrix.
    The values in the reward table contains the reward for each actions the robot can take at a given state.
    There are three possible values in the reward table, -1, 0, and 100.
     -1 represents an action that will result in a transition to an illegal state, i.e.: collision with a wall.
     0 represents an action that will result in a transition to a valid state
     100 represents an action that will result in a transition to the goal state or keep the robot in the goal state

     The q_matrix represents the usefullness of taking a given action when the robot is in a given state.
     The values in the table are all initialized to zero as the robot has no information about the usefullness
     of any given action.

     */
    void init_q_tables()
    {
      for (int i = 0; i < robo_states; i++)
      {
        for (int j = 0; j < actions; j++)
        {
          switch (i) {
            //the init the reward values for each action the robot can take while in the HEAD_ON_COLLISION state
            case 0 :
              switch (j) {
                //set the reward values for the forward action
                case 0 :
                  q_reward_table.emplace_back(-1);
                  q_matrix.emplace_back(0);
                  state_transitions.emplace_back(HEAD_ON_COLLISION);
                  break;
                  //set the reward values for the turn_left action
                case 1 :
                  q_reward_table.emplace_back(-1);
                  q_matrix.emplace_back(0);
                  //This transition is invalid as the robot will now be traveling counter-clock wise around
                  // the environment and can no longer detect the wall.
                  state_transitions.emplace_back(INVALID_STATE);
                  break;
                  //set the reward values for the turn_right action
                case 2 :
                  q_reward_table.emplace_back(0);
                  q_matrix.emplace_back(0);
                  state_transitions.emplace_back(PARALLEL_COLLISION);
                  break;
              }
              break;
            //the reward values for each action the robot can take while in the PARALLEL_COLLISION state
            case 1 :
              switch (j) {
                case 0 :
                  q_reward_table.emplace_back(0);
                  q_matrix.emplace_back(0);
                  state_transitions.emplace_back(PARALLEL_COLLISION);
                  break;
                case 1 :
                  q_reward_table.emplace_back(-1);
                  q_matrix.emplace_back(0);
                  state_transitions.emplace_back(HEAD_ON_COLLISION);
                  break;
                case 2 :
                  q_reward_table.emplace_back(0);
                  q_matrix.emplace_back(0);
                  state_transitions.emplace_back(NEAR_WALL);
                  break;
              }
              break;
            //the reward values for each action the robot can take while in the APPROCHING_90_DEGREE_TURN state
            case 2 :
              switch (j) {
                case 0 :
                  q_reward_table.emplace_back(0);
                  q_matrix.emplace_back(0);
                  state_transitions.emplace_back(HEAD_ON_COLLISION);
                  break;
                case 1 :
                  q_reward_table.emplace_back(-1);
                  q_matrix.emplace_back(0);
                  state_transitions.emplace_back(HEAD_ON_COLLISION);
                  break;
                case 2 :
                  q_reward_table.emplace_back(0);
                  q_matrix.emplace_back(0);
                  state_transitions.emplace_back(PARALLEL_COLLISION);
                  break;
              }
              break;
            //the reward values for each action the robot can take while in the AT_EDGE state
            case 3 :
              switch (j) {
                case 0 :
                  q_reward_table.emplace_back(-1);
                  q_matrix.emplace_back(0);
                  state_transitions.emplace_back(TOO_FAR);
                  break;
                case 1 :
                  q_reward_table.emplace_back(-1);
                  q_matrix.emplace_back(0);
                  state_transitions.emplace_back(PARALLEL_COLLISION);
                  break;
                case 2 :
                  q_reward_table.emplace_back(0);
                  q_matrix.emplace_back(0);
                  state_transitions.emplace_back(TOO_FAR);
                  break;
              }
              break;
            //the reward values for each action the robot can take while in the NEAR_WALL state
            case 4 :
              switch (j) {
                case 0 :
                  q_reward_table.emplace_back(0);
                  q_matrix.emplace_back(0);
                  state_transitions.emplace_back(NEAR_WALL);
                  break;
                case 1 :
                  q_reward_table.emplace_back(0);
                  q_matrix.emplace_back(0);
                  state_transitions.emplace_back(PARALLEL_COLLISION);
                  break;
                case 2 :
                  q_reward_table.emplace_back(0);
                  q_matrix.emplace_back(0);
                  state_transitions.emplace_back(IDEAL_DISTANCE);
                  break;
              }
              break;
            //the reward values for each action the robot can take while in the IDEAL_DISTANCE state
            case 5 :
              switch (j) {
                case 0 :
                  q_reward_table.emplace_back(100);
                  q_matrix.emplace_back(0);
                  state_transitions.emplace_back(IDEAL_DISTANCE);
                  break;
                case 1 :
                  q_reward_table.emplace_back(0);
                  q_matrix.emplace_back(0);
                  state_transitions.emplace_back(NEAR_WALL);
                  break;
                case 2 :
                  q_reward_table.emplace_back(0);
                  q_matrix.emplace_back(0);
                  state_transitions.emplace_back(TOO_FAR);
                  break;
              }
              break;
            //the reward values for each action the robot can take while in the TOO_FAR state
            case 6 :
              switch (j) {
                case 0 :
                  q_reward_table.emplace_back(0);
                  q_matrix.emplace_back(0);
                  state_transitions.emplace_back(TOO_FAR);
                  break;
                case 1 :
                  q_reward_table.emplace_back(0);
                  q_matrix.emplace_back(0);
                  state_transitions.emplace_back(IDEAL_DISTANCE);
                  break;
                case 2 :
                  q_reward_table.emplace_back(-1);
                  q_matrix.emplace_back(0);
                  state_transitions.emplace_back(TOO_FAR);
                  break;
              }
              break;
          }
        }
      }
    }

    void init_start_pose_twist()
    {
      start_pose.position.x = -2.0;
      start_pose.position.y = 2.0;
      start_pose.position.z = 0.1;
      start_pose.orientation.x = 0.0;
      start_pose.orientation.y = 0.0;
      start_pose.orientation.z = 0.0;
      start_pose.orientation.w = 0.0;

      start_twist.linear.x = 0.0;
      start_twist.linear.y = 0.0;
      start_twist.linear.z = 0.0;
      start_twist.angular.x = 0.0;
      start_twist.angular.y = 0.0;
      start_twist.angular.z = 0.0;
    }

    void init_action_spaces()
    {
      //init forward
      forward.type = 0;
      forward.x = 0.1;

      //init turn_left
      turn_left.type = 1;
      turn_left.x = 0.2;
      turn_left.w = M_PI/6;

      //init turn_right
      turn_right.type = 2;
      turn_right.x = 0.05;
      turn_right.w = -(M_PI/6);
    }

    action_space get_action(int i) {
      if (i == 0)
      {
        return forward;
      }
      else if (i == 1)
      {
        return turn_left;
      }
      else if (i == 2)
      {
        return turn_right;
      }
    }

    //get_states function for part 1
    void get_states(std::vector<int>& states)
    {
      //ROS_INFO("Begin get_states");
      //descretize the laser scan data
      int num_states = (int)scan_interval / (int)descretization_size;
      //state 0: left state, state 1: front left state, state 2: front state
      //ROS_INFO("Set min_a");
      int min_a = min_angle;

      //ROS_INFO("begin loop");
      for (int i = 0; i < num_states; i++)
      {
        //ROS_INFO("%d", num_states);
        int max_a = (min_a + descretization_size)%360;
        /*Descretization steps:
        * state 0: collision range (ranges dangously close to the wall defined as the range from 0 to 0.2)
        * state 1: close range (defined as the value of the laser scan being greater then 0.2 but less then d_wall)
        * state 2: goal range (defined as the value being within 0.1 m of d_wall)
        * state 3: far range (defined as a scan result greater then 0.1+d_wall)
        */
        if (min_a > max_a)
        {
          min_a -= 359;
        }
        float avg_range = scan.ranges[min_a];
        int count = 1;
        //ROS_INFO("begin inner loop");
        for (int j = min_a+1; j < max_a; j++)
        {
          //ROS_INFO("%d", j);
          if (j < 0)
          {
            int index = j + 359;
            avg_range += scan.ranges[index];
          }
          else
          {
            //ROS_INFO("get range %g", avg_range);
            avg_range += (float)scan.ranges[j];
          }
          //ROS_INFO("increment count");
          count++;
        }
        avg_range /= count;
        /*ROS_INFO("min_a = %i", min_a);
        ROS_INFO("max_a = %i", max_a);
        ROS_INFO("count = %i", count);
        ROS_INFO("avg_range = %g", avg_range);*/

        if (i == 0 || i == 2)
        {
          if (avg_range <= (d_wall-0.1))
          {
            states.emplace_back(0);
          }
          else if (avg_range > (d_wall-0.1) && avg_range < (d_wall-0.03))
          {
            states.emplace_back(1);
          }
          else if (avg_range >= (d_wall-0.03) && avg_range <= (d_wall+0.03))
          {
            states.emplace_back(2);
          }
          else if (avg_range > (d_wall+0.03) && avg_range <= (d_wall+0.08))
          {
            states.emplace_back(3);
          }
          else
          {
            states.emplace_back(4);
          }
        }
        else
        {
          if (avg_range > (d_wall+0.1))
          {
            states.emplace_back(3);
          }
          else
          {
            states.emplace_back(1);
          }
        }
        min_a = max_a;
      }
    }

    void get_max_reward_value(robo_state state, touple& max_val)
    {
      //iterate through the q_matrix to determine if there is a state transition that has
      // been shown to have utility greater then 0.
      for (int i = 0; i < actions; i++)
      {
        if (q_matrix[(actions*state)+i] > 0)
        {
          if (q_matrix[(actions*state)+i] > max_val.reward)
          {
            max_val.reward = q_reward_table[(actions*state)+i];
            max_val.action = i;
          }
          else if (q_matrix[(actions*state)+i] == max_val.reward)
          {
            if (rand() % 2 == 0)
            {
              max_val.action = i;
            }
          }
        }
      }

      for (int i = 0; i < actions; i++)
      {
        if (q_reward_table[(actions*state)+i] > max_val.reward)
        {
          max_val.reward = q_reward_table[(actions*state)+i];
          max_val.action = i;
        }
        else if (q_reward_table[(actions*state)+i] == max_val.reward)
        {
          if (rand() % 2 == 0)
          {
            max_val.action = i;
          }
        }
      }
    }

    // This function takes in the array of laser scan states and returns the
    // general state of the robot.
    int get_robot_state(std::vector<int> states)
    {
      int L_state = states[2]; //left state
      int LF_state = states[1]; //left forward state
      int F_state = states[0]; //forward state

      switch (F_state) {
        case 4:
          switch (LF_state) {
            case 3 :
              switch (L_state) {
                case 4:
                  return TOO_FAR;
                case 3 :
                  return TOO_FAR;
                case 2 :
                  return IDEAL_DISTANCE;
                case 1 :
                  return IDEAL_DISTANCE;
                case 0 :
                  return PARALLEL_COLLISION;
              }
              break;
             case 2 :
              switch (L_state) {
                case 4:
                  return NEAR_WALL;
                case 3 :
                  return NEAR_WALL;
                case 2 :
                  return IDEAL_DISTANCE;
                case 1 :
                  return NEAR_WALL;
                case 0 :
                  return PARALLEL_COLLISION;
              }
              break;
            case 1 :
              switch (L_state) {
                case 4 :
                  return AT_EDGE;
                case 3 :
                  return AT_EDGE;
                case 2 :
                  return NEAR_WALL;
                case 1 :
                  return NEAR_WALL;
                case 0 :
                  return PARALLEL_COLLISION;
              }
              break;
            default :
              return PARALLEL_COLLISION;
         }
        case 3 :
          return APPROCHING_90_DEGREE_TURN;
        case 2 :
          return APPROCHING_90_DEGREE_TURN;
        case 1 :
          return APPROCHING_90_DEGREE_TURN;
        default :
          return HEAD_ON_COLLISION;
      }
    }

    void publish_cmd_vel(action_space a)
    {
      geometry_msgs::Twist cmd;
      cmd.linear.x = a.x;
      cmd.angular.z = a.w;
      cmd_pub_.publish(cmd);
      ros::Duration(a.t).sleep();
    }

    //unused functions for part 2
    void q_follow(robo_state state, touple& q_value)
    {
      get_max_reward_value(state, q_value);
    }

    //iterative q-learning algorithm for part 3
    void q_learning(robo_state state, touple& q_value)
    {
      touple next_value;
      get_max_reward_value(state, q_value);
      robo_state future_state = (robo_state)state_transitions[(state*actions)+q_value.action];
      get_max_reward_value(future_state, next_value);
      int utility = q_value.reward + 0.8*next_value.reward;
      q_matrix[(state*actions)+q_value.action] = q_matrix[(state*actions)+q_value.action]+utility;
    }

    int apply_penalty(robo_state state, int action)
    {
      if (q_reward_table[(state*actions)+action] > 0 && q_reward_table[(state*actions)+action] != 100)
      {
        q_reward_table[(state*actions)+action] = q_reward_table[(state*actions)+action]-1;
        q_matrix[(state*actions)+action] = q_matrix[(state*actions)+action]-1;
      }
      return q_reward_table[(state*actions)+action];
    }

    void test_robo_state(robo_state state)
    {
      switch(state) {
        case HEAD_ON_COLLISION :
          publish_cmd_vel(turn_right);
          break;
        case PARALLEL_COLLISION :
          publish_cmd_vel(turn_right);
          break;
        case APPROCHING_90_DEGREE_TURN :
          publish_cmd_vel(turn_right);
          break;
        case AT_EDGE :
          publish_cmd_vel(turn_right);
          break;
        case NEAR_WALL :
          publish_cmd_vel(turn_right);
          break;
        case IDEAL_DISTANCE :
          publish_cmd_vel(forward);
          break;
        case TOO_FAR :
          publish_cmd_vel(turn_left);
          break;
      }
    }

    void print_robo_state(robo_state state) {
      std::string msg = "The current state of the robot is: ";
      switch(state) {
        case HEAD_ON_COLLISION :
          msg.append("HEAD_ON_COLLISION");
          break;
        case PARALLEL_COLLISION :
          msg.append("PARALLEL_COLLISION");
          break;
        case APPROCHING_90_DEGREE_TURN :
          msg.append("APPROCHING_90_DEGREE_TURN");
          break;
        case AT_EDGE :
          msg.append("AT_EDGE");
          break;
        case NEAR_WALL :
          msg.append("NEAR_WALL");
          break;
        case IDEAL_DISTANCE :
          msg.append("IDEAL_DISTANCE");
          break;
        case TOO_FAR :
          msg.append("TOO_FAR");
          break;
      }
      ROS_INFO("%s", msg.c_str());
    }

  public:
    Wall_Follow_Pub_Sub()
    {
      scan.header.frame_id = "i";
      //ROS_INFO("%s", scan.header.frame_id.c_str());
      cmd_pub_ = n_.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
      cmd_sub_ = n_.subscribe("/cmd_vel", 10, &Wall_Follow_Pub_Sub::cmd_callback, this);
      laser_sub_ = n_.subscribe("/scan", 100, &Wall_Follow_Pub_Sub::get_laser_message, this);
      gazebo_client_ = n_.serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state");

      //ROS_INFO("begin pub_sub");
      this->init_q_tables();
      //ROS_INFO("q table initialized");
      this->init_action_spaces();
      //ROS_INFO("action space initialized");
      this->init_start_pose_twist();
    }

    bool get_node_status()
    {
      return n_.ok();
    }

    //print functions for testing
    void print_laser_array()
    {
      std::string msg = "The laserscan returned: ";
      for (float range : scan.ranges)
      {
        std::stringstream stream;
        stream << "[" << range << "], ";
        msg.append(stream.str());
      }
      ROS_INFO("%s", msg.c_str());
    }

    void print_q_reward_table()
    {
      std::string msg = "The q reward table is: ";
      for (int reward : q_reward_table)
      {
        std::stringstream stream;
        stream << "[" << reward << "], ";
        msg.append(stream.str());
      }
      ROS_INFO("%s", msg.c_str());
    }

    void print_q_matrix()
    {
      std::string msg = "The q is: ";
      for (int reward : q_matrix)
      {
        std::stringstream stream;
        stream << "[" << reward << "], ";
        msg.append(stream.str());
      }
      ROS_INFO("%s", msg.c_str());
    }

    void print_state_table(std::vector<int> s)
    {
      std::string msg = "The current states are: ";
      for (int state : s)
      {
        std::stringstream stream;
        stream << "[" << state << "], ";
        msg.append(stream.str());
      }
      ROS_INFO("%s", msg.c_str());
    }

    //the public function that executes the wall following behavior
    void follow_wall()
    {
      //ROS_INFO("scan frame id: %s", scan.header.frame_id.c_str());
      if (scan.header.frame_id.compare("i") == 0)
      {
        ROS_INFO("error: no scan detected");
      }
      else
      {
        std::vector<int> states;

        //ROS_INFO("initializing state table.");
        get_states(states);

        //ROS_INFO("printing state table");
        print_state_table(states);
        int new_state = get_robot_state(states);
        if (cur_state != new_state)
        {
          cur_state = (robo_state)new_state;

          if (new_state != state_transitions[(new_state*actions)+q_value.action])
          {
            state_transitions[(new_state*actions)+q_value.action] = new_state;
          }

          cur_state_start = ros::Time::now();
        }

        print_robo_state((robo_state)new_state);
        q_learning((robo_state)new_state, q_value);
        print_q_matrix();
        publish_cmd_vel(get_action(q_value.action));
        last_action = q_value.action;

        ros::Time cur_time = ros::Time::now();
        ros::Duration cur_duration(cur_time - cur_state_start);
        if(cur_duration.toSec() >= 10 && cur_state != IDEAL_DISTANCE)
        {
          //apply_penalty((robo_state)new_state, last_action);
          gazebo_msgs::ModelState model_state;
          model_state.model_name = (std::string) "turtlebot3_burger";
          model_state.reference_frame = (std::string) "world";
          model_state.pose = start_pose;
          model_state.twist = start_twist;

          gazebo_msgs::SetModelState setmodelstate;
          setmodelstate.request.model_state = model_state;
          gazebo_client_.call(setmodelstate);
          cur_state = INVALID_STATE;
        }
      }
    }
};

int main(int argc, char** argv){

  ros::init(argc, argv,"Wall_Follow");
  ros::NodeHandle node;
  Wall_Follow_Pub_Sub w_follow;
  ros::Duration(2.0).sleep();
  ros::Rate r(10);

  while(w_follow.get_node_status())
  {
    w_follow.follow_wall();
    ros::spinOnce();
    r.sleep();
  }

  return 0;
}
