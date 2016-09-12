#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Empty.h>

#include <signal.h>
#include <termios.h>
#include <stdio.h>

#define KEYCODE_R             0x64	// Right
#define KEYCODE_L             0x61	// Left
#define KEYCODE_F             0x77	// Forward
#define KEYCODE_B             0x73	// Backwards
#define KEYCODE_U             0x41	// Up
#define KEYCODE_D             0x42	// Down
#define KEYCODE_ROT_ZPOS      0x44	// Rotate around Z axis positive
#define KEYCODE_ROT_ZNEG      0x43	// Rotate around Z axis negative

#define KEYCODE_TAKEOFF_LAND  0x0A	// Take-off / land

#define KEYCODE_STOP          0x20	// Stop

#define KEYCODE_Q             0x71	// Quit

class TeleopArdrone
{
public:
  TeleopArdrone();
  void keyLoop();

private:
  
  ros::NodeHandle nh_;
  double linear_x_, linear_y_, linear_z_;
  double angular_x_, angular_y_, angular_z_;
  double l_scale_, a_scale_;
  
  std_msgs::Empty emptyMsg;
  
  ros::Publisher twist_pub_;
  ros::Publisher pubArdroneTakeOff;
  ros::Publisher pubArdroneLand;  
};

TeleopArdrone::TeleopArdrone():
  linear_x_(0), linear_y_(0), linear_z_(0),
  angular_x_(0), angular_y_(0), angular_z_(0),
  l_scale_(0.1),
  a_scale_(0.1)
{
  twist_pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 1);
  pubArdroneTakeOff = nh_.advertise<std_msgs::Empty>("ardrone/takeoff", 1);
  pubArdroneLand = nh_.advertise<std_msgs::Empty>("ardrone/land", 1);
}

int kfd = 0;
struct termios cooked, raw;

void quit(int sig)
{
  tcsetattr(kfd, TCSANOW, &cooked);
  ros::shutdown();
  exit(0);
}




int main(int argc, char** argv)
{
  ros::init(argc, argv, "teleop_ardrone");
  TeleopArdrone teleop_ardrone;

  signal(SIGINT,quit);

  puts("Initializing...");
  
  teleop_ardrone.keyLoop();
  
  return(0);
}


void TeleopArdrone::keyLoop()
{
  char c;
  bool dirty=false;

  // ROS


  
  bool landed = true;
  

  // get the console in raw mode                                                              
  tcgetattr(kfd, &cooked);
  memcpy(&raw, &cooked, sizeof(struct termios));
  raw.c_lflag &=~ (ICANON | ECHO);
  // Setting a new line, then end of file                         
  raw.c_cc[VEOL] = 1;
  raw.c_cc[VEOF] = 2;
  tcsetattr(kfd, TCSANOW, &raw);

  puts("---------------------------");
  puts("Ardrone keyboard command.");
  puts("Enter        - Takeoff / Land");
  puts("W / S        - Forward / Backward");  
  puts("A / D        - Left / Right");
  puts("Up / Down    - Up / Down");
  puts("Left / Right - Rotate counter-clockwise / clockwise");
  puts("Space        - Set twist to zero");
  puts("---------------------------");
  puts("Reading from keyboard");
  puts("---------------------------");

  for(;;)
  {
    // get the next event from the keyboard  
    if(read(kfd, &c, 1) < 0)
    {
      perror("read():");
      exit(-1);
    }

    linear_x_ = linear_y_ = linear_z_ = 0.0;
    angular_x_ = angular_y_ = angular_z_ = 0.0;
    ROS_DEBUG("value: 0x%02X", c);
//     ROS_INFO("value: 0x%02X", c);
  
    switch(c)
    {
      // Take off / land
      case KEYCODE_TAKEOFF_LAND:
        ROS_DEBUG("TAKEOFF / LAND");
        linear_x_ = linear_y_ = linear_z_ = 0.0;
        angular_x_ = angular_y_ = angular_z_ = 0.0;
        dirty = true;
	if (landed)	pubArdroneTakeOff.publish(emptyMsg);
	else		pubArdroneLand.publish(emptyMsg);
	landed = !landed;
        break;
      
      // Move
      case KEYCODE_L:
        ROS_DEBUG("LEFT");
        linear_y_ = 1.0;
        dirty = true;
        break;
      case KEYCODE_R:
        ROS_DEBUG("RIGHT");
        linear_y_ = -1.0;
        dirty = true;
        break;
	
      case KEYCODE_F:
        ROS_DEBUG("FORWARD");
        linear_x_ = 1.0;
        dirty = true;
        break;
      case KEYCODE_B:
        ROS_DEBUG("BARWARD");
        linear_x_ = -1.0;
        dirty = true;
        break;
	
      case KEYCODE_U:
        ROS_DEBUG("UP");
        linear_z_ = 1.0;
        dirty = true;
        break;
      case KEYCODE_D:
        ROS_DEBUG("DOWN");
        linear_z_ = -1.0;
        dirty = true;
        break;

      case KEYCODE_ROT_ZPOS:
        ROS_DEBUG("ROTATE Z POSITIVE");
        angular_z_ = 1.0;
        dirty = true;
        break;
      case KEYCODE_ROT_ZNEG:
        ROS_DEBUG("ROTATE Z NEGATIVE");
        angular_z_ = -1.0;
        dirty = true;
        break;
	
      // Stop
      case KEYCODE_STOP:
        ROS_DEBUG("STOP");
        linear_x_ = linear_y_ = linear_z_ = 0.0;
        angular_x_ = angular_y_ = angular_z_ = 0.0;
        dirty = true;
        break;
    }
   

    geometry_msgs::Twist twist;
    twist.linear.x = l_scale_*linear_x_;
    twist.linear.y = l_scale_*linear_y_;
    twist.linear.z = l_scale_*linear_z_;
    twist.angular.x = 0.0;
    twist.angular.y = 0.0;
    twist.angular.z = a_scale_*angular_z_;
    if(dirty ==true)
    {
      twist_pub_.publish(twist);    
      dirty=false;
    }
  }


  return;
}