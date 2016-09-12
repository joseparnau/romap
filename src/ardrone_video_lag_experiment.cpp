#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Empty.h>

#include <ardrone_autonomy/Navdata.h>

#include <stdio.h>
#include <sstream>
#include <fstream>

#include <signal.h>
#include <termios.h>
#include <stdio.h>

#include <time.h> 
#include <ctime>

#define KEYCODE_R            0x72
#define KEYCODE_TAKEOFF_LAND 0x0A
#define KEYCODE_STOP         0x20


// Ardrone States
#define ARDRONE_OFF       0
#define ARDRONE_INIT      1
#define ARDRONE_LANDED    2
#define ARDRONE_TAKINGOFF 7
#define ARDRONE_FLYING    4
#define ARDRONE_LANDING   9
static unsigned int ardrone_state = ARDRONE_OFF;
void ardroneNavDataCallback(const ardrone_autonomy::Navdata::ConstPtr& msg)
{
  ardrone_state = msg->state;
//   ROS_INFO("ar state: %d", ardrone_state);
  return;
}



class TeleopArdrone
{
public:
  TeleopArdrone(int argc, char** argv);

  void keyLoop();

private:
  
  std_msgs::Empty emptyMsg;
  
  ros::Publisher twist_pub_;
  ros::Publisher pubArdroneTakeOff;
  ros::Publisher pubArdroneLand;  
};

TeleopArdrone::TeleopArdrone(int argc, char** argv)
{
  ros::init(argc, argv, "ardrone_video_lag_experiment_manager");  
  
  ros::NodeHandle nh_;
  
  twist_pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 1);
  pubArdroneTakeOff = nh_.advertise<std_msgs::Empty>("ardrone/takeoff", 1);
  pubArdroneLand = nh_.advertise<std_msgs::Empty>("ardrone/land", 1);
  
  ros::Subscriber subArdroneNavData = nh_.subscribe("ardrone/navdata", 10, ardroneNavDataCallback); 
}

int kfd = 0;
struct termios cooked, raw;

void quit(int sig)
{
  tcsetattr(kfd, TCSANOW, &cooked);
  ros::shutdown();
  exit(0);
}



std::ofstream expFile;


int main(int argc, char** argv)
{
  TeleopArdrone teleop_ardrone(argc, argv);

  expFile.open("/home/users/josep.a.claret/SparkleShare/iocdocs/projectes/romap/experiments/video_lag_rotation_press_key/videoLagExpFile.dat", std::ofstream::trunc);    
  
  signal(SIGINT,quit);

  puts("Initializing...");
  
  teleop_ardrone.keyLoop();
  
  expFile.close();  
  
  return(0);
}


void TeleopArdrone::keyLoop()
{
  char c;
  bool dirty = false;


  bool landed = true;
  
  bool rotating = false;
  
  double z_angular_vel = 0.0;
  double z_angular_vel_value = 0.3;
  

  timeval current_time_;  
  double time_sec = 0.0;
  

  // get the console in raw mode                                                              
  tcgetattr(kfd, &cooked);
  memcpy(&raw, &cooked, sizeof(struct termios));
  raw.c_lflag &=~ (ICANON | ECHO);
  // Setting a new line, then end of file                         
  raw.c_cc[VEOL] = 1;
  raw.c_cc[VEOF] = 2;
  tcsetattr(kfd, TCSANOW, &raw);

  puts("---------------------------");
  puts("Enter        - Takeoff / Land");
  puts("R            - Press to start rotation and when delay is perceived"); 
  puts("Space        - Set twist to zero");  
  puts("---------------------------");

  for(;;)
  {
    // get the next event from the keyboard
    if(read(kfd, &c, 1) < 0)
    {
      perror("read():");
      exit(-1);
    }
    ROS_DEBUG("value: 0x%02X", c);
//     ROS_INFO("value: 0x%02X", c);

    gettimeofday(&current_time_, NULL);
    time_sec = (current_time_.tv_sec*1e6 + current_time_.tv_usec) / 1e6;    
    
    switch(c)
    {
      // Take off / land
      case KEYCODE_TAKEOFF_LAND:
        ROS_DEBUG("TAKEOFF / LAND");
	z_angular_vel = 0.0;	
        dirty = true;
	if (landed)	pubArdroneTakeOff.publish(emptyMsg);
	else		pubArdroneLand.publish(emptyMsg);
	landed = !landed;
        break;
	
      // Stop
      case KEYCODE_STOP:
        ROS_DEBUG("STOP");
	z_angular_vel = 0.0;
        dirty = true;
        break;	
      
      // Rotate and delay
      case KEYCODE_R:
        ROS_DEBUG("ROTATE / DELAY");
	z_angular_vel = z_angular_vel_value;
	dirty = true;
	if (rotating)	expFile << std::setprecision(30) << time_sec << " " << 1 << std::endl;
	rotating = true;
        break;
    }

    geometry_msgs::Twist twist;
    twist.linear.x = 0.0;
    twist.linear.y = 0.0;
    twist.linear.z = 0.0;
    twist.angular.x = 0.0;
    twist.angular.y = 0.0;
    twist.angular.z = z_angular_vel;
    if(dirty == true)
    {
      twist_pub_.publish(twist);    
      dirty=false;
    }
  }

  return;
}