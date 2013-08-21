#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3.h>
#include <sensor_msgs/Joy.h>
#include <iostream>

using namespace std;



class JoystickToCommandVel
{
public:
 JoystickToCommandVel();
private:
 void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);
 ros::NodeHandle nh_;
 
 //Array-Indizes für Achsen des Joysticks
 int linear_, angular_, scale_;
 // Skalierungen für lineare und angulare geschwindigkeiten
 double l_scale_, a_scale_;
 // Ros pub und sub
 ros::Publisher vel_pub_;
 ros::Subscriber joy_sub_;
};



JoystickToCommandVel::JoystickToCommandVel():
 linear_(1),
 angular_(2),
 scale_(3)  // Initialisierung der Achsen (y-Achse linear, Z-Rotationsachse angular, Schubregler Skalierung)
{
 // paramater aus einem launch file besorgen, siehe ros-doku für weitere infos
 /* nh_.param("axis_linear", linear_, linear_);
 nh_.param("axis_angular", angular_, angular_);
 nh_.param("scale_angular", a_scale_, a_scale_);
 nh_.param("scale_linear", l_scale_, l_scale_); */

   l_scale_ = 1.39; //  V2013   1.39 maximale vel für 74:1 Übersetzung
// a_scale_ = 1.39; //  V2013 

 vel_pub_ = nh_.advertise<geometry_msgs::Twist>("robot/cmd_vel", 1);
 joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("joy", 10, &JoystickToCommandVel::joyCallback, this);
}



void JoystickToCommandVel::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
 //cout << "joy message recieved..."; 
 geometry_msgs::Vector3 vectorLinear;
 geometry_msgs::Vector3 vectorAngular;
 geometry_msgs::Twist vel;

 // Skalierung aus der Position der Skalierungsachse berechnen  
 a_scale_ = 1.39*(2.0+joy->axes[scale_]);

 if(joy->axes[scale_] == 0.0) // Im Fall der Initialisierung
{	a_scale_= 1.39;
	
}

// if(l_scale_ < 0.2)
//    l_scale_ = 0.2;    für Skalierung Kommentierung löschen
    
 vectorLinear.x = l_scale_*joy->axes[linear_];
 vectorLinear.y = vectorLinear.z = 0;
 vel.linear = vectorLinear;

 vectorAngular.z = a_scale_*joy->axes[angular_];
 vectorAngular.x = vectorAngular.y = 0;
 vel.angular = vectorAngular;

 //cout << "publishing cmd_vel message...";
 vel_pub_.publish(vel);
}



int main(int argc, char** argv)
{  
 ros::init(argc, argv, "JoystickToCommandVel");
 JoystickToCommandVel joyCommand;
 cout << "GazeboJoystickRunning......\n";
 ros::spin();
}

