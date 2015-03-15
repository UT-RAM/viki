/*! \file fly_from_keyboard.cpp 
* \brief Fly From Keyboard executable for the RAM package
*
* This package enables you to control a Parrot AR.Drone from the keyboard of your computer or laptop. It publishes to a number of topics:
* - /ardrone/takeoff
* - /ardrone/land
* - /ardrone/reset
* - /cmd_vel
*
* The chosen topics and messages are in line with the ardrone_autonomy package and it's ardrone_driver executable.
* TODO:
*
* - Look into stabilizing code (using the dirty var). It seems to make the control unintuitive.
* - Look into using this code for mulitple drones.
* - Look into the way the loop rate influences the code
* - Make sure the terminal properly exits: now it stays in raw mode
*/

#include <ros/ros.h> 			// ROS header
#include <std_msgs/Empty.h> 		// Class to define an empty message
#include <geometry_msgs/Twist.h>	// Class to define a twist message
#include <termios.h>			// Header for Terminal I/O services 
#include <stdio.h>			// Header for standard I/O operations
#include <sys/poll.h>			// Header needed for poll subroutines
#include <unistd.h>			// Header needed for sleep

/** \name Keyboard Configuration 
* In the following section the character codes of the keys used on the keyboard are defined.
* Use "xmodmap -pk" to get a list of codes that can be used. Use the shorthand notation of the small letter.
*/
/**@{*/
#define KEY_FORWARD 0x77 	/**< Character key for w */
#define KEY_LEFT 0x61 		/**< Character key for a */
#define KEY_BACKWARD 0x73 	/**< Character key for s */
#define KEY_RIGHT 0x64 		/**< Character key for d */
#define KEY_UP 0x6B 		/**< Character key for k */
#define KEY_DOWN 0x6D 		/**< Character key for m */
#define KEY_TURNR 0x65 		/**< Character key for e */
#define KEY_TURNL 0x71 		/**< Character key for q */
#define KEY_RESET 0x72 		/**< Character key for r */
#define KEY_LAUNCH 0x6F 	/**< Character key for o */
#define KEY_LAND 0x6C 		/**< Character key for l */
#define KEY_EXIT 0x58		/**< Character key for X (capital!) */	
/**@}*/

geometry_msgs::Twist twist_msg;	/**< Prepare the twist message to be filled later */
std_msgs::Empty emp_msg;	/**< Prepare the empty message. We can send this immediately when necessary */
int speed_x = 0;		/**< Set speed in x direction to zero initially. positive = forward */
int speed_y = 0;		/**< Set speed in y direction to zero initially. positive = left */
int speed_z = 0;		/**< Set speed in z direction to zero initially. positive = up */
int turn = 0;			/**< Set rotation around z to zero initially. positive = clockwise */
double speed_factor = 0.5;	/**< Include a factor to scale the linear motions in the final twist. Allowed values for twist are between -1 and 1. In this way you can slow down all motions.*/
double turn_factor = 0.5;	/**< Include a turnfactor to scale the rotations in the final twist. Allowed values for twist are between -1 and 1. */
bool dirty = false; 		/**< Boolean variable used to find out if we have to stabalize the quadcopter. */
bool hoverwait = false;		/**< Boolean variable counteracting the auto repeat delay keyboard setting */
bool running = true;		/**< Boolean variable that indicates if we should close control. */
int kfd = 0;			/**< Integer referring to terminal window */
struct termios cooked, raw;	/**< Struct to enable manipulation of the terminal. */


/**
* Main function
*/
int main(int argc, char** argv)
{
	ros::init(argc, argv, "fly_from_keyboard");
	ros::NodeHandle node;		
	ros::Rate loop_rate(20); 	// Set loop rate. 20 should be high enough for keyboard input
	ros::Publisher pub_twist; 	// Initialize publisher for twist command
	ros::Publisher pub_takeoff; 	// Initialize publisher for take-off command
	ros::Publisher pub_land; 	// Initialize publisher for land command
	ros::Publisher pub_reset; 	// Initialize publisher for reset command

	pub_twist = node.advertise<geometry_msgs::Twist>("cmd_vel",1); 		// Set pub_twist as the publisher to publish a twist to /cmd_vel
	pub_takeoff = node.advertise<std_msgs::Empty>("ardrone/takeoff",1); 	// Set pub_takeoff as the publisher to publish an empty message to /ardrone/takeoff
	pub_land = node.advertise<std_msgs::Empty>("ardrone/land",1); 		// Set pub_land as the publisher to publish an empty message to /ardrone/land
	pub_reset = node.advertise<std_msgs::Empty>("ardrone/reset",1); 	// Set pub_reset as the publisher to publish an empty message to /ardrone/reset
	
	char c; 					// Initialize a char c. This variable will hold our keyboard input
	tcgetattr(kfd, &cooked);			// Get the attributes of the terminal referenced by kfd and store them in the termios struct cooked. These are the old settings.
	memcpy(&raw, &cooked, sizeof(struct termios));	// Copy the memory block of the size of termios from the location of cooked to the location of raw. 
	raw.c_lflag &=~ (ICANON | ECHO);		// Set local modes (canonical: content is available line by line; echo: echo input characters)
	raw.c_cc[VEOL] = 1;				// Set End of Line special character to 1 (?)
	raw.c_cc[VEOF] = 2;				// Set End of File special character to 2 (?)
	tcsetattr(kfd, TCSANOW, &raw);			// Set the attributes of the terminal kfd to process every change immediately (AKA: process every key as soon as it get in)

	puts("Reading from keyboard.");			// Write message to terminal
	
	struct pollfd ufd;		// Create poll struct (poll = waiting for some event on a file descripter)			
	ufd.fd = kfd;			// The file descriptor we are referring to is the terminal input
	ufd.events = POLLIN;		// Event we want to track: there is data to read --> POLLIN

	// Keyboard loop.
	while(running)
	{
		int num;

		// Poll &ufd for 1 item with a timeout of 250 ms. If this returns < 0, there is an error. If positive, there are to be read items.
		if ((num = poll(&ufd, 1, 250)) < 0)
		{
			perror("poll():");
			return 0;
		}
		else if(num > 0)
		{
			// There is an event, so read the terminal input and place it in c. Under 0 should be impossible, so throw an error.
			if(read(kfd, &c, 1) < 0)
			{
				perror("read():");
				return 0;
			}
		}
		else
		{
			// Very nasty piece of code... the problem is that holding a key means, since we are using the terminal, repeating a character. To make text input more user-friendly however, it takes a while before the keyboard starts repeating the character (Auto Key Repeat Delay). So: before stabilizing, wait a period (intel based default to .25s) before sending the stabalize signal. Do reset the variables though, since new input can be received. 
			if(dirty == true && hoverwait == false)
			{			
				hoverwait = true;
				usleep(250000);
			}
			//If the boolean variable dirty is true, there has been sent a command. To stop the robot from moving in that direction, we have to "stabalize": everything 0.
			if(dirty == true && hoverwait == true)
			{
				twist_msg.linear.x= 0.0;
				twist_msg.linear.y= 0.0;
				twist_msg.linear.z= 0.0;
				twist_msg.angular.z = 0.0;
				pub_twist.publish(twist_msg);
				dirty = false;
				hoverwait = false;
			}

			// Reset vars at this point to enable 
			continue; // Skip over rest of iteration. Might not be necessary.
		}
		
		speed_x = 0;
		speed_y = 0;
		speed_z = 0;	
		turn = 0;
		// c contains a single character now, determining the action. Since we only have one character, there is no real support for diagonal motion.
		switch(c)
		{

			case KEY_LAUNCH:
				pub_takeoff.publish(emp_msg);		// Publish an empty message to take off
				dirty = true;
				break;
			case KEY_RESET:
				pub_reset.publish(emp_msg);		// Publish an empty message to reset
				dirty = true;
				break;
			case KEY_LAND:
				pub_land.publish(emp_msg);		// Publish an empty message to land
				dirty = true;
				break;
			case KEY_FORWARD:
				speed_x = 1;				// Set x forward
				dirty = true;
				break;
			case KEY_BACKWARD:
				speed_x = -1;				// Set x backward
				dirty = true;
				break;
			case KEY_UP:
				speed_z = 1;				// Set z up
				dirty = true;
				break;
			case KEY_DOWN:
				speed_z = -1;				// Set z down
				dirty = true;
				break;
			case KEY_LEFT:
				speed_y = 1;				// Set y left
				dirty = true;
				break;
			case KEY_RIGHT:
				speed_y = -1;				// Set y right
				dirty = true;
				break;
			case KEY_TURNR:
				turn = 1;				// Set rotation clockwise
				dirty = true;
				break;
			case KEY_TURNL:
				turn = -1;				// Set rotation counter clockwise
				dirty = true;
				break;
			case KEY_EXIT:					// Use with extreme caution
				pub_land.publish(emp_msg); 		// If you end up here, you exit the control. For the sake of safety: send a land command first. PLEASE dot not rely on this feature and land manually. You lost control at this point. 
				usleep(100000);				// Sleep a while. This appears to necessary to send the land command.
				running = false;			// Exit loop.
				break;
			default:
				speed_x = 0;				// Other key? Strange, but why not stabalize?
				speed_y = 0;
				speed_z = 0;
				turn = 0;
			
		}

		twist_msg.linear.x = speed_x*speed_factor;
		twist_msg.linear.y = speed_y*speed_factor;
		twist_msg.linear.z = speed_z*speed_factor;
		twist_msg.angular.z = turn*turn_factor;
		pub_twist.publish(twist_msg);
		ros::spinOnce(); 	// Not necessary, has to do with callbacks on listeners. Good habit.
		loop_rate.sleep();	// Wait the rest of the loop
	}
puts("Stopped reading from keyboard.");
tcsetattr(kfd, TCSAFLUSH, &cooked);	// Use old settings of the terminal.
return(0);
} // main
