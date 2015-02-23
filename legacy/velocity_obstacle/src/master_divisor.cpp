// 23/07/2014, Alex Kamphuis, a.kamphuis-1@student.utwente.nl, RAM, UTwente

// debug building: catkin_make --pkg ram_vo -DCMAKE_BUILD_TYPE=Debug

#include <ros/ros.h>	//include ROS main
#include <geometry_msgs/Pose.h> // Geometry messages
#include <std_msgs/String.h> // string message, used only for testing
#include <ram_vo/robot_state.h> // custom message

//#include <sys/types.h> // for fork
//#include <unistd.h> // for fork

class MasterDivisor
{
private:
	/* Communication with ROS */
		ros::NodeHandle node_handle_; // used to initialise this node. node_handle_ is now how i can reach it
		ros::Publisher info_publisher_; // publisher to test topic	
		std::vector<ros::Subscriber> subscribers; // subscribers to unfiltedred_pose robots
		std::vector<ros::Publisher> publishers; // publishers to all robots

	/* initialize topic variables */
		std::vector<ros::master::TopicInfo> all_topics; // vector to store topic infos
		std::string topic_name; // name of a topic

	/* info on robots */		
		std::vector<std::string> v_all_robots_; // names of all robots	

		/* for velocity estimation */
			static const int MAXTIMEWINDOW=15;
			static const int TIMEWINDOW=10;

			struct robot_speed
			{
				float x;
				float y;
				float z;
			};
			std::vector<robot_speed> v_robot_speeds_;

			struct robot_position
			{
				double time;
				float x;
				float y;
				float z;
			};
			std::vector<std::vector<robot_position> > v_robot_positions_;
	
	/* info on simulation */
		bool simulation_;
	
public:
	MasterDivisor() // constructor
	{
		/*
			MasterDivisor scans through all topics and assumes the ones that end in /unfiltered_pose to be robots.

			It then subscribes to all these unfiltered poses, and publishes all information: robot names, numbers and poses to all robots.

			In the future this function might need to start controllers for every robot, but now it does not
		*/
		
		/* behaviour variables */

		/* store robot names in v_all_robots_ */ 		
			/*
			input: topic name list by requestion the ros master
			output: vector of robot names, all names preceded with /

			description:
			all robot names are returned v_all_robots_ by running through all topic names,
			and getting all names that end in /unfiltered_pose.

			An exception is caught for topic names that are not long enough to contain '/unfiltered_pose'
			the exception is printed to ROS_DEBUG.
			*/

			if (TIMEWINDOW>=MAXTIMEWINDOW-1)
			{
				ROS_ERROR("MAXTIMEWINDOW is too small to go back the number of steps required by TIMEWINDOW");
				ros::shutdown();
			}

			// if this is a simulation, we only use John
			ROS_DEBUG("getting simulation parameter from server...");
			if (node_handle_.getParam("master_divisor/simulation",simulation_))
			{
				ROS_DEBUG_STREAM("simulation_:"<<simulation_);	
			}
			else
			{
				simulation_=false;
				ROS_DEBUG("Unable to get 'simulation' parameter from server. Resuming as if it was set to false.");
			}
			
			if(!(simulation_))
			{
				/* get all topics into a vector */
				ros::master::getTopics(all_topics); // get all topics into all_topics
				ros::Rate loop_rate(10); // set a loop rate, to give getTopics enough time
				loop_rate.sleep(); // wait a while to give getTopics time

				for (int ii=0;ii<all_topics.size();ii++){ // loop through all topics
					//check if it ends in /unfiltered_pose
					topic_name = all_topics.at(ii).name; // get the name of the topic
					try // catch out of range exception for too short topic names
					{
						if (topic_name.substr(topic_name.size()-16,16)=="/unfiltered_pose") // if a topic name ens in unfilterd pose
						{
							v_all_robots_.push_back(topic_name.substr(0,topic_name.size()-16)); // get the robot name
							std::stringstream ss;
							ss<<"found robot named: "<<topic_name.substr(0,topic_name.size()-16).c_str()<<" with number: "<<v_all_robots_.size()-1;
							ROS_INFO_STREAM(ss.str()); // output that we've found a robot	
						}	
					}
					// some error catching, only shown in debug
					catch(std::out_of_range)
					{
						ROS_DEBUG("skipped one topic because it is not long enough while looking for topics with /unfiltered_pose in their name");
					}

					ros::spinOnce(); //process callbacks that are waiting
				}
			}
			else
			{
				v_all_robots_.push_back("/John");
				v_all_robots_.push_back("/James");
				ROS_INFO("Running Simulation");
			}
		/* end of: store robot names in v_all_robots_ */

			
		/* subscribe and advertise to all robot topics */
			for (int ii=0;ii<v_all_robots_.size();ii++) // run through all robots
			{
				std::stringstream ss; // create a stringstream
				ss<<v_all_robots_.at(ii)<<"/unfiltered_pose"; // fill it with robotname/unfiltered_pose
				//ROS_INFO_STREAM(ss); //output to make sure we get the right stringstream
				

				/* the following line:
					subscribes to: unfiltered poses of all robots
					calls when recieved: sender, with the message and robot number as argument
				*/
				subscribers.push_back(node_handle_.subscribe<geometry_msgs::Pose>( //subscribe
					ss.str(), // to this topic
					100, // no of msgs to remember
					//boost::bind(printPosInfo,_1,v_all_robots_.at(ii))));
					boost::bind(&MasterDivisor::positionCallback, // call this function
						this,	// which belongs to this claas
						_1,						
						//v_all_robots_.at(ii)  // send robot argument name as argument
						ii 					// send robot nr as argument
						)));

				ss.str(""); // clear the temporary string stream
				ss<<v_all_robots_.at(ii)<<"/info_from_divisor"; // fill it with the topic to publish to
				publishers.push_back( // fill the publishers vector with:
					node_handle_.advertise<ram_vo::robot_state>( // publishers to:
						ss.str() // this topic
						,100 // keep this many messages
						));

			}
		/* end of subscribe and advertise to all robot topics */

	}
	
	~MasterDivisor() // destructor
	{

	}

	void positionCallback(const geometry_msgs::Pose::ConstPtr & pose, int robot_number)
	{
		storePosition(pose, robot_number);
		calcVelocity(robot_number);
		sender(pose, robot_number);
	}

	void storePosition(const geometry_msgs::Pose::ConstPtr & pose, int robot_number)
	{
		// create a vector with an empty entry
			std::vector<robot_position> zero_vector;		
		
		while(v_robot_positions_.size()<=robot_number)
		{	
			// if the memory vector for this robot does not exist
				
				//std::stringstream ss;
				//ss<<"v_robot_positions_.size(): "<<v_robot_positions_.size();
				//ROS_INFO_STREAM(ss.str());

			// add another zero-vector to it
			v_robot_positions_.push_back(zero_vector);
		}
		std::stringstream ss;
		ss<<"v_robot_positions_.size(): "<<v_robot_positions_.size();
		ROS_DEBUG_STREAM(ss.str());

		/* build the structure containing position and time info */
			// declaration
			robot_position position;
			
			// time info
			double current_time;
	    	current_time = ros::Time::now().toSec();
	    	position.time=current_time;

	    	// position info
	    	position.x=pose->position.x;
	    	position.y=pose->position.y;
	    	position.z=pose->position.z;

	    /* save the position to memory for this robot */
	    	// if the number of old position stored is too long 
	    	if (v_robot_positions_.at(robot_number).size()>=MAXTIMEWINDOW)
	    	{
	    		// delete the first element
	    		v_robot_positions_.at(robot_number).erase(v_robot_positions_.at(robot_number).begin());
	    	}
	    	v_robot_positions_.at(robot_number).push_back(position);

	    	//ss.str("");
	    	//ss<<"size of memoryvector for robot: "<<robot_number<<" is: "<<v_robot_positions_.at(robot_number).size();
	    	//ROS_INFO_STREAM(ss.str());
	}

	void calcVelocity(int robot_number)
	{
		//std::stringstream ss;
		//ss<<"calculating velocity for robot: "<<robot_number;
		//ROS_INFO_STREAM(ss.str());
		std::vector<robot_position> v_position = v_robot_positions_.at(robot_number);
		// create a zero  velocity structure
		robot_speed zero_speed;
		zero_speed.x=0;
		zero_speed.y=0;
		zero_speed.z=0;

		while (v_robot_speeds_.size()<v_all_robots_.size())
		{	
			// this loop ensures the vector is large enough
			v_robot_speeds_.push_back(zero_speed);
		}

		robot_position cp; //current position
		robot_position op; //old position

		if (v_position.size()<=TIMEWINDOW) // if we cant go back the full timewindow
		{
			// calculate speed over a smaller time window
			cp = v_position.back(); // current position
			op = v_position.front();
		}
		else
		{
			// calculate speed over the full timewindow
			cp=v_position.back();
			op=v_position.at(v_position.size()-TIMEWINDOW-1);			
		}

		// x speed
		v_robot_speeds_.at(robot_number).x=
			(cp.x-op.x)/
			(cp.time-op.time);

		// y speed
		v_robot_speeds_.at(robot_number).y=
			(cp.y-op.y)/
			(cp.time-op.time);

		// z speed
		v_robot_speeds_.at(robot_number).z=
			(cp.z-op.z)/
			(cp.time-op.time);
	}

	void sender(const geometry_msgs::Pose::ConstPtr & pose, int robot_number)//std::string name)
	{
		/* 
		The sender gets info from the master divisor.
		The info describes which robot is where
		every message is one robot with its current state.

		Right now this function sens all info of all robots to all robots.
		later one might decide to not send all info to all robots
		*/

		//ROS_DEBUG_STREAM("sender function in divisor is running");

		/* save the info in a message */
			ram_vo::robot_state state; // iniate a robot state message
			state.robot_number = robot_number; // set the name of the robot
			state.robot_name = v_all_robots_.at(robot_number); // set its name

		/* set the pose: */
			// position
			// note to myself: pose->position is the same as (*pose).position
			state.position = pose->position; // set the position

			// velocity
			state.velocity.x = v_robot_speeds_.at(robot_number).x;
			state.velocity.y = v_robot_speeds_.at(robot_number).y;
			state.velocity.z = v_robot_speeds_.at(robot_number).z;

			//ROS_DEBUG("starting sending messages to all bots now");
		/* publish the message to all known robots */
			for (int ii=0; ii<v_all_robots_.size();ii++)
			{
				publishers.at(ii).publish(state);	
			}
			ros::spinOnce();			
	}

};



int main(int argc, char **argv)  // we need argc and argv to pass to ross so that it can remap later
{
	ros::init(argc, argv, "master_divisor"); // create a rosnode first, now we can use other ros commands

	/* set verbosity level to debug */
		/*
		if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug) )
			{
		   		ros::console::notifyLoggerLevelsChanged();
			}
		*/

	ROS_INFO("initializing master divisor...");
	MasterDivisor master_divisor;
	ros::spin();
	ROS_INFO("master divisor stopped");
	

	return 0;
}