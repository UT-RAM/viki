		/* start a node for all robots */
				pid_t pID;
				for(int ii=0;ii<v_all_robots_.size();ii++)
				{
					pID = fork();
					if(pID<0)
					{
						ROS_INFO("error making a fork");
					}
					if (pID==0) // the child process 1
					{
						// give some output to screen so that we know it is working
						std::stringstream ss;
						ss<<"starting child process for robot "<<ii;
						ROS_INFO(ss.str().c_str());

						// code that the child must execute
						std::stringstream tmpss;
						tmpss<<"rosrun ram_vo VO_controller __name:=VO_controller"<<ii<<" /test_info:=/controller"<<ii<<"/test_info";
						system(ss.str().c_str());

						break; //BREAK IS REQUIRED!!! (or else the child might continue the loop, creating more childer. don't understand why yet)
					}
					else { //parent is running
						std::stringstream ss;
						ss<<"parent pID is: "<< pID;
						ROS_INFO(ss.str().c_str());
					}

				}

				if (pID>0) // parent IPC
				{
					ROS_INFO("the parent IPC part is now being run");
				}


		/* end of: start a node for all robots*/