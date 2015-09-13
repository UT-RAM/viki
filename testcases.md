VIKI test cases
===============
This is a collections of quick sort-of test that we would very much like you to run. Tell us how it went using this e-mail adress: a.kamphuis-1@student.utwente.nl

Test case 1: Read some stuff on VIKI
------------------------------------

### Requirements 
* Internet connection/browser
* RAM account

### Do
* Go to the VIKI documentation page
* Find out about what VIKI is and where you can find more info
* Lookup the page on installation
* Lookup the page on module writing
* Lookup the quickstart guide

### Feedback
* Let us know if you had any trouble

Test case 2: Install and run VIKI
---------------------------------

### Requirements
* Ubuntu
* ROS
* Python
* Access to repository (RAM account)
* Internet connection

### Do
1. `CD` to the folder you want to install VIKI. We call it *VIKIfolder/* here
1. Pull VIKI using `hg pull http://insertVIKIrepositoryhere`
1. Pull the RAM moduleslist fro VIKI using `hg pull http://instertmodulelistrepohere`
2. Run the installscript by `python installVIKI.py`
3. accept if needed
4. run VIKI using `python VIKI.py`

### Feedback

* Did you manage using the above instructions?
..* if **Yes** please let us know by e-mail. Do you have any suggestions?
..* if **No**:
....* What error's did you get?
....* Did you manage to fix the problem yourself? and how?
....* If not, let us know what ROS/Python version you are running
....* Do you have any suggestions for improvements?

Test case 3: simple turtle setup
--------------------------------

### Requirements
* Working VIKI

### Do
1. Run VIKI
2. Find the turtleSim modules by searching for the keyword `turtle`
3. Drag an instance of the turtle teleop key to the canvas
4. Drag an instance of the turtle sim to the canvas
5. Connect them
6. Run
A new terminal should open, running the launch file
7. Verify that the two nodes are working by using your arrow keys in the newly opened terminal
8. Open a terminal yourself and type rostopic `rqt_graph rqt_graph` to see what VIKI is running for you
9. Exit by hitting ctrl+c in the newly opened terminals. VIKI should remain running

### Feedback
* Did you manage using the above instructions
* If not, did you manage to fix the problem yourself? how?
* What error messages dit you get?

Test case 4: More complex turtlesim
-----------------------------------

### Requirements
* Working VIKI

### Do
1. run VIKi
2. Add two turtle teleop module, and 2 turtlesim modules
3. Connect 1 teleop module to both turtlesims
4. connect the other teleopmodule to one turtlesim
5. Make the teleop modules open in a new terminal
6. Run
7. Make sure that one teleop connects to both turtles, and the other to only one.

### Feedback
* Did you manage using the above instructions
* If not, did you manage to fix the problem yourself? how?
* What error messages dit you get?

Test case 5: your own module
----------------------------

### requirements
* Working VIKI
* ROS knowledge

### Do
1. Create a rosnode that listens to two command velocity topics (topic type twist)
2. Make the node publish one twist message
3. Use the linear part of one message, and the rotation from the other incoming messages, and combine them in the new message to be published
4. Create a new module using the tutorial, for your rosnode
5. Set up viki using 2 turtle teleops, and one turtlesim, and see if your node/module work as expected

### Feedback
* Send us the node and modulefile you wrote
* Did you manage using the above instructions
* If not, did you manage to fix the problem yourself? how?
* What error messages dit you get?