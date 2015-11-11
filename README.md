# Obstacle Avoidance and Contact

### Project 2 of Cornell CS 4752
###### By Isaac Qureshi, Chris Silvia, Zach Vinegar



Extras:
* RRT smoothing
* Bi-directional RRT
* Splines


#### Typewriter (`typewriter.py`)
	* Can load any font and draw any character
	* `$ roslaunch cs4752_proj2 typewriter.launch`
	* Go to http://localhost:8000/typewriter.html in a browser
#### Mouse Draw (`MouseDraw.py`)
	* `$ roslaunch cs4752_proj2 mousedraw.launch`
	* Baxter draws what you draw with your mouse in realtime
#### Image Draw (`MouseDraw.py`)
	* `$ roslaunch cs4752_proj2 mousedraw.launch`
	* Loads in an image
	* Uses a canny cv2 filter to extract edges
	* Creates paths from the edges and sends them to Baxter
#### Force PID Controller (`joint_action_server.py`)
	* Keeps the pen in contact with the drawing plane
	* Projects position error into the drawing plane for 2D position PID control
