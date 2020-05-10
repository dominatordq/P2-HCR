First, source:

-source /opt/ros/melodic/setup.bash
-source ~/Stingray-Simulation/catkin_ws/devel/setup.bash
-source ~/Stingray-Simulation/stingray_setup.bash

To run launch file:

-roslaunch stingray_sim wall_following_v1.launch

-Once the program is launched in Gazebo, open up the terminal you just used roslaunch in, and type in the number '1'. This will train the robot! Additionally, if you type in the number '0' in the terminal at first, the program will error because there isn't a file with the best q table yet. The robot must first be trained, which is why typing 1 first is critical. After the robot trains, you can rerun the program and type '0', which should follow the trained path.

Additionally, you can rosrun my source file:

-Navigate to stingray_sim's src folder if you didn't source, then:
	-rosrun stingray_sim wall_follow.py

Other Comments:

Unfortunately, my screen recorder I use on my Ubuntu machine is really slow and extensive. I wasn't able to get a full video of the robot training itself, since my free space runs out and I have no way to clear anything else up. The saved videos I took are very choppy, and there's no way for me to fix that since the screen recording software takes up almost all my RAM. I hope the few videos I sent are good enough, and feel free to train the robot yourself, which may take a while unfortunately.
