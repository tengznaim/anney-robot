# ANNEY Robot

**NOTE:** This is an archive branch for ROS Noetic project that does not fully function. However, if using Python 3 modules and Ubuntu 20.04, this may be the better distribution to use.

These scripts are for the model development and inferencing scripts used for a waiter robot named ANNEY built for the WID3005 Intelligent Robotics course.

The scope of the robot are as follows:

- Able to perform object detection on 5 Malaysian food (nasi lemak, nasi goreng, mee goreng, roti canai and satay)
- Able to understand customers and take their orders using speech recognition.
- Able to deliver food to the correct table in a restaurant setting using SLAM.

### Running the Project

1. Ensure that you have ROS installed in the system, either locally or in the bot.
2. cd into the `anney_ros` directory and build the project using
   ```
   catkin_make
   ```
3. Start roscore and run the launch all the nodes simultaneously using
   ```
   roslaunch anney_pkg launch_anney.launch
   ```
