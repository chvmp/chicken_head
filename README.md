# chicken_head
Chicken head demo using a 5DOF manipulator CHAMP Quadruped Robot. 
![CHAMP Chicken Head](https://raw.githubusercontent.com/chvmp/chicken_head/master/docs/images/chickenhead.gif)

## Installation

1. Clone and install all dependencies:

        sudo apt install -y python-rosdep
        cd <your_ws>/src
        git clone https://github.com/chvmp/chicken_head
        cd ..
        rosdep install --from-paths src --ignore-src -r -y

2. Build your workspace:

        cd <your_ws>
        catkin_make
        source <your_ws/>/devel/setup.bash

## Quick Start

1. Run the demo:

        roslaunch chicken_head demo.launch 

* This will launch CHAMP's ROS [driver](https://github.com/chvmp/champ), [champ_teleop](https://github.com/chvmp/champ_teleop), and the chicken head demo itself.
