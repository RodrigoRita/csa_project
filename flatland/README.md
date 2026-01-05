# Flatland 2

This is a fork of [Flatland](https://github.com/avidbots/flatland) modified to
work on [ROS 2](https://docs.ros.org/en/humble/The-ROS2-Project.html) natively
(not using [ROS 1 bridge](https://github.com/ros2/ros1_bridge)). The development
targeted [ROS 2 Humble](https://docs.ros.org/en/humble/The-ROS2-Project.html),
which was the latest release of
[ROS 2](https://docs.ros.org/en/humble/The-ROS2-Project.html) at the time of
writing.

Hopefully the changes in this repository will be merged to the original Flatland
repository.

## What is Flatland?

- Flatland is a performance centric 2D robot simulator
- It is useful for simple simulations that don't require 3D capabilities
- It allows for time acceleration, which is useful for reinforcement learning

## How do I get set up?

1. Install ROS 2 Humble following the
   [official installation guide](https://docs.ros.org/en/humble/Installation.html)
2. Install [git](https://git-scm.com)
3. `git clone` this repository into your project's workspace's `src` folder
4. Install the required dependencies using
   `rosdep install -i --from-path src --rosdistro humble -y`
5. `colcon build`
6. `source install/setup.bash`

## Do you have any working example?

### Official turtlebot_flatland example

You can check our fork of the
[official turtlebot_flatland example](https://github.com/JoaoCostaIFG/turtlebot_flatland).
We made this work on
[ROS 2](https://docs.ros.org/en/humble/The-ROS2-Project.html) natively as well.

1. `git clone https://github.com/JoaoCostaIFG/turtlebot_flatland.git` into your
   workspace's `src` directory
2. Install the required dependencies using
   `rosdep install -i --from-path src --rosdistro humble -y`
3. `colcon build`
4. `source install/setup.bash`
5. `ros2 launch turtlebot_flatland turtlebot_in_flatland.launch.py`

### Our wall-following reactive

We developed a
[wall-following reactive robot for ROS 1](https://github.com/JoaoCostaIFG/ri/tree/master/Proj).
We then converted the code to ROS 2, so it could be used as an example ROS 2
Flatland project.

1. `git clone -b ros2 https://github.com/JoaoCostaIFG/ri.git` into your
   workspace's `src` directory
2. Install the required dependencies using
   `rosdep install -i --from-path src --rosdistro humble -y`
3. `colcon build`
4. `source install/setup.bash`
5. `ros2 launch c_turtle c_turtle.launch.py`

## Who do I talk to?

You can either direct questions to us by creating issues. Although he isn't
directly associated with this fork, if needed, you can try to contact the
original Flatland creator @josephduchesne.

## Documentation

Currently, there isn't any new documentation for this fork of Flatland. It
should be safe to defer to the original Flatland's documentation:

- How to use: http://flatland-simulator.readthedocs.io
- Doxygen: http://flatland-simulator-api.readthedocs.io

## License

All Flatland code is BSD 3-clause licensed (see LICENSE for details)

Flatland uses a number of open source libraries that it includes in its source
tree:

- [ThreadPool](https://github.com/progschj/ThreadPool) Copyright (c) 2012 Jakob
  Progsch, Václav Zeman (zlib license)
- [Tweeny](https://github.com/mobius3/tweeny) Copyright (c) 2016 Leonardo
  Guilherme de Freitas (MIT license)
- [Box2d](https://github.com/erincatto/Box2D) Copyright (c) 2006-2017 Erin Catto
  [http://www.box2d.org](http://www.box2d.org) (zlib license)
