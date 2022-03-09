# MoveIt Pick and Place - Python version

![gen3_lite rviz demo](./demo.gif)

## Structure of package
```sh
.
├── CMakeLists.txt
├── package.xml
├── launch
│   ├── run_pick_place.launch
│   └── spawn_gen3_lite.launch
└── scripts
    └── main.py
```

## Installation
 - Package depends on [ros_kortex](https://github.com/Kinovarobotics/ros_kortex) and was tested on `melodic-devel` version of it. 

 - Thus install `ros_kortex melodic` by following instructions in the repository.

 - Add this package to your `src` folder with:

```sh
git clone https://github.com/dabarov/moveit-pick-place-python.git
```

 - And build the workspace with:

```sh
catkin_make
```

## Usage

1. To start simulation run:
```sh
roslaunch pick_place_python spawn_gen3_lite.launch
```

2. To start pick and place control run:
```sh
roslaunch pick_place_python run_pick_place.launch
```