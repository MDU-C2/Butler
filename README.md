# Butler

Butler MDH@home

## Getting Started

Clone recusively in order to sync submodules, such as

```
git clone --recursive https://github.com/notlochness/Butler.git 
```

Note that darknet\_ros automatically detects CUDA if existent and wants to build with CUDA support, see the CMakeLists.txt in darknet\_ros in order to cofigure the correct CUDA version.

Weights for darknet\_ros will be avaliable on Teams and should be placed in `darknet_ros/darknet_ros/yolo_network_config/weights`

Its also encouraged to build at least darknet\_ros with a `-DCMAKE_BUILD_TYPE=Release` flag in order to increase performance.


Please read through the coding style guides we are using

* [CppStyleGuide](http://wiki.ros.org/CppStyleGuide) - ROS Cpp Style Guide
* [PyStyleGuide](http://wiki.ros.org/PyStyleGuide) - ROS Python Style Guide

## Dependencies

## How to Git

Please refer to this cheat sheet before doing anything.

* [GitCheatSheet](https://services.github.com/on-demand/downloads/github-git-cheat-sheet.pdf) - Git Cheat Sheet

Open a command window and run:

```
man git
```

To access the git manual.

And check out the Git Mannerism page under InfoWiki on our team site.

### Clone the repository

```
cd ~/catkin_ws/src

git clone https://github.com/notlochness/BUTLER.git
```

### Checkout the branch you want and make a local one

```
git branch --list

git checkout <branch>

git branch <new-branch>
```

### When your code is stable merge the branches

```
git checkout <branch>

git pull

git merge <local-branch>
```
