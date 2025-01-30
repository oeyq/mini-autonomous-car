# Mini Autonomous Car (ROS2 + Docker)

## Overview

This project focuses on developing an autonomous navigation system using a 1:10 scale model car, powered by ROS2 and Gazebo simulation. Developed as part of a university project.
It includes features such as sensor fusion, path planning, and real-world deployment.

## Setup

#### 1. Prerequisites

To run this project, the following tools are needed:

    •Docker Engine
    •Visual Studio Code
    •VS Code Extensions:
        •Docker
        •Dev Containers

## Start the Container with Visual Studio Code

In Visual Studio Code, install the **Docker** and **Dev Containers** extensions.

Build and open the container.

## Start the Container with Docker

Run the following command in a terminal that is in the current folder's directory:

> docker build -t ros2-gazebo .devcontainer/

Start the container using the following command:

> docker run --network host -it -v .:/home/vscode/workspace ros2-gazebo
