# a-star-algorithm-point-robot
This program is an implmentation of A-star algorithm for a mobile robot of radius with 5 action spaces. 

Navigation of a point robot from start to goal point using Dijkstra's algorithm in a known environment with defined clearance to avoid collisions

## Dependencies

-   Python
-   Opencv 4.1.0
-   Numpy
-   queue
-   collections

## Steps to run the package
1.Clone the project 

    git clone https://github.com/Madhunc5229/a-star-algorithm-point-robot

2.cd into the code folder and the run the python file
    
    cd/a-star-algorithm-point-robot/code  
    python3 sourceCode.py

## Action space of the robot

![graph](https://user-images.githubusercontent.com/61328094/159148038-378d89f5-2444-4b4e-bcb0-99fa97043217.png)

## Obstacle Space Layout

<img width="474" alt="layout" src="https://user-images.githubusercontent.com/61328094/159148083-d94bad84-0b68-4e9d-af2e-ec14c31e0389.png">

## A-star implementation (plottting the explored points)
### Explored nodes = green color
### robot radius + clearance = red color
### Obstalce points = cyan color
### pink line is the backtracked solution path

![astarc2c](https://user-images.githubusercontent.com/61328094/159148128-5355d0c0-4c10-43bd-857c-393ea9d6167a.png)





    
