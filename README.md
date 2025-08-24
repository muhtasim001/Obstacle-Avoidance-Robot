# Turtlebot-Navigation-Demo

## About 

the goal of this project, was to write, play and experiment with commonly used algorithm and concepts used to robotics.  
from scratch the following were implemented : 
1. a local cost map 
2. global cost map with memory 
3. a basic path planning algorithm for static obsticle avoidance (A*)
4. the pure persuit controller 

## pre demo check list 

before you can run the demo, make sure you have the following installed: 
1. docker engine
2. chrome (only browser supported by foxglove)

on linux systems you can simply run the script in your terminal. However, if you are on windows you will need to use wsl.

## to run the demo
1. run the following command in the root directory ```./watod run```
2. look for the foxglove port and copy it down 
3. go to [foxglove](https://app.foxglove.dev/thom/dashboard)
4. open a conection to that port from step 2
5. now you can play around and make the robot move with teleop or placing points
