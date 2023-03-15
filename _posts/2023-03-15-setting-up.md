---
layout: post
title: Practice 1 - Follow-Line
subtitle: Week 1- Setting up the simulator and first contact with robotics
social-share: false
readtime: true
---

The first practice proposed for the Robotics’ Vision Course consists on the employment of a **PID reactive control** robot which is able to complete a racing circuit in the lowest time possible. For this purpose, the [Unibotics platform]( https://unibotics.org/)  will be used. 

Nonetheless, previous to the implementation of the controller, the environment needs to be set up, ensuring its proper working. In this post, we will go through this journey.

Additionally, we will perform a first processing of the images seen by the robot and try to implement a basic reactive controller, based on different scenarios. 

## The environment
Before going deep into the robot, the simulator needs to be arranged. In this sense, the first step is to create an account which gives access to all the open source exercises proposed by the [JdeRobot organization](https://jderobot.github.io/). Once logged in, we will have access to plenty of activities which will allow us to **understand robotics by practice**.

In our case, we will focus (at least by now) on the first exercise available (i.e, the follow-line) indented to program a F1 car which completes a given race circuit following a red line, as sharp and fast as possible. 

Since we are using the platform for the first time, a few more steps need to be carried before starting the Python-based programming. Following the instructions given by the developers, we found out the simulators can run in two ways: using **remote** or **local** connection.  For the first scenario, we need to ask for permission, since we will be accessing the developers’ servers. However, although this may be useful in some cases, the preferred way of running the exercise is the latter; that is, locally.

In order to be able to run the exercise locally the [installation instructions]( https://unibotics.org/academy/exercise/follow_line/) were followed. First step was to install [Docker](https://docs.docker.com/get-docker/). Since we will be using a Windows 11 computer, Windows Subsystem for Linux (i.e, WSL-2) was also installed to be able to run executable documents developed natively in Linux. With that done, the [latest JdeRobot docker image]( https://hub.docker.com/u/jderobot/) was installed using the command window through the query:
~~~
docker pull jderobot/robotics-academy:latest)
~~~
With that being properly install we can either enable GPU acceleration or start to deal with the exercise. In my case, I decided to get hands on with the exercise.
Last step prior to code-typing is to start docker container and leave it running. To do so, we will open the Docker-Desktop app and execute the following line on the command window:
~~~
docker run --rm -it -p 8000:8000 -p 2303:2303 -p 1905:1905 -p 8765:8765 -p 6080:6080 -p 1108:1108 jderobot/robotics-academy
~~~
With that running, we will enter Unibotics platform, select the Local backend option on the dropdown menu and make sure connection is established. With that, and after clicking the Launch button, we are ready to play!

## Processing 

As mentioned, the goal of this first practice is to program a robot based on a PID controller which can follow a line to complete a race circuit. The first challenge we need to approach will therefore be the **line detector**.

To familiarize with the image seen by the robot, the first step is to display it on the screen. Doing this, we realize the line to chase possess a pure red color, which is not seen on other objects of the circuit. Taking this in consideration, the easiest way to track the line will be to implement a **color filter**, which, in order to make it more robust to illumination changes, will be implemented in the HSV color space. After some research, it was discovered that red color in the RGB scale corresponds to the (0,100,100) value in the HSV space; therefore, a range around this value was set to segment the line, obtaining a binary mask similar to the one shown below this text. In order not to lose information related to circuit and for better visualization purposes, the obtained mask was blended with the image seen by robot.

![Line Detection](https://github.com/brodgon/brodgon.github.io/blob/master/docs/fig%20.png?raw=true)

With the line detected, we want to determine its centroid to be able to estimate the position where the car should be. The approach to do this is pretty simple. Since the red line always fall around the bottom-half of the image, we will select a row around it and, based on it, we will count the number of pixels until we find the first white pixel on the mask (starting from right and left mask borders). With this information, we will just need to calculate a simple mean in order to get the center of the line and plot it with a bright color!

![Centroid](https://github.com/brodgon/brodgon.github.io/blob/master/docs/centroid.png?raw=true)

After doing this, we have useful information to start dealing with the intelligence of our robot!

## Scenario-based controller
Although the main goal is to implement a PID controller, I found interesting to create a simple robot based on scenarios. To do this, the first step was to model an action-reaction table based on the information perceived by the robot.

Since we have modeled the number of pixels from left and right till we find the line, we will compute a ratio with it, precisely:
$$ratio = \frac{number \quad of \quad pixels \quad located \quad right \quad to \quad line}{number \quad of \quad pixels \quad located \quad left \quad to \quad line}$$

Depending on the value of this ratio we can model if we have more or less pixels on one side or the other and, therefore, make a decision. Specifically:
- If $ratio < 1$ the car will be on the left side of the image, that is, there are more pixels on the left than on the right side. In order to correct this, we want to turn right in order to approach the line.
- If $ratio > 1$ the car will be on the right side of the image, that is, there are more pixels on the right side than on the left one. Same as before, since we want to get closer to the line, we will turn left.
- If $ratio=1$ it means we have the same pixels right and left, meaning we are centered. This will be the preferred state.


Having this idea in mind, we can think further; precisely, we can consider the actual value of the ratio. If this value is really small or really big (with respect to one) it will mean we are closer to the walls of the circuit, and therefore, the shift needs to be more drastic. On the other hand, the closest to one the ratio is, the smaller the turn. In order to model the shifts, we will use linear and angular velocity of the robot. 

With this information, we modeled 7 possible scenarios depending on different ratio ranges. Scenarios were modeled based on trial and error:

| Ratio | Linear Velocity | Angular Velocity |
| :------ |:--- | :--- |
| (- $\infty$,0.1] | 1.5 | -0.9 |
| (0.1, 0.7] |3 | -0.5|
| (0.7,0.9] | 9 | -0.1 |
| (0.9,1.2] | 12 | 0 | 
| (1.2, 2.1] | 9 | 0.1 |
| (2.1, 2.5] | 3 | 0.5 |
| (2.5, $\infty$) | 1.5 | 0.9 | 

With this, we are able to finish a complete lap in **2:56 min**. However, the behavior of the robot is not perfect. The robot is continuously shaking, shifting drastically in some cases, especially, when the ratio is close to the frontiers settled. Additionally, the robot could not be maintained on the center, it is always shifted to one of the sides. All of this results in a slow robot.

All these drawbacks will be solved when implementing a PID controller. 

There is a lot of work to do!! I will keep you updated! 

See you soon!
