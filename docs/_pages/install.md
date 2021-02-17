---
permalink: /install/

title: "Installation and use"

sidebar:
  nav: "docs"
---

## Installation

JdeRobot Robotics Academy

(a)ROSNode templates (= RoboticsAcademy 2.1). Here a native installation of ROS and Gazebo is recommended, but then the exercise is not edited in any webpage (such as the one of your first image), but programming your solution in the MyAlgorithm.py file, which will be executed from the ROSNode template of the exercise.

(b) WebTemplates (= RoboticsAcademy 2.3). Here the typical execution recommends the installation of the RoboticsAcademy Docker Image (RADI) as backend. It already has everything pre-installed. And the user will edit her source code at the webpage you shown on your first image

(c) RoboticsAcademy webserver (=unibotics.org, online RoboticsAcademy). Here you may also install RADI on your machine (=c1, local backend) or if you are a URJC student no installation at all will be required (the backend will run remotely on one URJC computer) (=c2, remote backend). In this usage, the users also use the browser to edit their source code.


2.0 ROSnodes and assets in JdeRobot-assets and JdeRobot-base

2.1 ROSnodes and assets in CustomRobots, Drones, IndustrialRobotics.

23 exercises are in these two versions 

2.3 WebTemplates and RADI, es RoboticsAcademy offline 
 2.3 we have Unibotics, que es RoboticsAcademy online
 always with RADI(Robotics Academy Docker Image) as backend, both local mode and remote mode.


Unibotics http://unibotics.org:8000/academy/

Follow Line Exercise WebTemplates & RosNode Templates https://jderobot.github.io/RoboticsAcademy/exercises/AutonomousCars/follow_line/


ROSnode https://jderobot.github.io/RoboticsAcademy/installation/#generic-infrastructure



'Inciencia, Rama, Parche'
Issue, Branch, Pull Request
1º git pull  & git checkout master ;  to know you are in the master branch and it is updated

2º Create Issue in your github repository and then you have a issue number  #X

3º Create issue-X branch in local  git checkout -b issue-X

4º git status to know in what branch you are an the files that are changed 

5º git add <file>  with all the files you want to upload
  
6º when you add all files :  git commit -m "Message with your changes"

7º Upload your local branch to the github repository: git push origin issue-X

8º Once you upload your changes in the issue-x branch. Do a Pull request in Github.


