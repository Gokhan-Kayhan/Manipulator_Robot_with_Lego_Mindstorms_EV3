# Manipulator Robot with Lego Mindstorms EV3

## About the Project

This project was prepared as the practical assignment of **"Mechatronic Systems"** course during my master study in University of Siegen. The _"Lego Mindstorms Robot Arm"_ first was studied and then programmed to perform a set of pick-and-place tasks.

<img src="figures/2.gif"  />  


There are 3 different stations and in each task, the ball should be picked up and placed in another station. We started by measuring the physical properties of the robot and sketched its section view.  

The sonic sensor on top of gripper, alwasys calculate the distance between gripper and ball. But this data can not be used directly becase the height of the stations changed during the tasks.

Therefore, inverse kinematic was used and the angle between robot arms is expressed in terms of distance between gripper and ball. In this way a dynamic system which can adapt the differences at the environment was obtained.


[//]: # (<img src="figures/Picture 1.png"/>)

[(<img src="figures/Picture 1.png"/>)]: # 

<img src="figures/3.gif"  />  

---

**Authors**  
Elchin Ismatli @ University of Siegen  
Alireza Yazdani @ University of Siegen  
Dinh Tan Nguyen @ University of Siegen  
Gökhan Kayhan @ University of Siegen  
