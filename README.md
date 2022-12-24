# Quadcopter Control
Language: C++ <br /><br />
Quadcopter control implements the control system illustrated below for quadcopters while in the states of hovering and also heading to a target desitnation. 
UserCode.cpp contains the code for motor force calculations, estimator and feedback control for attitude and position of the quadcopter. 
UtilityFunctions.cpp contains the functions utilized by UserCode including conversion from desired speed to PWM commands for the motors. <br /><br />
![image](https://user-images.githubusercontent.com/59189831/209451379-a1138b2b-325b-46ec-abb5-6b8484bcc828.png) <br /><br />
Using the accelerometer and gyroscope measurements collected from the quadcopter, attitude estimators are designed for roll, pitch and yaw angles of the UAV while also taking account of bias and errors in measurements. With 3D attitude estimation and attitude control with a motor mixer, rates control and angle control are also implemented. Moreover, estimators for horizontal and vertical states are included which are used for state position closed-loop controls to stabilize the quadcopter through flight and also in hover. Overall, with this feedback control strategy, the quadcopter can take off, maintain a desired height and then land softly at a target location. <br /><br />
Simulation: <br />
https://user-images.githubusercontent.com/59189831/209451621-e7d9b636-2fd7-4c20-b0bd-41adfaaad2c4.mp4 <br /> <br />
![image](https://user-images.githubusercontent.com/59189831/209452580-0a87460e-eed5-4963-8221-5500c445d225.png)



