# Controlling-Ground-moving-robot-by-using-Camera-put-on-top-of-the-room
## 1. Introduction
# * 1.1 Definition

The project embraces a list of essential concepts applied by our team while creating a program in Python, OpenCV library. When images are analyzed and modified using     
computational algorithms, this process is known as “image processing”.  In particular, it entails the robot’s detection and recognition of colored circles placed on a cardboard surface. Next, the term used to control the robot’s movement, named “control algorithm” refers to the set of instructions or logic. The analysis of them forms the basis of the movement. This algorithm computes the robot’s trajectory along a predefined path to guarantee that the robot moves in the desired path. Along with this, it retains the stability. Futhermore, the central position, or coordinates of the detected colored circles on the cardboard is indicated by the term “center point”. The term “path” refers to the established track that gives directions to the robot to follow. The motors are the mechanical components that drive the robot’s movement, and signals are sent to them to control the speeds of both left and right motors. Last but not least, “Bluetooth” permits wireless communication between the motors and the robot’s control system, which is referred to as “embot”. A key part of this project’s image processing workflow is the study of OpenCV library’s capability to read images straight from the web camera. Thus, it’s also vital to distinguish the colors’ centers individually.

# * 1.2 Purpose

When cardboard is placed on a robot that is standing in any direction, it turns white and allows for the application of two different colored cirles - one red and the other blue.
After an image is taken from above by the web camera and sent to the computer (as though it were connected to the web camera), image processing takes place. Prior to finding their common point, we must first identify two colored circles. To be processed along the path that has been given to us. We encounter an error if the robot moves in both directions. The next step is to determine the appropriate speeds for the left and right motors based on the desired path, and control algorithm. Subsequently, we transmit it to the robot, which, assuming it is equipped with Bluetooth, transmits those signals to the motors. The robot’s only function in this situation is to use Bluetooth to obtain the motors’ speed data.                                                                                                                        





