# ROS2

## What is ROS?

Our main framework we use to process real world data and incorporate our Algorithms into our System is ROS2. It is an open source robotics middleware that processes Data using a publisher/subscriber model based on topics. Additionally you are able to configure services and parameters, which are not too important to us. 

In practice, you operate with ROS within a ROS workspace, which is essentially just a directory containing ROS packages. ROS packages represent a gathering of source code and building instructions (e.g. necessities for compiling), and usually hold a specific purpose (e.g. implementing a specific planning algorithm). - You will learn more about this later on.

## Why ROS?

The node and publisher/subscriber architecture makes communication between processes and algorithms very easy and most importantly modular. Because of the wide spread community most companies offer out of the box ROS-drivers for their Sensors, which plays into less headache connecting perception. Due to the inbuilt CANopen API we also provide a interface to common use Sensors such as velocity and acceleration but also can simultaneously publish to Vehicle Control.

## Exercise

In this section you will learn by doing. So no big explanations beforehand and we will dive right into experimenting and learning. 

!!! info
    These Tutorials are heavily inspired by the [ROS2 Docs](https://docs.ros.org/en/humble/index.html). For any questions you can feel free to take a look at the topics covered there.

## What if I want to learn more about ROS2?

These Tutorials just give a short overview of ROS2 and its concepts - they resemble the [Beginner: CLI tools](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools.html)-Section from the official Documentation. If you want to dive in deeper into how you can build your own applications, similar to what you will see in the following pages we recommend the [Beginner: Client Libraries](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries.html). The `Driverless Onboarding` git repository you will be using is also fit to running these tutorials.