# Self-Driving Car Nanodegree Capstone Project (Team: ThinkLikeCar)

## Overview

This repo contains the submissions and related material for Udacity "Self Driving Car" Nano degree program's Term 3 - Project 3, "System-Integration"

## Team Member

* Sulabh Matele (sulabhmatele@gmail.com)
* Clint Adams (clintonadams23@gmail.com)
* Sunil Prakash (prakashsunil@gmail.com)
* Ankit Jain (asj.ankit@gmail.com)
* Frank Xia (tyxia2004@gmail.com)

## System Architecture Diagram
Here we use ROS nodes to implement core functionality of the autonomous vehicle system, including traffic light detection, control, and waypoint following. The code was tested in a simulator, and later run on [Carla](https://medium.com/udacity/how-the-udacity-self-driving-car-works-575365270a40), the Udacity's own self-driving car. 

The following is a system architecture diagram showing the ROS nodes and topics used in the project.

![System Diagram](./imgs/system.png)

## ROS Node
### Waypoint Updater node (waypoint_updater)

The main responsibilities of this node are:
+ This node listens for the `/base_waypoints` topic sent by `Waypoint Loader` node. This topic sends all the available waypoints at the start and then never updates. 
+ This node stores all the waypoints and sends a chunk of waypoints at a time, when it receives the current vehicle position from simulator/car.
+ This node listens for `/current_pose` topic to receive car's current position. Then decides to send the required number of future waypoints to follow by publishing `final_waypoints`
+ This node also listens for `/traffic_waypoint` and `/obstacle_waypoint` topics to receive the information about RED signal and obstacle on road if any.
+ When RED light is detected by `tl_classifier` then it sends the RED light waypoint index, and then waypoint updater starts setting velocities for future waypoints to make car stop at the RED light waypoint.
+ Once the light turns GREEN, then `tl_classifier` again informs to waypoint updater. and waypoint updater sends next waypoints with required velocity to make car moving.

Different challenges in implementation of Waypoint Updater:
+ The main challenge was to efficiently detecting the nearest waypoint to the car position, this was resolved by checking the Euclidean distance between the simulator reported car position to the list of waypoints using method `euclidean_dist` and then if the distance is increasing for 25 waypoints then break the loop.
+ The next challenge was to handle the stopping and starting car on RED and GREEN :
  + When the traffic light turns RED, it sets `self.is_stop_req` so waypoint updater stats setting velocities in advance upto `self.decrement_factor` advanced waypoints. When car stops on RED, then we also stop sending new list of waypoints so, car keeps in stop position.
  + When the light turns GREEN, the request to stop `self.is_stop_req` gets clear and makes the method `pose_cb` to fill the waypoints with their velocities, which in turn moves the car.
+ The last challenge was to resolve the condition when the car is at end of the track and our list of waypoints is about to be finished. This condition is handled as an special condition and we set `self.short_of_points`, which then takes care of setting velocities to 0, near end of waypoints. 

### Twist Controller Node (dbw_node)

### Traffic light detection node (tl_detector)

### TLClassifier class (tl_classifier)





