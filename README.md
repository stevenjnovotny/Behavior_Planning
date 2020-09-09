# Self Driving Car Path Planning

### Goals
The goals of this project were to safely navigate around a virtual highway with other traffic that is driving +-10 MPH of the 50 MPH speed limit. The AI is provided the car's localization and sensor fusion data, as well as a sparse map list of waypoints around the highway. The car will drive as close as possible to the 50 MPH speed limit, which requires passing slower traffic when possible. Other cars will try to change lanes too. The car avoids hitting other cars (an infinite cost) and drives inside of the marked road lanes at all times, unless going from one lane to another. The goal was to complete a loop around the 6946m highway in roughly 5+ minutes. The car was also limited to total acceleration of 10 m/s^2 and jerk that is less than 50 m/s^3.

[//]: # (Image References)

[image1]: ./behavior1.png "Complete Track"
[image2]: ./behavior2.png "Follow Path"
[image3]: ./behavior3.png "Maintain Spacing and Speed"
[image4]: ./behavior4.png "Change Lanes"

### Overall Scheme

#### Finite State Machine

The algorithm used three conceptual states:  
- Stay in Lane
- Prepare to Change Lanes
- Change Lanes

These states (and transitions between them) were based on evaluating the sensor fusion data. By cycling through the car objects, the AI constructs the following two items that were used in decision making:

["lanes_blocked"] A three element vector containing boolean values for whether or not the lane is blocked. The blocking is judged for current lane and adjacent lanes. The current lane is blocked if the predicted trajectories (of our car and the car  immediately in front) intersect or are within a particle threshold (e.g. 30 m). The side lanes are blocked if the predicted trajectories for cars ahead or behind intersect with ours, within a safe interval defined as 20 m behind or 15 m ahead.

["lane_speeds"] A three element vector containing an estimate of lane speeds. The lane speeds were defined as the speed of the closest car ahead of our car's current position.

#### Decision Making

Since the only goal was to go fast and other events had an infinite cost (hitting other cars, failing to stay in a lane, etc.) cost functions were not used. However, based on the often erratic behavior of other cars, the next iteration of this code should probably consider multiple trajectories and choose ones tha maximize distance from other vehicles.

Therefore, the decision making was based on the following:

- Is current land blocked?
    - If so, check nearby lanes
       - If nearby lane is clear AND lane's speed is greater, change lanes
      - If nearby lanes are not clear, slow down
- If not blocked, drive at maximum speed

This resulted in a target lane and speed adjustment (increase, decrease, or stay the same).

#### Trajectory Planning

Once a lane and speed change were decided upon, a spline function was used to calculate a trajectory from the current position using a combination of points from the previous trajectory and from points projected forward. Frenet coordinates were used and then converted to global map coordinates. The parameters for the trajectory were set such that the acceleration and jerk constraints were not violated.


### Results

The algorithm met all goals and requirements, as summarized below.

#### Driving the Course

The car completed the course in roughly 5-7 minutes-- depending on traffic. The main factor in lap time was how aggressive the passing parameters were set, i.e. how the algorithm calculated safe distances and acceptable gaps. The longest the program ran without an incident was over 20 minutes--which is when the simulation was terminated due to user boredom. The following shows data verifying the completion of the entire loop.

![alt text][image1]

#### Following Spline Based Trajectory (staying in a lane)

The spline curve generated always used a combination of projected points and points from the previous trajectory.  That ensured a smooth (jerk reducing) path. The following shows a visualization of the car and its trajectory.

![alt text][image2]

#### Maintain Safe Spacing 

The AI was defined that it would keep a safe distance from the car in front and would not pass unless it had a safe gap to do so. In addition, the car would not change lanes unless the target lane allowed a higher speed. The following shows an example of the car exercising these safety protocols.

![alt text][image3]

#### Passing into Faster Lanes

If the AI determined that 1) the current lane was blocked, 2) a neighboring lane was not blocked, and 3) the neighboring lane allowed higher speeds, then the car would pass. The image below shows an example of the passing trajectory.

![alt text][image4]

### Reflections

Several seemingly small features had a significant impact on the performance of the AI.

1) Evaluating lane speeds prior to lane changes: Though the performance was adequate without ever considering lane speeds, I found the car executed many inefficient lane changes and ultimately had to execute additional changes to compensate. I believe that lane changes are a high risk transition for self-driving cars and thus believe minimizing the number of changes to be beneficial. Adding the capability to evaluate the speed benefits made a significant difference.

2) Evaluating lane speeds to determine appropriate following speed: I originally commanded to car to decrease speed if it was within a certain distance to a leading car. This resulted in jerky motion as the car would continue to decelerate, reach a safe distance, accelerate again, and then repeat the whole process. I ultimately set the spped decrease to occur only if the leading car was going slower that our car. This resulted in a much smoother following motion.

3) Internal variables vs parameters derived from sensor/localization data: at first, I determined all car parameters (lane, speed, etc) from sensor data instead of maintaining global parameters from the vehicle. This surprisingly caused a great deal of problems. When I switched to maintaining global variables and updating them in a model-based way, the AI worked much better. I am still considering why this was the case. I assume it may have been latency with the simulator, but I have yet to reach a conclusion.

### Simulator.
This project uses the (Unity-based) Udacity self-driving car simulator that can be downloaded from the [releases tab (https://github.com/udacity/self-driving-car-sim/releases/tag/T3_v1.2).  

To run the simulator on Mac/Linux, first make the binary file executable with the following command:
```shell
sudo chmod u+x {simulator_file_name}
```

#### The map of the highway is in data/highway_map.txt
Each waypoint in the list contains  [x,y,s,dx,dy] values. x and y are the waypoint's map coordinate position, the s value is the distance along the road to get to that waypoint in meters, the dx and dy values define the unit normal vector pointing outward of the highway loop.

### Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./path_planning`.

Data provided from the Simulator to the AI

#### Main car's localization Data (No Noise)

["x"] The car's x position in map coordinates

["y"] The car's y position in map coordinates

["s"] The car's s position in frenet coordinates

["d"] The car's d position in frenet coordinates

["yaw"] The car's yaw angle in the map

["speed"] The car's speed in MPH

#### Previous path data given to the Planner

["previous_path_x"] The previous list of x points previously given to the simulator

["previous_path_y"] The previous list of y points previously given to the simulator

#### Previous path's end s and d values 

["end_path_s"] The previous list's last point's frenet s value

["end_path_d"] The previous list's last point's frenet d value

#### Sensor Fusion Data, a list of all other car's attributes on the same side of the road. (No Noise)

["sensor_fusion"] A 2d vector of cars and then that car's [car's unique ID, car's x position in map coordinates, car's y position in map coordinates, car's x velocity in m/s, car's y velocity in m/s, car's s position in frenet coordinates, car's d position in frenet coordinates. 


