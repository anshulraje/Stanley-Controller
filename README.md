# Stanley
## Video Demonstration
![Stanley Video Reduced ‐ Made with Clipchamp](https://user-images.githubusercontent.com/65621792/151651630-61aa2268-a145-4864-91ea-e507ce4cb40d.gif) <br />
> Turtlebot moves in a smooth sine curve

## Introduction
> The Stanley Controller was developed by **Stanford University** to win the **DARPA (Defense Advanced Research Projects Agency) Grand Challenge 2005**. 
> It corrects the heading error of a rover while moving to provide the required turning angle using a *closed feedback loop*.

## The Need for Stanley Controller
Traditionally, for a rover to move, it would require to move upto a certain point on a global frame, then stop there and turn again to face to the next point and move up to that point. While this methodology works, it's extremely time consuming and doesn't look very nice. That's where the Stanley Controller comes into picture. 

If we were simply to try turning in the direction of the goal while moving straight, we might overshoot the goal or undershoot it since we might not turn enough as we are constantly moving and the required turning angle is calculated with respect to a fixed position. Therefore, we need to identify the extra heading anglee that needs to be turned to face the next point.

## The Mathematics
![My Beautifully Drawn Stanley Explanation](https://github.com/CocaKhosla/Stanley/blob/images/StanleyImage.jpg?raw=true)

The current trajectory has a _minimum lateral error_ with the desired trajectory called the _cross track error = e(t)_. Cross track error is calculated by first finding slope of the _desired trajectory_. Then slope of the error = ```-1/slope```. Therefore, with the slope and our given point, we can get a line that passes through our _current trajectory_ and is _perpendicular_ to the _desired trajectory_. Through the _intersection_ of this line with the _desired trajectory_, we get the point of minimum distance. Therefore, _cross track error_ is simply the _distance_ between these two points.

Now that we know _e(t)_, we can calculate the rest using the following formulae. We know the direction of the velocity is our current direction of trajectory. Therefore, _delta_ can be calculated as follows

![Formula](https://github.com/CocaKhosla/Stanley/blob/images/CodeCogsEqn.png?raw=true)

Here, _k_ is the proportional gain which depends on the vehicle's dimensions and is subject to testing. In the case of the simulations of turtlebot3 waffle pi and burger, there is **0** heading error. A smoothening constant _c_ is applied to prevent division by zero.

(If not using these simulations as provided above, one must calculate a heading error)


## How To Use
__*IF YOU'RE USING ROS NOETIC*__
1. Clone the repository in your desired ROS package. 
2. Go into the folder Stanley and make the file ```StanleySim-Working-Demo.py``` and ```live_plotter.py``` executable using the command: 
```
chmod +x StanleySim-Working-Demo.py
chmod +x live_plotter.py
``` 
3. Finally, after getting ```roscore``` running and gazebo turned on, run the two files in two different terminal tabs
```
rosrun <your_ROS_package> StanleySim-Working-Demo.py
rosrun <your_ROS_package> live_plotter.py
```
4. Voilà. The default path in the code is a _sine curve_. You can change that yourself to whatever you like.
