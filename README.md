# warmup_project

## drive_square
### Description 
This node will cause the robot to drive forward for five seconds and then make a 90 degree turn. It will keep repeating this, effectively driving in a square path.

### Code Explanation
* `DriveSquare`: The pythonic class that is responsible for the **drive_square** behavior.
    * `DriveSquare.__init__()`: constructor for a `DriveSquare`. In addition to creating the object, this function initializes the ROS node, a `Twist` message for the node to use, and the `Twist` publisher. 
    * `DriveSquare.run()`: This starts the robot's movement. The function contains a loop which will cause it to move forward for five seconds and then turn 90 degrees to the left. This loops indefinitely, so the robot will continue to drive in a square-like fashion. This will also print out the time to the console every time the robot starts moving or turning.

![Drive Square](/gifs/drive_square.gif)

## Person Follower
### Description 
This node will cause the robot orient itself towards the nearest object ('person') and move towards it. The object may be moved around and the robot will automatically adjust. It will stop about 0.2m in front of the object.

### Code Explanation
* `PersonFollower`: The pythonic class that is responsible for the **person_follower** behavior.
    * `PersonFollower.__init__()`: constructor for a `PersonFollower`. In addition to creating the object, this function initializes the ROS node, a `Twist` message for the node to use, the `Twist` publisher, a `LaserScan` subscriber, and several constants necessary for proportional control. 
    * `PersonFollower.follow_person()`: This is the meat of the robot. Here, the code loops through the `ranges` array in the `LaserScan` message and finds the distance and index of the closest object. It then uses these values to orient the robot and move it forward. The index is "rotated" by 180 degrees such that anything in the range of 150 to 210 degrees can be considered "in front" of the robot. If the index is within this range, the robot moves forward at a speed proportional to its distance (faster if further away). Otherwise, the robot moves forward at a constant speed. In either case, the robot adjusts it's angular velocity based on how far the index is from 180. Because of proportional control, the robot will also automatically stop when it is close (0.2m) to the object.
    * `PersonFollower.run()`: This is just a wrapper call to `rospy.spin()` to keep the robot busy.

![Person Follower](/gifs/person_follower.gif)

## Wall Follower
### Description 
This node will cause the robot to move forwards until it encounters a wall. After this, it will turn itself so that the robot is approximately parallel to the wall and begin "following" the wall. The robot will continue to move so that it is always approximately parallel to the wall, including around corners. 

### Code Explanation
* `WallFollower`: The pythonic class that is responsible for the **wall_follower** behavior.
    * `WallFollower.__init__()`: constructor for a `WallFollower` object. In addition to creating the object, this function initializes the ROS node, a `Twist` message for the node to use, a `Twist` publisher, a `LaserScan` subscriber, and several constants for proportional control.
    * `WallFollower.follow_wall()`: This function is responsible for the robot's behavior. The function will first check how far the closest wall to the robot is. If it is greater than 0.75m, then the robot is not likely currently following a wall and so the robot should move straight until it finds one. Once it does, the robot constantly adjusts its angular velocity so that the 45 degree sensor is approximately the same distance from the wall as the 135 degree sensor. This will ensure that the robot always remains roughly parallel to the wall and is essentially what the hint picture in the project writeup is cluing about. If the robot hits a corner (or the wall, when it is still searching for it), the robot will slow down and turn clockwise until its front sensor no longer detects that there is a wall within 0.75m of it. I also added a `side_error` variable to make sure that there is always a little bit of space (0.25m) between the robot and the wall. This makes rounding corners easier and keeps the robot from bumping into the wall while following. The `side_distance` is also used to assist in the corner detection, as the only time that the `side_distance` is greater than both the angle distances is when the robot is in a corner.
    * `WallFollower.run()`: Just a wrapper function for `rospy.spin()`.

The robot will find the wall in front of it and then begin following the perimeter.
![Wall Follower](/gifs/wall_follower_straight.gif)

It doesn't really matter what the initial angle is; the robot will readjust.
![Wall Follower Angle](/gifs/wall_follower_angle.gif)

## Challenges
For **drive_square**, one of the hardest parts was cleaning up the noise. If the velocity was too great, the noise artificially generated by Gazebo Simulator would throw the robot hopelessly off course. This was overcome by using a slower velocity (and thus a smaller noise). However, it should still be noted that the robot does not drive in a perfect square. After 3-4 iterations, it will no longer be on target. I didn't find **person_follower** terribly difficult as this was conceptually similar to **stop_at_wall**. For **wall_follower**, I had a lot of trouble figuring out how to round corners. I eventually settled on having a static angular velocity to turn clockwise anytime the robot's front was close to a wall or if the robot's side was closer to the wall than at least one of it's angles (which should only happen in a corner).

## Future Work
For **drive_square**, I would like to have the robot be able to drive faster and still be able to fend off the noise. I think this could be accomplished by gradually accelerating/decelerating when moving/stopping along a straight line, but I did not have a chance to test. For **person_follower**, I would like to test this with objects that automatically move rather than ones that are manually moved by the tester/developer. It would be cool to have a "leader robot" and a "follower robot". For **wall_follower**, similar to **drive_square** I wanted to make the robot speed up when it was farther from the wall to close the gaps quicker. However, the noise made this too unstable and the robot would always drift away from the wall. 

## Takeaways
* *Object Orientated code is very clean*. Although it requires a bit more thought and planning on the front-end, keeping OOP in mind makes your code really clean overall. Each subgoal/problem should be met with a single instance of an object. This object should have all the necessary methods and tools to solve this subtask. Although this project was short, I imagine this would be very handy for lengthier projects.
* *Noise is a major problem*. For any movement-based robot, this is going to be one of the main problems that needs solving. The noise completely throws off any robot moving/turning relatively fast. As such, the robot should be designed to be able to overcome significant amounts of noise, especially if it also needs to move quickly.
* *Proportional control is really useful!*. I'm not sure how I would have approached **person_follower** or **wall_follower** before learning about proportional control. But after that lecture and the **line_follower** example, it was not too difficult to see how this would come in handy. Especially for variables that you constantly need to update and adjust.
