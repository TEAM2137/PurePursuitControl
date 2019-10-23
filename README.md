# PurePursuitControl
FRC 2137 TORC's code for autonomous driving of our robot in preparation for the 2020 FRC game
This code is built off of the guidance of FRC 1712's Pure Pursuit whitepaper
## Components
This code consists of 3 major portions: Path generation, robot position tracking (odometry), path following (pure pursuit), and velocity control
### Path Generation
Path generation takes a set of waypoints consisting of X and Y and generating a path of more frequent points (roughly every 6 inches, can be customized) into a smooth path with information attached to each point
* We start with a list of waypoints that are the major points that the robot should pass through (or at least come close to)
* Point interpolation

    We start by adding points every 6 inches between these points, so that we have more points to work with when manipulating the path
* Path smoothing

    We use a path smoothing function taken from FRC 2168's path generation code to smooth the path so that it is easier to follow
* Add additional data to each point

    We add data to each point regarding how the robot will travel with it, and other characteristics
These are the distance along the path, the curvature of the path at each point, the max velocity at each point (for slowing around turns), and the target velocity at each point (to allow smooth deceleration, accleration is handled when following the path)
### Robot Position Tracking
To be able to follow the path, we need to know where the robot is when we update our path following
* To do this, we use Field-Relative Positioning through Non-Linear State Estimation, also called odometry

    This uses the encoder on each side of the drivetrain and the robot's gyro sensor to live track the robot's movement

    We calculate how far the robot moved from last iteration, and use the angle to do some trigonometry to figure out how far the robot moved in the X and Y axis
### Pure Pursuit
Pure Pursuit is a control algorithm that works by following the path generated above
* Find Closest Point

    By simply using the distance formula with the robot's current position to find the closest point, which is used for speed limiting
    
* Find Lookahead Point

    The lookahead point is the point that the robot aims for when following the path
    
    The lookahead point is found by finding the first (following the path from start to finish) intersection of a circle and a line segment between two points on the path
    
    Once we've found an intersection, we check that it isn't before the previous lookahead point, to prevent turning around.
    
* Drive the Robot

    We calculate the curvature our robot must take to get to the lookahead point
    
    Next we determine the target velocity using the closest point to the robot (calculated above) as our max speed, for slowing around turns, and using a rate limiter to limit acceleration
    
    We finally calculate the wheel speeds desired for each side of our drivetrain, and feed it to the velocity control
    
### Velocity Control
Using control loop to reach a desired drivetrain velocity for the robot for smooth control
* This is a constantly updating loop that takes the current velocity and determines the best course of action to reach the target velocity
* This takes a Velocity constant parameter, a Proportional parameter, and an Acceleration parameter, and does math with them to determine the new target wheel speed to reach and maintain velocity
