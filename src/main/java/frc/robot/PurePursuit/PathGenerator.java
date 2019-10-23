/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.PurePursuit;

import java.util.ArrayList;

import org.opencv.core.Point;

/**
 * This generates the paths for PurePursuit
 * Works by being given an ArrayList of Points (from org.opencv.core) that serve as waypoints
 * Then, we take those points and create points between them equally spaced at 6"
 * After that, we smooth the path so that it's not janky
 * We then calculate the curvature of each point
 * Using the curvature calculated, we limit speed on turns for a max velocity
 * We then go backwards through the path to figure out the deceleration required for each point to get target velocities
 * We don't do acceleration because then the robot would never move, that's solved in path following
 * 
 * This methodology for all of this PurePursuit is taken from FRC 1712's whitepaper which can be found here:
 * https://www.chiefdelphi.com/uploads/default/original/3X/b/e/be0e06de00e07db66f97686505c3f4dde2e332dc.pdf
 * 
 * I also stole FTC 11115's circle-line intersection code from their tutorial on youtube
 * 
 * TODO: Consider adding a save to/load from CSV option?
 * This will depend on how long generation takes
 * You can save to CSV on the Rio and get it off by connecting to the Rio
 * After that, just throw it into the deploy folder
 */
public class PathGenerator {

    //Stolen from 2168, for smoother function, hope it works
    double pathAlpha = 0.7;
    double pathBeta = 0.3;
    double pathTolerance = 0.0000001;

    private PathGenerator() {} //Singleton because it's just used for operations, and is just for functions
    
    private static PathGenerator SingleInstance;

    public static PathGenerator GetInstance() {
        if (SingleInstance == null) {
          SingleInstance = new PathGenerator();
        }
        return SingleInstance;
      }

    /**
     * Generates a path that can be followed by PathFollower
     * @param waypoints List of waypoints to generate path from
     * @param pointSpacing Spacing between inserted points in path, typically 6"
     * @param pathMaxVelocity Max velocity of the path
     * @param turnSlowConstant How much the robot should slow around turns, typically 1-5, high is slower
     * @param pathMaxAcceleration Max acceleration of the robot. Maybe made deceleration since it's used for target velocity
     * @return An ArrayList of PathPoints that is the path to follow
     */

    public ArrayList<PathPoint> generatePath(ArrayList<Point> waypoints, double pointSpacing, double pathMaxVelocity, 
        double turnSlowConstant, double pathMaxAcceleration) {
        
        ArrayList<Point> injectedPoints = new ArrayList<>();
        //Run through each current segment and inject points into it
        for (int i = 0; i < waypoints.size() - 1; i++) {
            injectedPoints.addAll(injectPoints(waypoints.get(i), waypoints.get(i+1), pointSpacing));    
        }
        // injectedPoints.addAll(injectPoints(new Point(1,0), new Point(5,0), pointSpacing));
        //injectPoints doesn't add the last point of each segment, so add it
        injectedPoints.add(waypoints.get(waypoints.size()-1)); 

        //Smooth path
        ArrayList<Point> smoothedPath = smoother(injectedPoints);

        //Convert to PathPoints now that we need them
        ArrayList<PathPoint> pathPointPath = pointToPathPoint(smoothedPath);

        //Add distance from previous point to each PathPoint
        ArrayList<PathPoint> distancesAddedPath = addDistancesToPoints(pathPointPath);

        //Figure out curvature of path at each point
        ArrayList<PathPoint> curvatureAddedPath = addCurvatureToPoints(distancesAddedPath);

        //Now calculate max velocity at each point
        ArrayList<PathPoint> maxVelocityPath = addMaxVelocityToPoints(curvatureAddedPath, pathMaxVelocity, turnSlowConstant);

        //Now the fun computing target velocity
        ArrayList<PathPoint> targetVelocityPath = addTargetVelocityToPoints(maxVelocityPath, pathMaxAcceleration);

        return targetVelocityPath; 
    }

    /**
     * Injects points into the path
     * @param startPoint Point to start the injection from, included in result
     * @param endPoint Point to head to, not included in result
     * @param pointSpacing Spacing between Points
     * @return An Arraylist of the points between the startPoint and endPoint, not including endPoint
     */
    private ArrayList<Point> injectPoints(Point startPoint, Point endPoint, double pointSpacing) {
        double xChangeTotal = endPoint.x-startPoint.x;
        double yChangeTotal = endPoint.y-startPoint.y;
        double length = Math.sqrt(Math.pow(xChangeTotal, 2) + Math.pow(yChangeTotal, 2));
        double numPointsToAdd = Math.ceil(length/pointSpacing);
        double xChangePerPoint = xChangeTotal/numPointsToAdd;
        double yChangePerPoint = yChangeTotal/numPointsToAdd;

        ArrayList<Point> injectedPoints = new ArrayList<>();

        for (int i = 0; i < numPointsToAdd; i++) {
            double newPointX = startPoint.x + (xChangePerPoint * i);
            double newPointY = startPoint.y + (yChangePerPoint * i);
            injectedPoints.add(new Point(newPointX, newPointY));
        }

        return injectedPoints;
    }

    /**
	 * Optimization algorithm, which optimizes the data points in path to create a smooth trajectory.
	 * This optimization uses gradient descent. While unlikely, it is possible for this algorithm to never
	 * converge. If this happens, try increasing the tolerance level.
     * 
     * Modified from team 2168
     * Modification is just to have it use an Arraylist on input and output
	 * 
	 * BigO: N^x, where X is the number of of times the while loop iterates before tolerance is met. 
	 * 
	 * @param path
	 * @param weight_data
	 * @param weight_smooth
	 * @param tolerance
	 * @return A smoothed path
	 */
	public ArrayList<Point> smoother(ArrayList<Point> pathToSmooth) {
        double weight_data = pathAlpha;
        double weight_smooth = pathBeta;
        double tolerance = pathTolerance;

        //convert ArrayList to array to get it to work

        double [][] path = new double[pathToSmooth.size()][2];
        for (int i = 0; i < pathToSmooth.size(); i++) {
            path[i][0] = pathToSmooth.get(i).x;
            path[i][1] = pathToSmooth.get(i).y;
        }

		//copy array
		double[][] newPath = doubleArrayCopy(path);

		double change = tolerance;
		while(change >= tolerance)
		{
			change = 0.0;
			for(int i=1; i<path.length-1; i++)
				for(int j=0; j<path[i].length; j++)
				{
					double aux = newPath[i][j];
					newPath[i][j] += weight_data * (path[i][j] - newPath[i][j]) + weight_smooth * (newPath[i-1][j] + newPath[i+1][j] - (2.0 * newPath[i][j]));
					change += Math.abs(aux - newPath[i][j]);	
				}					
		}

        //convert back to ArrayList because I like it that way
        ArrayList<Point> smoothedPath = new ArrayList<>();
        for (int i = 0; i < newPath.length; i++) {
            smoothedPath.add(new Point(newPath[i][0], newPath[i][1]));
        }

		return smoothedPath;

    }
    
    /**
	 * Performs a deep copy of a 2 Dimensional Array looping thorough each element in the 2D array
	 * 
     * From team 2168
     * 
	 * BigO: Order N x M
	 * @param arr
	 * @return A copy of the array
	 */
	public static double[][] doubleArrayCopy(double[][] arr)
	{

		//size first dimension of array
		double[][] temp = new double[arr.length][arr[0].length];

		for(int i=0; i<arr.length; i++)
		{
			//Resize second dimension of array
			temp[i] = new double[arr[i].length];

			//Copy Contents
			for(int j=0; j<arr[i].length; j++)
				temp[i][j] = arr[i][j];
		}

		return temp;

    }

    /**
     * Converts an ArrayList of points to an ArrayList of PathPoints
     * @param list List of points to convert
     * @return ArrayList of PathPoints
     */
    public ArrayList<PathPoint> pointToPathPoint(ArrayList<Point> list) {
        ArrayList<PathPoint> newList = new ArrayList<>();
        for (int i = 0; i < list.size(); i++) {
            newList.add(new PathPoint(list.get(i), i));
        }

        return newList;
    }

    /**
     * Adds distance from start of path to path
     * @param path Path to have distances added to
     * @return Arraylist of the path
     */
    private ArrayList<PathPoint> addDistancesToPoints(ArrayList<PathPoint> path) {
        double runningDistanceTotal = 0;
        ArrayList<PathPoint> distancesAddedPath = new ArrayList<>();
        PathPoint startPointDistance = path.get(0);
        startPointDistance.distanceAlongPath = 0;
        distancesAddedPath.add(startPointDistance);
        for (int i = 1; i < path.size(); i++) {
            PathPoint previousPoint = path.get(i-1);
            PathPoint modifyingPoint = path.get(i);
            double distanceBetweenPoints = MathFunctions.findDistance(previousPoint, modifyingPoint);
            runningDistanceTotal += distanceBetweenPoints;
            modifyingPoint.distanceAlongPath = runningDistanceTotal;
            distancesAddedPath.add(modifyingPoint);
        }

        return distancesAddedPath;
    }

    /**
     * Adds curvature attribute to path
     * @param path Path to have curvature added to
     * @return Arraylist of the Path
     */
    private ArrayList<PathPoint> addCurvatureToPoints(ArrayList<PathPoint> path) {
        ArrayList<PathPoint> curvatureAddedPath = new ArrayList<>();

        PathPoint startPointCurvature = path.get(0);
        startPointCurvature.curvature = 0;
        curvatureAddedPath.add(startPointCurvature);

        for (int i = 1; i < path.size()-1; i++) {
            PathPoint previousPoint = path.get(i-1);
            PathPoint modifyingPoint = path.get(i);
            PathPoint followingPoint = path.get(i+1);
            
            double x1 = modifyingPoint.x;
            double y1 = modifyingPoint.y;
            double x2 = previousPoint.x;
            double y2 = previousPoint.y;
            double x3 = followingPoint.x;
            double y3 = followingPoint.y;

            //Ensure no divide by zero error in k1
            if (x1 == x2) {
                x1 += 0.0001;
            }

            double k1 = 0.5 * (Math.pow(x1,2) + Math.pow(y1,2) - Math.pow(x2,2) - Math.pow(y2,2)) / (x1 - x2);
            double k2 = (y1 - y2)/(x1 - x2);
            double b  = 0.5 * (Math.pow(x2,2) - (2 * x2 * k1) + Math.pow(y2,2) - Math.pow(x3,2) + (2 * x3 *k1)) - Math.pow(y3,2) /
                            ((x3 * k2) - y3 + y2 - (x2 * k2));
            double a  = k1 - (k2 * b);
            double r  = Math.sqrt(Math.pow(x1 - a, 2) + Math.pow(y1 - b, 2));
            double curvature = 1/r;

            //Check if curvature is Nan, which is straight line
            if (curvature == Double.NaN) {
                curvature = 0;
            }
            modifyingPoint.curvature = curvature;

            curvatureAddedPath.add(modifyingPoint);
        }
        PathPoint endPointCurvature = path.get(0);
        endPointCurvature.curvature = 0;
        curvatureAddedPath.add(endPointCurvature);

        return curvatureAddedPath;
    }

    /**
     * Adds maximum velocity to path
     * @param path Path to have max velocity added to
     * @param pathMaxVelocity Max velocity of the path
     * @param turnSlowConstant How much the robot should slow around turns, typically 1-5, high is slower
     * @return Arraylist of the path
     */
    private ArrayList<PathPoint> addMaxVelocityToPoints(ArrayList<PathPoint> path, double pathMaxVelocity, double turnSlowConstant) {
        ArrayList<PathPoint> maxVelocityPath = new ArrayList<>();

        for (int i = 0; i < path.size(); i++) {
            PathPoint modifyingPoint = path.get(i);
            double pointCurvature = modifyingPoint.curvature;
            double pointMaxVelocity = Math.min(pathMaxVelocity, turnSlowConstant/pointCurvature);
            modifyingPoint.maxVelocity = pointMaxVelocity;
            maxVelocityPath.add(modifyingPoint);
        }

        return maxVelocityPath;
    }

    /**
     * Adds target velocity to path
     * @param path Path to have target velocity added to
     * @param pathMaxAcceleration Max acceleration of the path
     * @return ArrayList of points with all attributes filled out
     */
    private ArrayList<PathPoint> addTargetVelocityToPoints(ArrayList<PathPoint> path, double pathMaxAcceleration) {
        //Run through the path backwards, using kinematics to determine the max reachable velocity
        //  given the previous (after it in the path)'s target velocity
        //Take this and max velocity calculated above, take the smaller of the two

        ArrayList<PathPoint> targetVelocityPath = new ArrayList<>();

        PathPoint endPointVelocity = path.get(path.size()-1);
        endPointVelocity.targetVelocity = 0;
        targetVelocityPath.add(endPointVelocity);
        
        for (int i = path.size() - 2; i > 0; i++) {
            PathPoint previousPoint = path.get(i+1); //technically after it on the path, but decel is backwards accel
            PathPoint modifyingPoint = path.get(i);

            double vi = previousPoint.targetVelocity;
            double a = pathMaxAcceleration;
            double d = MathFunctions.findDistance(previousPoint, modifyingPoint);

            double vf = Math.sqrt(Math.pow(vi,2) + (2 * a * d)); //max attainable velocity at the current point

            double targetVelocity = Math.min(modifyingPoint.maxVelocity, vf);
            modifyingPoint.targetVelocity = targetVelocity;
        }

        return targetVelocityPath;
    }
}
