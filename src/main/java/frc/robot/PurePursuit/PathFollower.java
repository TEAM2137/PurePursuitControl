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
 * What follows the generated paths
 * Here's how it works:
 * It starts by finding the closest point in the path to the robot
 * This is used for the robot's speed later on
 * Then it finds the point it is looking ahead to by finding if a line intersects a circle
 * This point is what the robot aims for
 * It then calculates how to drive to that point
 * And then updates motor values
 * This is then fed into the DriveTrain class for velocity control
 */
public class PathFollower {

    ArrayList<PathPoint> path;
    double maxAcceleration;
    double lookaheadDistance;
    double trackWidth;
    double kv;
    double ka;
    double kp;

    double targetVelocity;
    double leftSpeed;
    double rightSpeed;

    int lastRateLimiterCallTime;

    PathPoint closestPoint;
    Point lookaheadPoint;
    PathPoint lookaheadPointSegmentStart;

    RobotPosition currentPosition;

    /**
     * 
     * @param path Path to be followed
     * @param maxAcceleration Max acceleration of the robot
     * @param lookaheadDistance How far to lookahead to follow
     * @param trackWidth Distance between the wheel of the robot side-to-side
     * @param kv Velocity control constant, velocity
     * @param ka Velocity control constant, acceleration
     * @param kp Velocity control constant, P as in PID
     */
    public PathFollower(ArrayList<PathPoint> path, double maxAcceleration, double lookaheadDistance, 
                            double trackWidth, double kv, double ka, double kp) {
        this.path = path;
        this.maxAcceleration = maxAcceleration;
        this.lookaheadDistance = lookaheadDistance;
        this.trackWidth = trackWidth;
        this.kv = kv;
        this.ka = ka;
        this.kp = kp;

        closestPoint = path.get(0);
        lookaheadPoint = path.get(0);
        lookaheadPointSegmentStart = path.get(0);
    }

    /**
     * Updates the left and right wheel speeds
     */
    public void update() {
        updateClosestPoint(closestPoint, currentPosition);
        updateLookaheadPoint(lookaheadPoint, lookaheadPointSegmentStart, currentPosition, lookaheadDistance);

        double relativeX = findLookaheadRelativeX(lookaheadPoint, currentPosition);
        double unsidedCurvature = 2 * Math.abs(relativeX) / (Math.pow(lookaheadDistance,2)); 
        double curvature = unsidedCurvature * Math.signum(relativeX);

        double oldTargetVelocity = targetVelocity;
        targetVelocity = findTargetVelocity(maxAcceleration, oldTargetVelocity, closestPoint);

        double[] wheelSpeeds = calculateWheelSpeeds(targetVelocity, curvature, trackWidth);
        leftSpeed = wheelSpeeds[0]; //set public variables to new values
        rightSpeed = wheelSpeeds[1];
    }

    /**
     * Update the robot's position
     * @param position The robot's position
     */
    public void updateRobotPosition(RobotPosition position) {
        currentPosition = position;
    }


    public boolean isFinished() {
        return closestPoint == path.get(path.size()-1); 
    }

    /**
     * Finds closest point to robot
     * @param startingPoint Point to start search from, typically last closest point
     * @param currentPosition Current robot position
     */
    private void updateClosestPoint(PathPoint startingPoint, RobotPosition currentPosition) {
        PathPoint currentClosest = startingPoint;
        double currentClosestDistance = MathFunctions.findDistance(path.get(startingPoint.index), currentPosition);
        for (int i = startingPoint.index + 1; i < path.size(); i++) {
            PathPoint currentPoint = path.get(i);
            double currentDistance = MathFunctions.findDistance(currentPoint, currentPosition);
            if (currentDistance < currentClosestDistance) {
                currentClosest = path.get(i);
                currentClosestDistance = currentDistance;
            }
        }

        closestPoint = currentClosest;
    }

    /**
     * Updates lookahead point if needed
     * @param startingLookaheadPoint
     * @param startingPreviousPoint
     * @param currentPosition
     * @param lookaheadDistance
     */
    private void updateLookaheadPoint(Point startingLookaheadPoint, PathPoint startingPreviousPoint, 
                                            RobotPosition currentPosition, double lookaheadDistance) {
        
        //set to previous lookahead and prior point incase no new point is found
        PathPoint newLookaheadPreviousPoint = startingPreviousPoint; 
        Point newLookahead = startingLookaheadPoint;
        double oldFractionalIndex = findFractionalIndex(startingLookaheadPoint, startingPreviousPoint);
        for (int i = startingPreviousPoint.index; i < path.size() - 1; i++) { // Point can't go backwards in segments
            PathPoint lineStart = path.get(i);
            PathPoint lineEnd = path.get(i+1);
            ArrayList<Point> intersections = MathFunctions.circleLineIntersection(currentPosition, lookaheadDistance, 
                lineStart, lineEnd);
            boolean toBreak = true;
            switch(intersections.size()) {
                case 0: // No intersections, skip this point
                    break;
                case 1: // 1 intersection, check if fractional index increases, no going backwards
                    if(findFractionalIndex(intersections.get(0), lineStart) > oldFractionalIndex) {
                        newLookahead = intersections.get(0);
                        newLookaheadPreviousPoint = lineStart;
                        toBreak = true;
                    }
                    break;
                case 2: // 2 intersections, choose first point where fractional index increases
                    double fractionalIndex0 = findFractionalIndex(intersections.get(0), lineStart);
                    double fractionalIndex1 = findFractionalIndex(intersections.get(1), lineStart);
                    if (fractionalIndex0 > fractionalIndex1 && fractionalIndex0 > oldFractionalIndex) {
                        newLookahead = intersections.get(0);
                        newLookaheadPreviousPoint = lineStart;
                        toBreak = true;
                    } else if (fractionalIndex1 > fractionalIndex0 && fractionalIndex1 > oldFractionalIndex) {
                        newLookahead = intersections.get(1);
                        newLookaheadPreviousPoint = lineStart;
                        toBreak = true;
                    }
                    break;
            }
            if (toBreak) {
                break;
            }
        }

        lookaheadPoint = newLookahead;
        lookaheadPointSegmentStart = newLookaheadPreviousPoint;
    }

    /**
     * Finds the X of the lookahead point relative to the robot
     * @param lookaheadPoint Current lookahead point
     * @param currentPosition Current robot position
     * @return Distance from robot line to lookahead point
     */
    private double findLookaheadRelativeX(Point lookaheadPoint, RobotPosition currentPosition) {
        double robotX = currentPosition.x;
        double robotY = currentPosition.y;
        double robotAngle = currentPosition.angle;

        double lookaheadX = lookaheadPoint.x;
        double lookaheadY = lookaheadPoint.y;

        //Math to fine point distance from robot line
        double a = -Math.tan(robotAngle);
        double b = 1;
        double c = (Math.tan(robotAngle) * robotX) - robotY;
        double unsidedX = Math.abs(a * lookaheadX + b * lookaheadY + c) / Math.sqrt(Math.pow(a,2) + Math.pow(b,2));

        //Now find side of the robot line X is on
        double side = Math.signum((Math.sin(robotAngle) * (lookaheadX  - robotX)) - 
            (Math.cos(robotAngle) * (lookaheadY - robotY))); 

        double sidedX = unsidedX * side;
        return sidedX;
    }

    /**
     * Find the fractional index of a point along the path
     * @param pointToFind Point along the path
     * @param previousPoint PathPoint just before it
     * @return Fractional index of the point
     */
    private double findFractionalIndex(Point pointToFind, PathPoint previousPoint) {
        double wholeSegmentLength = MathFunctions.findDistance(previousPoint, path.get(previousPoint.index + 1));
        double smallSegmentLength = MathFunctions.findDistance(previousPoint, pointToFind);
        double fraction = smallSegmentLength/wholeSegmentLength;
        double fractionalIndex = previousPoint.index + fraction;

        /**
         * Fractional index is as follows
         * Take the PathPoint before it on the path, this is the base number
         * Then, find the total distance between that point and the point following it on the path
         * Divide the distance from the Point to the PathPoint by the overall segment length
         * Then add that to the PathPoint before it's index
         * This is used so that you never go backwards in the path
         */

        return fractionalIndex;
    }

    /**
     * Find the target velocity of the robot
     * @param maxAcceleration
     * @param previousTargetVelocity
     * @param closestPoint
     * @return Target velocity of the robot given the parameters
     */
    private double findTargetVelocity(double maxAcceleration, double previousTargetVelocity, PathPoint closestPoint) {
        double currentTime = System.currentTimeMillis();
        double maxChange = ((currentTime - lastRateLimiterCallTime) / 1000) * maxAcceleration;
        double output = MathFunctions.clamp(closestPoint.targetVelocity - previousTargetVelocity, -maxChange, maxChange);

        return output;
    }

    /**
     * Finds each wheel's speed
     * @param targetVelocity
     * @param curvature
     * @param trackWidth
     * @return A double[] of {leftSpeed, rightSpeed}
     */
    private double[] calculateWheelSpeeds(double targetVelocity, double curvature, double trackWidth) {
        double leftSpeed = targetVelocity * (2 + (curvature * trackWidth)) / 2;
        double rightSpeed = targetVelocity * (2 - (curvature * trackWidth)) / 2;
        double[] speedArray = {leftSpeed, rightSpeed};

        return speedArray;
    }

    /**
     * @return the leftSpeed
     */
    public double getLeftSpeed() {
        return leftSpeed;
    }

    /**
     * @return the rightSpeed
     */
    public double getRightSpeed() {
        return rightSpeed;
    }
}
