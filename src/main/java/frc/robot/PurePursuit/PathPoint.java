/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.PurePursuit;

import org.opencv.core.Point;

/**
 * This is a point in the path
 * Extends OpenCV's point because I like OpenCV's point and it has nice methods
 */
public class PathPoint extends Point {

    public int index;
    public double maxVelocity;
    public double targetVelocity;
    public double curvature;
    public double distanceAlongPath;

    /**
     * 
     * @param x X location
     * @param y Y location
     * @param maxVelocity Max velocity at that point
     * @param targetVelocity Target Velocity at point
     * @param curvature Curvature of path at point
     * @param distanceAlongPath Distance from start of path
     */
    PathPoint(int index, double x, double y, double maxVelocity, double targetVelocity, double curvature, double distanceAlongPath) {
        this.index = index;
        this.x = x;
        this.y = y;
        this.maxVelocity = maxVelocity;
        this.targetVelocity = targetVelocity;
        this.curvature = curvature;
        this.distanceAlongPath = distanceAlongPath;
    }

    /**
     * 
     * @param index Index in path
     * @param x X location
     * @param y Y location
     */
    PathPoint(int index, double x, double y) {
        this.index = index;
        this.x = x;
        this.y = y;
    }

    PathPoint(Point point) {
        this.x = point.x;
        this.y = point.y;
    }

    PathPoint(Point point, int index) {
        this.x = point.x;
        this.y = point.y;
        this.index = index;
    }
}
