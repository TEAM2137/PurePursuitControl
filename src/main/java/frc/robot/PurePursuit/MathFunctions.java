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
 * Add your docs here.
 */
public class MathFunctions {
    /**
     * Finds the distance between 2 points
     * @param point1
     * @param point2
     * @return
     */
    public static double findDistance(Point point1, Point point2) {
        double xChangeTotal = point1.x-point2.x;
        double yChangeTotal = point1.y-point2.y;
        double length = Math.sqrt(Math.pow(xChangeTotal, 2) + Math.pow(yChangeTotal, 2));
        return length;
    }

    /**
     * Finds if a circle at location w/ radius intersects a line segment
     * Stolen from FTC 11115's Pure Pursuit tutorial becasue 1712's formula was too complex for me
     * I'm only in precalc, deal with it
     * But now we have a god in our code
     * @param circleCenter Center of circle to work from
     * @param circleRadius Radius of circle
     * @param lineStart Start of line
     * @param lineEnd End of line
     * @return
     */
    public static ArrayList<Point> circleLineIntersection(Point circleCenter, double circleRadius, Point lineStart, Point lineEnd) {
        
        if (Math.abs(lineStart.y - lineEnd.y) < 0.003) {
            lineStart.y = lineEnd.y + 0.003;
        }
        if (Math.abs(lineStart.x - lineEnd.x) < 0.003) {
            lineStart.x = lineEnd.x + 0.003;
        }

        double m1 = (lineEnd.y - lineStart.y)/(lineEnd.x - lineStart.x);

        double quadraticA = 1.0 + Math.pow(m1, 2);

        //Make it so that the circle is centered at (0,0) for easier math
        double x1 = lineStart.x - circleCenter.x;
        double y1 = lineStart.y - circleCenter.y;

        double quadraticB = (2.0 * m1 * y1) - (2.0 * Math.pow(m1,2) * x1);

        double quadraticC = ((Math.pow(m1,2) * Math.pow(x1,2))) - (2.0*y1*m1*x1) + Math.pow(y1,2) - Math.pow(circleRadius,2);

        ArrayList<Point> allPoints = new ArrayList<>();

        try {
            double xRoot1 = (-quadraticB + Math.sqrt(Math.pow(quadraticB,2) - (4.0 * quadraticA * quadraticC))) / (2.0 * quadraticA);

            double yRoot1 = m1 * (xRoot1 - x1) + y1;

            //Add the offset back
            xRoot1 += circleCenter.x;
            yRoot1 += circleCenter.y;

            double minX = lineStart.x < lineEnd.x ? lineStart.x : lineEnd.x;
            double maxX = lineStart.x > lineEnd.x ? lineStart.x : lineEnd.x;
            
            if (xRoot1 > minX && xRoot1 < maxX) {
                allPoints.add(new Point(xRoot1, yRoot1));
            } 

            double xRoot2 = (-quadraticB - Math.sqrt(Math.pow(quadraticB,2) - (4.0 * quadraticA * quadraticC))) / (2.0 * quadraticA);

            double yRoot2 = m1 * (xRoot2 - x1) + y1;

            //Add the offset back
            xRoot2 += circleCenter.x;
            yRoot2 += circleCenter.y;
            
            if (xRoot2 > minX && xRoot2 < maxX) {
                allPoints.add(new Point(xRoot2, yRoot2));
            } 

        } catch (Exception e) {

        }
        
        return allPoints;
    }

    /**
     * Limits a number between min and max
     * @param val Value to limit
     * @param min Min allowable value
     * @param max Max allowable value
     * @return
     */
    public static double clamp(double val, double min, double max) {
	    return Math.max(min, Math.min(max, val));
	}
}
