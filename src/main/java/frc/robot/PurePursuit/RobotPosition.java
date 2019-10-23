/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.PurePursuit;

import org.opencv.core.Point;

/**
 * Add your docs here.
 */
public class RobotPosition extends Point{

    public double angle;

    public RobotPosition(double x, double y, double angle) {
        this.x = x;
        this.y = y;
        this.angle = angle;
    }
}
