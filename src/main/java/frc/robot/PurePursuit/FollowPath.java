/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.PurePursuit;

import java.util.ArrayList;

import org.opencv.core.Point;

import frc.robot.*;

import edu.wpi.first.wpilibj.command.Command;

public class FollowPath extends Command {

  DriveTrain driveTrain;

  RobotPosition currentPosition;

  PathFollower follower;

  ArrayList<PathPoint> path;

  double leftTargetSpeed;
  double rightTargetSpeed;

  public FollowPath(DriveTrain driveTrain, ArrayList<Point> waypoints) {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    this.driveTrain = driveTrain;

    path = PathGenerator.GetInstance().generatePath(waypoints, Robot.pointSpacing, Robot.maxVelocity, 
      Robot.turnSlowConstant, Robot.maxAccelerationPathGen, Robot.pathExtensionLength);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    driveTrain.enableAutonVelocityControl(true);
    driveTrain.enablePositionTracking(true);

    driveTrain.setLeftVelocityControlTarget(0);
    driveTrain.setRightVelocityControlTarget(0);

    follower = new PathFollower(path, Robot.maxAccelerationPathFollow, Robot.lookaheadDistance, Robot.trackWidth,
       Robot.kv, Robot.ka, Robot.kp);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    currentPosition = driveTrain.getRobotPosition();
    follower.updateRobotPosition(currentPosition);
    follower.update();
    leftTargetSpeed = follower.getLeftSpeed();
    rightTargetSpeed = follower.getRightSpeed();
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return follower.isFinished();
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}
