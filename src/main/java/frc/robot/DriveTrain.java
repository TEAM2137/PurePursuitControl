/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.PurePursuit.RobotPosition;

/**
 * Add your docs here.
 */
public class DriveTrain extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  // TODO: tune values
  public double kv = 0.0; //velocity control constant, around 1/top robot speed
  public double ka = 0.002; //acceleration constant for velocity control, start around 0.002
  public double kp = 0.01; //p constant for velocity control

  public double countsPerInch = 180; // TODO: update

  TalonSRX leftMotor = new TalonSRX(22);
  TalonSRX rightMotor = new TalonSRX(10);
  TalonSRX leftFollower = new TalonSRX(23);
  TalonSRX rightFollower = new TalonSRX(11);

  int leftVelocityLastCallTime = 0;
  int rightVelocityLastCallTime = 0;

  double leftVelocityPreviousTarget = 0;
  double rightVelocityPreviousTarget = 0;

  double leftVelocityControlTarget = 0;
  double rightVelocityControlTarget = 0;

  double leftVelocityControlAccel = 0;
  double rightVelocityControlAccel = 0;

  boolean velocityControlEnabled = false;

  boolean positionTrackingEnabled = false;

  public RobotPosition currentPosition;
  
  int previousLeftEncoderValue = 0;
  int previousRightEncoderValue = 0;

  PigeonIMU pidgeon = new PigeonIMU(4);
  PigeonIMU.GeneralStatus pidgeonStatus = new PigeonIMU.GeneralStatus();

  public void initSubsystem() {
    // leftMotor.setInverted(true);
    // leftFollower.setInverted(true);
    leftFollower.follow(leftMotor);
    rightFollower.follow(rightMotor);
    resetGyro();
    resetDriveEncoders();
    resetOdometry();
    enableAutonVelocityControl(false);
    enablePositionTracking(false);
  }

  public void resetGyro() {
    pidgeon.setYaw(0);
  }

  public void resetLeftEncoder() {
    leftMotor.getSensorCollection().setQuadraturePosition(0,0);
  }

  public void resetRightEncoder() {
    rightMotor.getSensorCollection().setQuadraturePosition(0,0);
  }

  public void resetDriveEncoders() {
    resetLeftEncoder();
    resetRightEncoder();
  }

  public double getGyroYaw() {
    double[] ypr = new double[3];
    pidgeon.getYawPitchRoll(ypr);
    return -ypr[0];
  }

  public void resetOdometry() {
    setOdometryPosition(0, 0, 0);
  }

  public void setOdometryPosition(double x, double y, double angle) {
    currentPosition = new RobotPosition(x, y, angle);
  }

  public RobotPosition getRobotPosition() {
    return currentPosition;
  }

  public void setLeftPower(double power) {
    leftMotor.set(ControlMode.PercentOutput, power);
  }
  public void setRightPower(double power) {
    rightMotor.set(ControlMode.PercentOutput, power);
  }

  public int getLeftEncoderPosition() {
    return -leftMotor.getSensorCollection().getQuadraturePosition();
  }

  public int getRightEncoderPosition() {
    return rightMotor.getSensorCollection().getQuadraturePosition();
  }

  public int getLeftEncoderVelocity() {
    return leftMotor.getSensorCollection().getQuadratureVelocity();
  }

  public int getRightEncoderVelocity() {
    return rightMotor.getSensorCollection().getQuadratureVelocity();
  }

  public int inchesToCounts(double inches) {
    return (int) Math.floor(inches * countsPerInch);
  }

  public double countsToInches(int counts) {
    return counts/countsPerInch;
  }

  public double getLeftVelocityInchesPerSecond() {
    return 10 * countsToInches(getLeftEncoderVelocity());
  }

  public double getRightVelocityInchesPerSecond() {
    return 10 * countsToInches(getRightEncoderVelocity());
  }

  private void runLeftVelocityControl() {
    double ff = (kv * leftVelocityControlTarget) + (ka *  leftVelocityControlAccel);
    double fb = kp * (leftVelocityControlTarget - getLeftVelocityInchesPerSecond()); //probably some conversion required
    double power = ff + fb;
    setLeftPower(power);
  }

  private void runRightVelocityControl() {
    double ff = (kv * rightVelocityControlTarget) + (ka *  rightVelocityControlAccel);
    double fb = kp * (rightVelocityControlTarget - getRightVelocityInchesPerSecond()); //probably some conversion required
    double power = ff + fb;
    setRightPower(power);
  }

  /**
   * @param leftVelocityControlTarget the leftVelocityControlTarget to set
   */
  public void setLeftVelocityControlTarget(double leftVelocityControlTarget) {
    this.leftVelocityPreviousTarget = this.leftVelocityControlTarget;
    this.leftVelocityControlTarget = leftVelocityControlTarget;
    double targetChange = leftVelocityControlTarget - leftVelocityPreviousTarget;
    double timeChange = (System.currentTimeMillis() - leftVelocityLastCallTime) / 1000; //time in seconds
    this.leftVelocityControlAccel = targetChange / timeChange;
;  }

  /**
   * @param rightVelocityControlTarget the rightVelocityControlTarget to set
   */
  public void setRightVelocityControlTarget(double rightVelocityControlTarget) {
    this.leftVelocityPreviousTarget = this.rightVelocityControlTarget;
    this.rightVelocityControlTarget = rightVelocityControlTarget;
    double targetChange = rightVelocityControlTarget - rightVelocityPreviousTarget;
    double timeChange = (System.currentTimeMillis() - rightVelocityLastCallTime) * 1000; //time in seconds
    this.rightVelocityControlAccel = targetChange / timeChange;
  }

  public void enableAutonVelocityControl(boolean enabled) {
    // boolean priorState = velocityControlEnabled;
    velocityControlEnabled = enabled;
    // if (!priorState && enabled) {
    //   Thread t = new Thread(() -> { //TODO: should I take this off of its own thread and it with the rest of the path follower 
    //     while (!Thread.interrupted() && velocityControlEnabled) {
    //       runLeftVelocityControl();
    //       runRightVelocityControl();
    //     }
    //   });
    //   t.start();
    // }
  }

  public void enablePositionTracking(boolean enabled) {
    // boolean priorState = positionTrackingEnabled;
    positionTrackingEnabled = enabled;
    // if (!priorState && enabled) {
    //   Thread t = new Thread(() -> { 
    //     while (!Thread.interrupted() && positionTrackingEnabled) {
    //       updateOdometry();
    //    }
    //   });
    //   t.start();
    // }
  }

  void updateOdometry() {
    int currentLeftEncoder = getLeftEncoderPosition();
    int currentRightEncoder = getRightEncoderPosition();
    int leftEncoderChange = currentLeftEncoder - previousLeftEncoderValue;
    int rightEncoderChange = currentRightEncoder - previousRightEncoderValue;
    // System.out.println("lchange: " + leftEncoderChange);
    // System.out.println("rchange: " + rightEncoderChange);
    double leftEncoderChangeInches = countsToInches(leftEncoderChange);
    double rightEncoderChangeInches = countsToInches(rightEncoderChange);
    double distance = (leftEncoderChangeInches + rightEncoderChangeInches) / 2;
    double xChange = distance * Math.cos(getGyroYaw());
    double yChange = distance * Math.sin(getGyroYaw());
    currentPosition.x += xChange;
    currentPosition.y += yChange;
    currentPosition.angle = getGyroYaw();
    previousLeftEncoderValue = currentLeftEncoder;
    previousRightEncoderValue = currentRightEncoder;
  }

  public void runPeriodic() {
    if (positionTrackingEnabled) {
      updateOdometry();
    }
    if (velocityControlEnabled) {
      runLeftVelocityControl();
      runRightVelocityControl();
    }

    //Smartdashboard outputs
    SmartDashboard.putNumber("Robot X Position", currentPosition.x);
    SmartDashboard.putNumber("Robot Y Position", currentPosition.y);
    SmartDashboard.putNumber("Robot Angle", currentPosition.angle);

    SmartDashboard.putNumber("Left Encoder", getLeftEncoderPosition());
    SmartDashboard.putNumber("Right Encoder", getRightEncoderPosition());
    SmartDashboard.putNumber("Left Velocity", getLeftEncoderVelocity());
    SmartDashboard.putNumber("Right Velocity", getRightEncoderPosition());
    SmartDashboard.putNumber("Left Inches", countsToInches(getLeftEncoderPosition()));
    SmartDashboard.putNumber("Right Inches", countsToInches(getRightEncoderPosition()));
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
}
