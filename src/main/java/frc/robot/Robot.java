/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.util.ArrayList;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.Faults;

import org.opencv.core.Point;

import frc.robot.PurePursuit.*;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private static final String kDefaultAuto = "Default";
  private static final String kCustomAuto = "My Auto";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();

  //Constants for the path generation/following go here
  //Everything is in inches because it's easier

  public static double pointSpacing = 6.0; //roughly 6 in point spacing (changes after smoothing)
  public static double maxVelocity = 13.0 * 12.0; // 13fps (it's a guess for now) * 12inches per foot. Is this inches per second?
  public static double turnSlowConstant = 3.0; //a value of 1-5 on how much the robot slows when turning
  public static double maxAccelerationPathGen = 8.0; //max acceleration for path generation, so essentally decel. I decided inPerSec
  //These might actually be the same, I'm unsure
  public static double maxAccelerationPathFollow = 8.0; //max acceleration when path following, so actaully accel. I decided inPerSec
  public static double pathExtensionLength = 10.0;
  public static double lookaheadDistance = 20.0; //12-25 in inches of how far to aim ahead
  public static double trackWidth = 25.0; //distance between wheels of robot side-to-side plus a couple to account for scrub
  public static double kv = 0.0; //velocity control constant, around 1/top robot speed
  public static double ka = 0.002; //acceleration constant for velocity control, start around 0.002
  public static double kp = 0.01; //p constant for velocity control

  DriveTrain driveTrain = new DriveTrain();

  ArrayList<Point> waypoints = new ArrayList<>();

  Command followPathCommand;

  XboxController controller = new XboxController(0);

  /**
   * This function is run when the robot is first started up and should be
   * used for any initialization code.
   */
  @Override
  public void robotInit() {
    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("My Auto", kCustomAuto);
    SmartDashboard.putData("Auto choices", m_chooser);

    waypoints.add(new Point(0,0));
    waypoints.add(new Point(5,1)); //likely has to be switched based on odometry direction
    followPathCommand = new FollowPath(driveTrain, waypoints);

    SmartDashboard.putBoolean("Reset Odometry/Encoders", false);

    driveTrain.initSubsystem();
  }

  /**
   * This function is called every robot packet, no matter the mode. Use
   * this for items like diagnostics that you want ran during disabled,
   * autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before
   * LiveWindow and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    //Put auto selector/path generation code in here, and have it display a green light on the dashboard
    //This way drive team can know when to go

    driveTrain.runPeriodic();

    driveTrain.updateOdometry();
    
    if(SmartDashboard.getBoolean("Reset Odometry/Encoders", false)) {
      SmartDashboard.putBoolean("Reset Odometry/Encoders", false);
      driveTrain.resetGyro();
      driveTrain.resetDriveEncoders();
      driveTrain.resetOdometry();
    }
  }

  @Override
  public void disabledInit() {
    driveTrain.enableAutonVelocityControl(false);
    driveTrain.enablePositionTracking(false);
  }

  /**
   * This autonomous (along with the chooser code above) shows how to select
   * between different autonomous modes using the dashboard. The sendable
   * chooser code works with the Java SmartDashboard. If you prefer the
   * LabVIEW Dashboard, remove all of the chooser code and uncomment the
   * getString line to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional comparisons to
   * the switch structure below with additional strings. If using the
   * SendableChooser make sure to add them to the chooser code above as well.
   */
  @Override
  public void autonomousInit() {
    // m_autoSelected = m_chooser.getSelected();
    // // m_autoSelected = SmartDashboard.getString("Auto Selector", kDefaultAuto);
    // System.out.println("Auto selected: " + m_autoSelected);

    driveTrain.enablePositionTracking(true);
    driveTrain.resetDriveEncoders();
    driveTrain.resetOdometry();
    driveTrain.resetGyro();
    driveTrain.initSubsystem();
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
    // RobotPosition currentPosition = driveTrain.getRobotPosition(); //Gotta figure out what the robot thinks is x, y, and angle
    // System.out.println("X: " + currentPosition.x + " Y: " + currentPosition.y + " Angle: " + currentPosition.angle);
    
    System.out.println("Left: " + driveTrain.getLeftEncoderPosition()); //make sure forward = forward
    System.out.println("Right: " + driveTrain.getRightEncoderPosition());

  }

  @Override
  public void teleopInit() {
    driveTrain.initSubsystem();
    driveTrain.resetOdometry();
    driveTrain.resetDriveEncoders();
    driveTrain.resetGyro();
    driveTrain.enablePositionTracking(true);
    // driveTrain.enablePositionTracking(false);
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
    //Just a simple drive program for testing stuff
    double throttle = -controller.getY(Hand.kLeft);
    double turn = controller.getX(Hand.kRight);

    // System.out.println("th" + throttle);
    // System.out.println("tu" + turn);

    double leftPower = throttle + turn;
    double rightPower = throttle - turn;

    driveTrain.setLeftPower(leftPower);
    driveTrain.setRightPower(rightPower);

    driveTrain.updateOdometry();

    System.out.println("x:" + driveTrain.getRobotPosition().x);
    System.out.println("y:" + driveTrain.getRobotPosition().y);
    System.out.println("a:" + driveTrain.getRobotPosition().angle);
    System.out.println("l:" + driveTrain.getLeftEncoderPosition());
    System.out.println("r:" + driveTrain.getRightEncoderPosition());
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  }
}
