// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.config.PIDConstants;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import swervelib.math.Matter;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean constants. This
 * class should not be used for any other purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants
{

  public static final double ROBOT_MASS = (148 - 20.3) * 0.453592; // 32lbs * kg per pound
  public static final Matter CHASSIS    = new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), ROBOT_MASS);
  public static final double LOOP_TIME  = 0.13; //s, 20ms + 110ms sprk max velocity lag
  public static final double MAX_SPEED  = Units.feetToMeters(15.1);


  public static class DrivetrainConfig {
    public static final double MAX_DRIVE_SPEED = 10.0; // m/s
    public static final double MAX_TURN_SPEED = 200.0; // deg/s
    public static final double SLOWMODE_FACTOR = 0.4;
    public static final PIDConstants DRIVE_PID = new PIDConstants(0.5, 0, 0);
    public static final PIDConstants TURN_PID = new PIDConstants(0.5, 0, 0);
  }
  

  public static class SystemConfig {
   
  }
  // Maximum speed of the robot in meters per second, used to limit acceleration.

//  public static final class AutonConstants
//  {
//
//    public static final PIDConstants TRANSLATION_PID = new PIDConstants(0.7, 0, 0);
//    public static final PIDConstants ANGLE_PID       = new PIDConstants(0.4, 0, 0.01);
//  }

  public static final class DrivebaseConstants
  {

    // Hold time on motor brakes when disabled
    public static final double WHEEL_LOCK_TIME = 10; // seconds
  }

  public static class OperatorConstants
  {


    // Joystick Deadband
    public static final double DEADBAND        = 0.1;
    public static final double LEFT_Y_DEADBAND = 0.1;
    public static final double RIGHT_X_DEADBAND = 0.1;
    public static final double TURN_CONSTANT    = 6;
    public static final int DRIVER_PORT         = 0;
    public static final int OPERATOR_PORT       = 1;
  }

  public static class CanConstants
  {
    // CAN IDs
    //Swerver motor 1 Front Right Neo Spark Max
    public static final int SwerveFrontRightDriveMotorCanID  = 10;
    public static final int SwerveFrontRightSteerMotorCanID  = 11;
    public static final int CanCoderFrontRight                = 20;
    
    //Swerver motor 2 Back Right Neo Spark Max
    public static final int SwerveBackRightDriveMotorCanID   = 12;
    public static final int SwerveBackRightSteerMotorCanID   = 13;
    public static final int CanCoderBackRight                = 21;

    //swerver motor 3 Back Left Neo Spark Max
    public static final int SwerveBackLeftDriveMotorCanID    = 14;
    public static final int SwerveBackLeftSteerMotorCanID    = 15;
    public static final int CanCoderBackLeft                 = 22;

    //swerver motor 4 Front Left Neo Spark Max
    public static final int SwerveFrontLeftDriveMotorCanID   = 16;
    public static final int SwerveFrontLeftSteerMotorCanID   = 17;
    public static final int CanCoderFrontLeft                = 23;

    //Flywheel motors Vortex Spark Flex
    public static final int FlywheelMotor1CanID              = 18;
    public static final int FlywheelMotor2CanID              = 19;

    // Indexer motor Rev Neo Spark Max
    public static final int IndexerMotor1CanID                = 28;
    public static final int IndexerMotor2CanID                = 31;

    //Intake spin motors Vortex Spark Flex
    public static final int IntakeSpinMotor1CanID            = 24;
    public static final int IntakeSpinMotor2CanID            = 25;


    //Intake deply motors Rev Neo Spark Max
    public static final int IntakeDeployMotor1CanID          = 26;
    public static final int IntakeDeployMotor2CanID          = 27;

    //Hopper motor Rev Neo Spark Max 
    //**needs updated to correct CAN ID if used */
    public static final int FuelAgitatorMotorCanID           = 01;

    //Launcher hood motor Rev Neo Spark Max
    public static final int LauncherHoodMotorCanID          = 33;

    //Climb motors Rev Neo Spark Max
    public static final int ClimbDeployMotorCanID            = 29;
    public static final int ClimbGoUpMotor1CanID              = 30;
    public static final int ClimbGoUpMotor2CanID              = 32;
  }


  public static class Climber {
    public static final double kGearRatio = 16.0;             // Example 16:1
    public static final double kDrumDiameterInches = 1.25;    // Diameter of winch or sprocket
    public static final double kMaxHeightInches = 18.0;      // Physical limit in inches
  }

  public static class IntakeDeploy {
    public static final double kGearRatio = 5.0;            // Example gearbox reduction
    public static final double kTravelPerRotation = 0.5;    // e.g., 0.5 inches per 1 rotation of the screw
    public static final double kExtendedInches = 8.0;       // How far to push out
    public static final double kMaxExtensionInches = 8.5;   // Physical stop
  }

  public static class Launcher {
    public static final double kHoodGearRatio = 100.0; // Example: 100:1 reduction
    public static final double kMaxHoodAngle = 45.0;  // Maximum degrees of travel
    public static final double kAngleFender = 10.0;   // Angle for shooting near the speaker
    public static final double kAnglePodium = 35.0;   // Angle for shooting from further away
  }
}
