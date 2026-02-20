// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.CANConfig;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import java.io.File;
import java.util.Collection;

import swervelib.SwerveInputStream;
import yams.mechanisms.velocity.FlyWheel;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.swervedrive.*;
import frc.robot.commands.SwervedriveCommands.auto.*;
import frc.robot.commands.SwervedriveCommands.drivebase.*;
import frc.robot.subsystems.ShootFuel;


import frc.robot.subsystems.flywheel;
import frc.robot.subsystems.intake;
import frc.robot.commands.Shoot;
import frc.robot.commands.intakeRun;

import com.pathplanner.lib.auto.AutoBuilder;
import frc.robot.commands.FireFuel;
import frc.robot.subsystems.ShootFuel;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
// import frc.robot.subsystems.drive.DriveSubsystem;


//https://docs.wpilib.org/en/stable/docs/software/hardware-apis/misc/addressable-leds.html

 /* This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer
{
  //Define Subsystems
  //public static SwerveSubsystem driveTrain = new SwerveSubsystem();
  public static flywheel m_flywheel = new flywheel();
  public static ShootFuel m_shoot_left = new ShootFuel();
  public static intake m_Intake = new intake();

  
  
  //Define Controllers
  public static CommandXboxController driverController = new CommandXboxController(0);
  public static CommandXboxController operatorController = new CommandXboxController(1);

  //Auto Chooser
  //private final SendableChooser<Command> autoChooser;

  /**
   * Converts driver input into a field-relative ChassisSpeeds that is controlled by angular velocity.
   */
  //SwerveInputStream driveAngularVelocity = SwerveInputStream.of(driveTrain.getSwerveDrive(),
                                                            //     () -> driverController.getLeftY() * -1,
                                                            //     () -> driverController.getLeftX() * -1)
                                                            // .withControllerRotationAxis(() -> driverController.getRightX() * -1)
                                                            // .deadband(OperatorConstants.DEADBAND)
                                                            // .cubeTranslationControllerAxis(true)
                                                            // .cubeRotationControllerAxis(true)
                                                            // .scaleTranslation(.8)
                                                            // .allianceRelativeControl(true);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer()
  {
    // Configure the trigger bindings
    configureBindings();
    DriverStation.silenceJoystickConnectionWarning(true);
      // Startup logging to help debug subsystems
      SmartDashboard.putBoolean("Robot/Started", true);
      //DriverStation.reportWarning("RobotContainer initialized; flywheel CAN ID = " + Constants.CANConfig.FLYWHEEL_LEFT, false);
    // autoChooser = AutoBuilder.buildAutoChooser("Pass The Line Auto");
    // SmartDashboard.putData("Auto Mode", autoChooser);
    // Command driveFieldOrientedAnglularVelocity = driveTrain.driveFieldOriented(driveAngularVelocity);

    // driveTrain.setDefaultCommand(driveFieldOrientedAnglularVelocity);
  }
 
  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary predicate, or via the
   * named factories in {@link edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller PS4}
   * controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight joysticks}.
   */
  
  private void configureBindings()
  {
  // Bind operator right trigger to run the flywheel while held
  operatorController.rightBumper().whileTrue(new Shoot(2000));
  operatorController.leftBumper().whileTrue(new FireFuel(2000));
  operatorController.leftTrigger().whileTrue(new intakeRun(2000));
  }
  //public void setMotorBrake(boolean brake)
  {
    // driveTrain.setMotorBrake(brake);
  }
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand()
  {
    // Pass the auto line for points
    //return autoChooser.getSelected();
    // return driveTrain.getAutonomousCommand("Pass The Line Auto");
    return null;
  }
}
