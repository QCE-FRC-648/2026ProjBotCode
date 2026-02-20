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
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import java.io.File;
import swervelib.SwerveInputStream;


import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeDeploySubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LauncherHoodSubsystem;
import frc.robot.subsystems.LauncherSubsystem;
import frc.robot.subsystems.FuelAgitatorSubsystem;
import frc.robot.subsystems.swervedrive.*;
import frc.robot.commands.Intake.IntakeHomingCommand;
import frc.robot.commands.Intake.SmartAgitateCommand;
import frc.robot.commands.Climber.ClimberHomingCommand;
import frc.robot.commands.SwervedriveCommands.auto.*;
import frc.robot.commands.SwervedriveCommands.drivebase.*;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
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
  public static SwerveSubsystem driveTrain = new SwerveSubsystem();
  
  // Mechanism Subsystems
  private final LauncherSubsystem m_launcher = new LauncherSubsystem();
  private final IndexerSubsystem m_indexer = new IndexerSubsystem();
  private final IntakeSubsystem m_intake = new IntakeSubsystem();
  private final IntakeDeploySubsystem m_intakeDeploy = new IntakeDeploySubsystem();
  private final ClimberSubsystem m_climber = new ClimberSubsystem();
  private final FuelAgitatorSubsystem m_fuelAgitator = new FuelAgitatorSubsystem();
  private final LauncherHoodSubsystem m_hood = new LauncherHoodSubsystem();


  //Define Controllers
  public static CommandXboxController driverController = new CommandXboxController(0);
  public static CommandXboxController operatorController = new CommandXboxController(1);

  //Auto Chooser
  private final SendableChooser<Command> autoChooser;

  /**
   * Converts driver input into a field-relative ChassisSpeeds that is controlled by angular velocity.
   */
  SwerveInputStream driveAngularVelocity = SwerveInputStream.of(driveTrain.getSwerveDrive(),
                                                                () -> driverController.getLeftY() * -1,
                                                                () -> driverController.getLeftX() * -1)
                                                            .withControllerRotationAxis(() -> driverController.getRightX() * -1)
                                                            .deadband(OperatorConstants.DEADBAND)
                                                            .cubeTranslationControllerAxis(true)
                                                            .cubeRotationControllerAxis(true)
                                                            .scaleTranslation(.8)
                                                            .allianceRelativeControl(true);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer()
  {
    // Configure the trigger bindings
    configureNamedCommands();
    configureBindings();
    DriverStation.silenceJoystickConnectionWarning(true);
    autoChooser = AutoBuilder.buildAutoChooser("Pass The Line Auto");
    SmartDashboard.putData("Auto Mode", autoChooser);
    Command driveFieldOrientedAnglularVelocity = driveTrain.driveFieldOriented(driveAngularVelocity);

    driveTrain.setDefaultCommand(driveFieldOrientedAnglularVelocity);
  }
 
  private void configureNamedCommands() {
    // Register commands for use in PathPlanner Event Markers
    NamedCommands.registerCommand("HomeAll", getHomingSequence());
    NamedCommands.registerCommand("IntakeExtend", new InstantCommand(m_intakeDeploy::extend));
    NamedCommands.registerCommand("IntakeRetract", new InstantCommand(m_intakeDeploy::retract));
    NamedCommands.registerCommand("LaunchPrep", new InstantCommand(() -> m_launcher.setVelocity(4500)));
    NamedCommands.registerCommand("ClimbUp", new InstantCommand(() -> m_climber.setHeight(Constants.Climber.kMaxHeightInches)));
    NamedCommands.registerCommand("ClimbDown", new InstantCommand(() -> m_climber.setHeight(0)));
    NamedCommands.registerCommand("AgitateFuel", new SmartAgitateCommand(m_fuelAgitator));
  }

  /*
   private void configureNamedCommands() {
    NamedCommands.registerCommand("HomeAll", getHomingSequence());
    
    // INTAKE: Wait until fully extended/retracted
    NamedCommands.registerCommand("IntakeExtend", 
        Commands.runOnce(m_intakeDeploy::extend, m_intakeDeploy)
        .andThen(Commands.waitUntil(() -> Math.abs(m_intakeDeploy.getPosition() - Constants.Intake.kExtendedInches) < 0.5))
    );

    NamedCommands.registerCommand("IntakeRetract", 
        Commands.runOnce(m_intakeDeploy::retract, m_intakeDeploy)
        .andThen(Commands.waitUntil(() -> Math.abs(m_intakeDeploy.getPosition() - 0) < 0.5))
    );

    // CLIMBER: Wait until height is reached
    NamedCommands.registerCommand("ClimbUp", 
        Commands.runOnce(() -> m_climber.setHeight(Constants.Climber.kMaxHeightInches), m_climber)
        .andThen(Commands.waitUntil(() -> Math.abs(m_climber.getPosition() - Constants.Climber.kMaxHeightInches) < 0.5))
    );

    // LAUNCHER: Wait until flywheels are at full speed before moving to the next path step
    NamedCommands.registerCommand("LaunchPrep", 
        Commands.runOnce(() -> m_launcher.setVelocity(4500), m_launcher)
        .andThen(Commands.waitUntil(m_launcher::isAtTarget))
    );
  }
   */

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary predicate, or via the
   * named factories in {@link edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller PS4}
   * controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight joysticks}.
   */
  private void configureBindings() {
    // Operator: Manual Homing (Emergency/Reset)
    operatorController.start().whileTrue(new IntakeHomingCommand(m_intakeDeploy).withTimeout(2.0));
    operatorController.back().whileTrue(new ClimberHomingCommand(m_climber).withTimeout(2.0));

    // Operator: Intake Control
    operatorController.rightBumper().onTrue(new InstantCommand(m_intakeDeploy::extend));
    operatorController.leftBumper().onTrue(new InstantCommand(m_intakeDeploy::retract));

    // Operator: Launcher Control (Hold A to spin up)
    operatorController.a().whileTrue(new RunCommand(() -> m_launcher.setVelocity(4000), m_launcher))
                         .onFalse(new InstantCommand(m_launcher::stop));

    operatorController.b().whileTrue(new SmartAgitateCommand(m_fuelAgitator));
  
    // Use D-Pad for quick angle presets
    operatorController.povUp().onTrue(new InstantCommand(() -> m_hood.setAngle(Constants.Launcher.kAnglePodium)));
    operatorController.povDown().onTrue(new InstantCommand(() -> m_hood.setAngle(Constants.Launcher.kAngleFender)));
    operatorController.povLeft().onTrue(new InstantCommand(() -> m_hood.setAngle(0))); // Stowed

  }
     
  /**
   * Returns a command that homes both the intake and climber in parallel.
   */
  public Command getHomingSequence() {
    return new ParallelCommandGroup(
        new IntakeHomingCommand(m_intakeDeploy).withTimeout(2.5),
        new ClimberHomingCommand(m_climber).withTimeout(2.5)
    );
  }

  public void setMotorBrake(boolean brake)
  {
    driveTrain.setMotorBrake(brake);
  }
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // This creates a sequence that HOMES first, then runs the PathPlanner Auto
    return new SequentialCommandGroup(
        getHomingSequence(),
        autoChooser.getSelected()
    );
  }
}

