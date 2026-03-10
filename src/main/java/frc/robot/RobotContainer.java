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
import frc.robot.commands.Launcher.AutoAimSpinUpAndFeedCommand;
import frc.robot.commands.Launcher.SpinUpAndFeedCommand;
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
  private static final String kLauncherRpmKey = "Tuning/Launcher RPM";
  private static final String kIndexerRpmKey = "Tuning/Indexer RPM";
  private static final String kAgitatorRpmKey = "Tuning/Agitator RPM";
  private static final String kIntakeRpmKey = "Tuning/Intake RPM";

  private static final double kLauncherRpmDefault = 4500;
  private static final double kIndexerRpmDefault = 2000;
  private static final double kAgitatorRpmDefault = 3000;
  private static final double kIntakeRpmDefault = 3000;
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
    initTuningDashboard();
    // Configure the trigger bindings
    configureNamedCommands();
    configureBindings();
    DriverStation.silenceJoystickConnectionWarning(true);
    autoChooser = AutoBuilder.buildAutoChooser("Pass The Line Auto");
    SmartDashboard.putData("Auto Mode", autoChooser);
    Command driveFieldOrientedAnglularVelocity = driveTrain.driveFieldOriented(driveAngularVelocity);

    driveTrain.setDefaultCommand(driveFieldOrientedAnglularVelocity);
  }

  /**
   * Sync hood expected position to the current encoder reading. Call this when the robot is
   * enabled (auton/teleop/test) to prevent the hood from moving unexpectedly on enable.
   */
  public void syncHoodPosition() {
    m_hood.syncToEncoder();
  }
 
  private void configureNamedCommands() {
    // Register commands for use in PathPlanner Event Markers
    NamedCommands.registerCommand("HomeAll", getHomingSequence());
    NamedCommands.registerCommand("IntakeExtend", new InstantCommand(m_intakeDeploy::extend));
    NamedCommands.registerCommand("IntakeRetract", new InstantCommand(m_intakeDeploy::retract));
    NamedCommands.registerCommand("LaunchPrep", new InstantCommand(() ->
      m_launcher.setVelocity(MathUtil.clamp(getTuningNumber(kLauncherRpmKey, kLauncherRpmDefault), 0.0, Constants.Launcher.kMaxSafeRpm))));
    NamedCommands.registerCommand("ClimbUp", new InstantCommand(() -> m_climber.setHeight(Constants.Climber.kMaxHeightInches)));
    NamedCommands.registerCommand("ClimbDown", new InstantCommand(() -> m_climber.setHeight(0)));
    NamedCommands.registerCommand("AgitateFuel", new SmartAgitateCommand(m_fuelAgitator));
    NamedCommands.registerCommand("SpinIntake", new RunCommand(() ->
      m_intake.setVelocity(getTuningNumber(kIntakeRpmKey, kIntakeRpmDefault)), m_intake)
      .finallyDo(interrupted -> m_intake.stop()));
    NamedCommands.registerCommand("AutoHoodAngle", new InstantCommand(() -> {
      boolean isRed = DriverStation.getAlliance().isPresent() &&
                      DriverStation.getAlliance().get() == DriverStation.Alliance.Red;
      Constants.FieldObjectLocations.FieldTarget hub = isRed
          ? Constants.FieldObjectLocations.RED_HUB
          : Constants.FieldObjectLocations.BLUE_HUB;

      m_hood.setAngleFromPose(driveTrain.getPose().getTranslation(), hub.pos);
    }, m_hood));
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

    m_launcher.setDefaultCommand(new RunCommand(m_launcher::stop, m_launcher));
m_indexer.setDefaultCommand(new RunCommand(m_indexer::stop, m_indexer));
m_fuelAgitator.setDefaultCommand(new RunCommand(m_fuelAgitator::stop, m_fuelAgitator));
    // Operator: Manual climber control (left stick up/down)

    boolean isRed = DriverStation.getAlliance().isPresent() && 
                    DriverStation.getAlliance().get() == DriverStation.Alliance.Red;

    Constants.FieldObjectLocations.FieldTarget hub = isRed ? Constants.FieldObjectLocations.RED_HUB : Constants.FieldObjectLocations.BLUE_HUB;
    Constants.FieldObjectLocations.FieldTarget outpost = isRed ? Constants.FieldObjectLocations.RED_OUTPOST : Constants.FieldObjectLocations.BLUE_OUTPOST;
    Constants.FieldObjectLocations.FieldTarget tower = isRed ? Constants.FieldObjectLocations.RED_TOWER : Constants.FieldObjectLocations.BLUE_TOWER;
    Constants.FieldObjectLocations.FieldTarget depot = isRed ? Constants.FieldObjectLocations.RED_DEPOT : Constants.FieldObjectLocations.BLUE_DEPOT;

   
    
    // Aim the robot at the scoring station using the target constant
    driverController.rightTrigger().whileTrue(
        driveTrain.driveAndAim(driveAngularVelocity, hub));
    driverController.leftTrigger().whileTrue(
        driveTrain.driveAndAim(driveAngularVelocity, outpost));
    driverController.rightBumper().whileTrue(
        driveTrain.driveAndAim(driveAngularVelocity, tower));
    driverController.leftBumper().whileTrue(
        driveTrain.driveAndAim(driveAngularVelocity, depot));  

     // Operator Controls 
    m_climber.setDefaultCommand(new RunCommand(() -> {
      double speed = MathUtil.applyDeadband(-operatorController.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND);

      if (m_climber.isAtBottom() && speed < 0) {
        speed = 0;
      }

      if (m_climber.getPosition() >= Constants.Climber.kMaxHeightInches && speed > 0) {
        speed = 0;
      }

      m_climber.runManual(speed);
    }, m_climber));

    // Intake deploy: manual control using operator right stick X (left/right).
    // Pushing right -> extend, left -> retract. Applies safety stops using
    // the DIO-mounted limit switches on the intake deploy subsystem.
    m_intakeDeploy.setDefaultCommand(new RunCommand(() -> {
      double speed = MathUtil.applyDeadband(operatorController.getRightX(), OperatorConstants.RIGHT_X_DEADBAND);

      // If lower (retracted) limit is hit and driver requests further retract, stop.
      if (m_intakeDeploy.isLowerSwitchActive() && speed < 0) {
        speed = 0;
      }

      // If we're at or past the maximum extension and driver requests more extend, stop.
      if (m_intakeDeploy.getPosition() >= Constants.IntakeDeploy.kMaxExtensionInches && speed > 0) {
        speed = 0;
      }

      m_intakeDeploy.runAtPower(speed);
    }, m_intakeDeploy));

    // Operator: Manual Homing (Emergency/Reset)
    operatorController.start().whileTrue(new IntakeHomingCommand(m_intakeDeploy).withTimeout(2.0));
    operatorController.back().whileTrue(new ClimberHomingCommand(m_climber).withTimeout(2.0));

    // Operator: Intake Control
    operatorController.rightBumper().onTrue(new InstantCommand(m_intakeDeploy::extend));
    operatorController.leftBumper().onTrue(new InstantCommand(m_intakeDeploy::retract));

    // Intake spin: Right trigger spins the intake forward at the tuned RPM.
    // Left trigger spins the intake in reverse at a reduced magnitude for safety.
    // Reverse is intentionally scaled to 50% to reduce mechanical stress and
    // avoid ejecting game pieces violently. Adjust the scale as needed.
    //intake
    operatorController.rightTrigger().whileTrue(new RunCommand(() ->
      m_intake.setVelocity(getTuningNumber(kIntakeRpmKey, kIntakeRpmDefault)), m_intake))
      .onFalse(new InstantCommand(m_intake::stop));
    //extract
    operatorController.leftTrigger().whileTrue(new RunCommand(() ->
      m_intake.setVelocity(-0.5 * getTuningNumber(kIntakeRpmKey, kIntakeRpmDefault)), m_intake))
      .onFalse(new InstantCommand(m_intake::stop));

    // Operator: Launcher Control (Hold A to spin up)
    // Create reusable RunCommand instances so the command lifecycle (and finallyDo) is consistent
    Command launcherHold = new RunCommand(() ->
      m_launcher.setVelocity(MathUtil.clamp(getTuningNumber(kLauncherRpmKey, kLauncherRpmDefault), 0.0, Constants.Launcher.kMaxSafeRpm)), m_launcher)
        .finallyDo(interrupted -> m_launcher.stop());

    // Operator: Individual mechanism manual spin
    // B -> spin the INDEXER at the tuned indexer RPM while held
    Command indexerHold = new RunCommand(() ->
      m_indexer.setVelocity(MathUtil.clamp(getTuningNumber(kIndexerRpmKey, kIndexerRpmDefault), 0.0, Constants.Indexer.kMaxSafeRpm)), m_indexer)
        .finallyDo(interrupted -> m_indexer.stop());

    // X -> spin the FUEL AGITATOR at the tuned agitator RPM while held
    Command agitatorHold = new RunCommand(() ->
      m_fuelAgitator.setVelocity(MathUtil.clamp(getTuningNumber(kAgitatorRpmKey, kAgitatorRpmDefault), 0.0, Constants.Agitator.kMaxSafeRpm)), m_fuelAgitator)
        .finallyDo(interrupted -> m_fuelAgitator.stop());

    // Bind the triggers to the reusable commands (hold-to-run)
    operatorController.a().whileTrue(launcherHold).onFalse(new InstantCommand(m_launcher::stop,m_launcher));
    operatorController.b().whileTrue(indexerHold).onFalse(new InstantCommand(m_indexer::stop,m_indexer));
    operatorController.x().whileTrue(agitatorHold).onFalse(new InstantCommand(m_fuelAgitator::stop,m_fuelAgitator));

  // Debugging: log press/release events for A/B/X to help diagnose toggle-like behavior
  operatorController.a().onTrue(new InstantCommand(() -> DriverStation.reportWarning("Operator A pressed", false)));
  operatorController.a().onFalse(new InstantCommand(() -> DriverStation.reportWarning("Operator A released", false)));

  operatorController.b().onTrue(new InstantCommand(() -> DriverStation.reportWarning("Operator B pressed", false)));
  operatorController.b().onFalse(new InstantCommand(() -> DriverStation.reportWarning("Operator B released", false)));

  operatorController.x().onTrue(new InstantCommand(() -> DriverStation.reportWarning("Operator X pressed", false)));
  operatorController.x().onFalse(new InstantCommand(() -> DriverStation.reportWarning("Operator X released", false)));

    //operatorController.b().whileTrue(new SmartAgitateCommand(m_fuelAgitator));

  // Operator: Spin up launcher, then feed indexer + agitator
   operatorController.y().whileTrue(
      new SpinUpAndFeedCommand(
        m_launcher,
        m_indexer,
        m_fuelAgitator,
        () -> MathUtil.clamp(getTuningNumber(kLauncherRpmKey, kLauncherRpmDefault), 0.0, Constants.Launcher.kMaxSafeRpm),
        () -> MathUtil.clamp(getTuningNumber(kIndexerRpmKey, kIndexerRpmDefault), 0.0, Constants.Indexer.kMaxSafeRpm),
        () -> MathUtil.clamp(getTuningNumber(kAgitatorRpmKey, kAgitatorRpmDefault), 0.0, Constants.Agitator.kMaxSafeRpm)));

    // Operator: Auto-aim hood + spin up + feed (default hub)
    /*
    operatorController.rightTrigger().whileTrue(
      new AutoAimSpinUpAndFeedCommand(
        m_launcher,
        m_indexer,
        m_fuelAgitator,
        m_hood,
        driveTrain,
        () -> getTuningNumber(kLauncherRpmKey, kLauncherRpmDefault),
        () -> getTuningNumber(kIndexerRpmKey, kIndexerRpmDefault),
        () -> getTuningNumber(kAgitatorRpmKey, kAgitatorRpmDefault)));

    // Operator: Auto-aim hood + spin up + feed (hard-coded target at 2m,2m)
    operatorController.leftTrigger().whileTrue(
      new AutoAimSpinUpAndFeedCommand(
        m_launcher,
        m_indexer,
        m_fuelAgitator,
        m_hood,
        driveTrain,
        () -> getTuningNumber(kLauncherRpmKey, kLauncherRpmDefault),
        () -> getTuningNumber(kIndexerRpmKey, kIndexerRpmDefault),
        () -> getTuningNumber(kAgitatorRpmKey, kAgitatorRpmDefault),
        new Translation2d(2.0, 2.0)));
    */
    // Use D-Pad for quick angle presets
    //operatorController.povUp().onTrue(new InstantCommand(() -> m_hood.setAngle(Constants.Launcher.kAnglePodium)));
    //operatorController.povDown().onTrue(new InstantCommand(() -> m_hood.setAngle(Constants.Launcher.kAngleFender)));
    //operatorController.povLeft().onTrue(new InstantCommand(() -> m_hood.setAngle(0))); // Stowed
    // step in degrees for each POV press

    // Increase hood angle on POV up
    operatorController.povUp().onTrue(
      new InstantCommand(() -> m_hood.setAngle(m_hood.getAngle() +  Constants.Launcher.khoodStepDeg)));

    // Decrease hood angle on POV down
    operatorController.povDown().onTrue(
      new InstantCommand(() -> m_hood.setAngle(m_hood.getAngle() - Constants.Launcher.khoodStepDeg)));
  }

  private void initTuningDashboard() {
    SmartDashboard.putNumber(kLauncherRpmKey, SmartDashboard.getNumber(kLauncherRpmKey, kLauncherRpmDefault));
    SmartDashboard.putNumber(kIndexerRpmKey, SmartDashboard.getNumber(kIndexerRpmKey, kIndexerRpmDefault));
    SmartDashboard.putNumber(kAgitatorRpmKey, SmartDashboard.getNumber(kAgitatorRpmKey, kAgitatorRpmDefault));
    SmartDashboard.putNumber(kIntakeRpmKey, SmartDashboard.getNumber(kIntakeRpmKey, kIntakeRpmDefault));
  }

  private double getTuningNumber(String key, double defaultValue) {
    return SmartDashboard.getNumber(key, defaultValue);
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
        //getHomingSequence(),
        autoChooser.getSelected()
    );
  }
}

