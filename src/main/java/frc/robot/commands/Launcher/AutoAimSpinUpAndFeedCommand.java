package frc.robot.commands.Launcher;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.FuelAgitatorSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.LauncherHoodSubsystem;
import frc.robot.subsystems.LauncherSubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import java.util.Objects;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

/**
 * Spins up the launcher and feeds while auto-aiming the hood from robot pose to a target pose.
 * Target pose can default to the alliance hub or be provided explicitly.
 */
public class AutoAimSpinUpAndFeedCommand extends Command {
    private final LauncherSubsystem m_launcher;
    private final IndexerSubsystem m_indexer;
    private final FuelAgitatorSubsystem m_agitator;
    private final LauncherHoodSubsystem m_hood;
    private final SwerveSubsystem m_swerve;
    private final Supplier<Translation2d> m_targetSupplier;
    private final DoubleSupplier m_launcherRpm;
    private final DoubleSupplier m_indexerRpm;
    private final DoubleSupplier m_agitatorRpm;
    private boolean m_isFeeding = false;

    public AutoAimSpinUpAndFeedCommand(
            LauncherSubsystem launcher,
            IndexerSubsystem indexer,
            FuelAgitatorSubsystem agitator,
            LauncherHoodSubsystem hood,
            SwerveSubsystem swerve,
            double launcherRpm,
            double indexerRpm,
            double agitatorRpm) {
    this(launcher, indexer, agitator, hood, swerve,
        () -> launcherRpm, () -> indexerRpm, () -> agitatorRpm,
        AutoAimSpinUpAndFeedCommand::getAllianceHubPose);
    }

    public AutoAimSpinUpAndFeedCommand(
            LauncherSubsystem launcher,
            IndexerSubsystem indexer,
            FuelAgitatorSubsystem agitator,
            LauncherHoodSubsystem hood,
            SwerveSubsystem swerve,
            double launcherRpm,
            double indexerRpm,
            double agitatorRpm,
            Translation2d targetPose) {
    this(launcher, indexer, agitator, hood, swerve,
        () -> launcherRpm, () -> indexerRpm, () -> agitatorRpm,
        () -> Objects.requireNonNull(targetPose, "targetPose"));
    }

    public AutoAimSpinUpAndFeedCommand(
        LauncherSubsystem launcher,
        IndexerSubsystem indexer,
        FuelAgitatorSubsystem agitator,
        LauncherHoodSubsystem hood,
        SwerveSubsystem swerve,
        DoubleSupplier launcherRpm,
        DoubleSupplier indexerRpm,
        DoubleSupplier agitatorRpm) {
    this(launcher, indexer, agitator, hood, swerve, launcherRpm, indexerRpm, agitatorRpm,
        AutoAimSpinUpAndFeedCommand::getAllianceHubPose);
    }

    public AutoAimSpinUpAndFeedCommand(
        LauncherSubsystem launcher,
        IndexerSubsystem indexer,
        FuelAgitatorSubsystem agitator,
        LauncherHoodSubsystem hood,
        SwerveSubsystem swerve,
        DoubleSupplier launcherRpm,
        DoubleSupplier indexerRpm,
        DoubleSupplier agitatorRpm,
        Translation2d targetPose) {
    this(launcher, indexer, agitator, hood, swerve, launcherRpm, indexerRpm, agitatorRpm,
        () -> Objects.requireNonNull(targetPose, "targetPose"));
    }

    public AutoAimSpinUpAndFeedCommand(
            LauncherSubsystem launcher,
            IndexerSubsystem indexer,
            FuelAgitatorSubsystem agitator,
            LauncherHoodSubsystem hood,
            SwerveSubsystem swerve,
        DoubleSupplier launcherRpm,
        DoubleSupplier indexerRpm,
        DoubleSupplier agitatorRpm,
            Supplier<Translation2d> targetSupplier) {
        m_launcher = launcher;
        m_indexer = indexer;
        m_agitator = agitator;
        m_hood = hood;
        m_swerve = swerve;
        m_launcherRpm = launcherRpm;
        m_indexerRpm = indexerRpm;
        m_agitatorRpm = agitatorRpm;
        m_targetSupplier = Objects.requireNonNull(targetSupplier, "targetSupplier");

        addRequirements(m_launcher, m_indexer, m_agitator, m_hood);
    }

    @Override
    public void initialize() {
        m_isFeeding = false;
    m_launcher.setVelocity(m_launcherRpm.getAsDouble());
    }

    @Override
    public void execute() {
        m_hood.setAngleFromPose(m_swerve.getPose().getTranslation(), m_targetSupplier.get());
    m_launcher.setVelocity(m_launcherRpm.getAsDouble());

        if (!m_isFeeding && m_launcher.isAtTarget()) {
            m_isFeeding = true;
        }

        if (m_isFeeding) {
            m_indexer.setVelocity(m_indexerRpm.getAsDouble());
            m_agitator.setVelocity(m_agitatorRpm.getAsDouble());
        }
    }

    @Override
    public void end(boolean interrupted) {
        m_launcher.stop();
        m_indexer.stop();
        m_agitator.stop();
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    private static Translation2d getAllianceHubPose() {
        boolean isRed = DriverStation.getAlliance().isPresent()
            && DriverStation.getAlliance().get() == DriverStation.Alliance.Red;
        return (isRed ? Constants.FieldObjectLocations.RED_HUB : Constants.FieldObjectLocations.BLUE_HUB).pos;
    }
}