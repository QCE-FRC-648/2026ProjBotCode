package frc.robot.commands.Launcher;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.FuelAgitatorSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.LauncherSubsystem;
import java.util.function.DoubleSupplier;

/**
 * Spins the launcher to a target RPM, then feeds using the indexer and fuel agitator.
 * This command runs until interrupted.
 */
public class SpinUpAndFeedCommand extends Command {
    private final LauncherSubsystem m_launcher;
    private final IndexerSubsystem m_indexer;
    private final FuelAgitatorSubsystem m_agitator;
    private final DoubleSupplier m_launcherRpm;
    private final DoubleSupplier m_indexerRpm;
    private final DoubleSupplier m_agitatorRpm;
    private boolean m_isFeeding = false;

    public SpinUpAndFeedCommand(
            LauncherSubsystem launcher,
            IndexerSubsystem indexer,
            FuelAgitatorSubsystem agitator,
            double launcherRpm,
            double indexerRpm,
            double agitatorRpm) {
        this(launcher, indexer, agitator, () -> launcherRpm, () -> indexerRpm, () -> agitatorRpm);
    }

    public SpinUpAndFeedCommand(
            LauncherSubsystem launcher,
            IndexerSubsystem indexer,
            FuelAgitatorSubsystem agitator,
            DoubleSupplier launcherRpm,
            DoubleSupplier indexerRpm,
            DoubleSupplier agitatorRpm) {
        m_launcher = launcher;
        m_indexer = indexer;
        m_agitator = agitator;
        m_launcherRpm = launcherRpm;
        m_indexerRpm = indexerRpm;
        m_agitatorRpm = agitatorRpm;

        addRequirements(m_launcher, m_indexer, m_agitator);
    }

    @Override
    public void initialize() {
        m_isFeeding = false;
    m_launcher.setVelocity(m_launcherRpm.getAsDouble());
    }

    @Override
    public void execute() {
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
}