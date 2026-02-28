package frc.robot.commands.Climber;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.Constants;

public class ClimberExtendCommand extends Command {
    private final ClimberSubsystem m_climber;

    public ClimberExtendCommand(ClimberSubsystem subsystem) {
        m_climber = subsystem;
        addRequirements(m_climber);
    }

    @Override
    public void initialize() {
        // Tell the PID controller to go to the max height constant
        m_climber.setHeight(Constants.Climber.kMaxHeightInches);
    }

    @Override
    public boolean isFinished() {
        // This command finishes once the climber is within a small threshold of the target
        // (Assuming you add an isAtTarget method, otherwise return true to make it an Instant action)
        return Math.abs(m_climber.getPosition() - Constants.Climber.kMaxHeightInches) < 0.5;
    }
}