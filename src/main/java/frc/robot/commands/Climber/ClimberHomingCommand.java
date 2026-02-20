package frc.robot.commands.Climber;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimberSubsystem;

public class ClimberHomingCommand extends Command {
    private final ClimberSubsystem m_climber;

    public ClimberHomingCommand(ClimberSubsystem subsystem) {
        this.m_climber = subsystem;
        // Ensure this command has exclusive access to the climber while running
        addRequirements(m_climber);
    }

    @Override
    public void initialize() {
        // Optional: Any setup before movement begins
    }

    @Override
    public void execute() {
        // Drive the climber down slowly toward the limit switch
        // Use a negative value if your climber moves down with negative power
        m_climber.runManual(-0.15); 
    }

    @Override
    public boolean isFinished() {
        // The command ends when the bottom limit switch is triggered
        return m_climber.isAtBottom();
    }

    @Override
    public void end(boolean interrupted) {
        m_climber.stop();
        
        // Only reset the encoder if we actually hit the switch (not if interrupted or timed out)
        if (!interrupted && m_climber.isAtBottom()) {
            m_climber.resetEncoder();
        }
    }
}