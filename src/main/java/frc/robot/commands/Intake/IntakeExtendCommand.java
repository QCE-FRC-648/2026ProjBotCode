package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeDeploySubsystem;
import frc.robot.Constants;

public class IntakeExtendCommand extends Command {
    private final IntakeDeploySubsystem m_intake;

    public IntakeExtendCommand(IntakeDeploySubsystem subsystem) {
        m_intake = subsystem;
        addRequirements(m_intake);
    }

    @Override
    public void initialize() {
        m_intake.extend(); // This calls your existing method that uses setLinearPosition
    }

    @Override
    public boolean isFinished() {
        // Finishes when within 0.25 inches of full extension
        return Math.abs(m_intake.getPosition() - Constants.IntakeDeploy.kExtendedInches) < 0.25;
    }
}