package frc.robot.commands.Intake;


import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeDeploySubsystem;

public class IntakeHomingCommand extends Command {
    private final IntakeDeploySubsystem m_intakeDeploy;

    public IntakeHomingCommand(IntakeDeploySubsystem subsystem) {
        this.m_intakeDeploy = subsystem;
        addRequirements(m_intakeDeploy);
    }

    @Override
    public void execute() {
        // Move slowly backward (-10% power)
        m_intakeDeploy.runAtPower(-0.1);
    }

    @Override
    public boolean isFinished() {
        // Stop when the reverse limit switch is hit
        return m_intakeDeploy.isReverseLimitPressed();
    }

    @Override
    public void end(boolean interrupted) {
        m_intakeDeploy.stopPivot();
        if (!interrupted && m_intakeDeploy.isReverseLimitPressed()) {
            m_intakeDeploy.resetEncoder();
        }
    }
}
