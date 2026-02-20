package frc.robot.commands.Intake;


import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.FuelAgitatorSubsystem;

public class SmartAgitateCommand extends Command {
    private final FuelAgitatorSubsystem m_agitator;
    private final Timer m_unjamTimer = new Timer();
    private boolean m_isUnjamming = false;

    public SmartAgitateCommand(FuelAgitatorSubsystem subsystem) {
        m_agitator = subsystem;
        addRequirements(m_agitator);
    }

    @Override
    public void execute() {
        if (!m_isUnjamming) {
            if (m_agitator.isStalled()) {
                m_isUnjamming = true;
                m_unjamTimer.reset();
                m_unjamTimer.start();
            } else {
                m_agitator.setVelocity(3000); // Normal forward speed
            }
        } else {
            // Reversing logic
            m_agitator.setVelocity(-1500); // Pulse backward to clear jam
            if (m_unjamTimer.hasElapsed(0.5)) { // Reverse for half a second
                m_isUnjamming = false;
                m_unjamTimer.stop();
            }
        }

        if (m_isUnjamming) {
            SmartDashboard.putString("Intake Status", "JAM DETECTED - CLEARING");
        } else {
            SmartDashboard.putString("Intake Status", "Normal");
        }
    }

    @Override
    public void end(boolean interrupted) {
        m_agitator.stop();
    }
}