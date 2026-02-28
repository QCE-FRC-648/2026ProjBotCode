package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.RobotContainer;

/** Command to run the flywheel at a target RPM. */
public class Shoot extends Command {
    private final double rpm;

    /**
     * @param targetRpm target velocity in RPM for the flywheel
     */
    public Shoot(double targetRpm) {
        this.rpm = targetRpm;
        // require the flywheel subsystem from RobotContainer
        addRequirements(RobotContainer.m_flywheel);
    }

    @Override
    public void initialize() {
        // start the flywheel at the requested velocity
        RobotContainer.m_flywheel.setVelocity(rpm);
        SmartDashboard.putNumber("Flywheel/TargetRPM", rpm);
       // DriverStation.reportInfo("Shoot.initialize(): targetRPM=" + rpm, false);
    }

    @Override
    public void execute() {
        // keep the flywheel at the requested velocity (controller handles it)
        RobotContainer.m_flywheel.setVelocity(rpm);
        double actual = RobotContainer.m_flywheel.getVelocity();
        SmartDashboard.putNumber("Flywheel/ActualRPM", actual);
       // DriverStation.reportInfo("Shoot.execute(): target=" + rpm + " actual=" + actual, false);
    }

    @Override
    public void end(boolean interrupted) {
        // stop the flywheel when the command ends
        RobotContainer.m_flywheel.setVelocity(0);
        //DriverStation.reportInfo("Shoot.end(): interrupted=" + interrupted, false);
    }

    @Override
    public boolean isFinished() {
        // runs until explicitly canceled
        return false;
    }
}
