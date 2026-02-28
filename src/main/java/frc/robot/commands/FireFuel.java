package frc.robot.commands;
import frc.robot.RobotContainer;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
public class FireFuel extends Command {
    
    private final double FireRPM;

    /**
     * @param targetFireRPM target velocity in RPM for the flywheel
     */
    public FireFuel(double targetRpm) {
        this.FireRPM = targetRpm;
        // require the flywheel subsystem from RobotContainer
        addRequirements(RobotContainer.m_shoot_left);
    }

    @Override
    public void initialize() {
        // start the flywheel at the requested velocity
        RobotContainer.m_shoot_left.setVelocity(FireRPM);
       // DriverStation.reportInfo("Shoot.initialize(): targetRPM=" + rpm, false);
    }

    @Override
    public void execute() {
        // keep the flywheel at the requested velocity (controller handles it)
        RobotContainer.m_shoot_left.setVelocity(FireRPM);

       // DriverStation.reportInfo("Shoot.execute(): target=" + rpm + " actual=" + actual, false);
    }

    @Override
    public void end(boolean interrupted) {
        // stop the flywheel when the command ends
        RobotContainer.m_shoot_left.setVelocity(0);
        //DriverStation.reportInfo("Shoot.end(): interrupted=" + interrupted, false);
    }

    @Override
    public boolean isFinished() {
        // runs until explicitly canceled
        return false;
    }
}
