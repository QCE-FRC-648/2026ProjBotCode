package frc.robot.commands;


 

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.RobotContainer;

/** Command to run the flywheel at a target RPM. */
public class intakeRun extends Command {
    private final double PickupRPM;

    /**
     * @param targetRpm target velocity in RPM for the flywheel
     */
    public intakeRun(double targetPickupRPM) {
        this.PickupRPM = targetPickupRPM;
        // require the flywheel subsystem from RobotContainer
        addRequirements(RobotContainer.m_Intake);
    }

    @Override
    public void initialize() {
        // start the flywheel at the requested velocity
        RobotContainer.m_Intake.setVelocity(PickupRPM);
        
       // DriverStation.reportInfo("Shoot.initialize(): targetRPM=" + rpm, false);
    }

    @Override
    public void execute() {
        // keep the flywheel at the requested velocity (controller handles it)
        RobotContainer.m_Intake.setVelocity(PickupRPM);
        // Actual velocity retrieval is not available on the intake subsystem; if needed add a getVelocity() method to the intake class.
        // DriverStation.reportInfo("Shoot.execute(): target=" + PickupRPM, false);
    }

    @Override
    public void end(boolean interrupted) {
        // stop the flywheel when the command ends
        RobotContainer.m_Intake.setVelocity(0);
        //DriverStation.reportInfo("Shoot.end(): interrupted=" + interrupted, false);
    }

    @Override
    public boolean isFinished() {
        // runs until explicitly canceled
        return false;
    }
}


