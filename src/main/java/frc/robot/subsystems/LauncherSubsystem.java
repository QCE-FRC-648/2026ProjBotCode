package frc.robot.subsystems;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkBase.PersistMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class LauncherSubsystem extends SubsystemBase {

    private final SparkFlex flywheelMotor1;
    private final SparkFlex flywheelMotor2;
    private final SparkClosedLoopController velocityController;
    private final RelativeEncoder flywheelEncoder;

    private double targetRPM = 0;
    private final double VELOCITY_TOLERANCE = 100.0;

    private SparkFlexConfig flywheelMotor1Config = new SparkFlexConfig();
    private SparkFlexConfig flywheelMotor2Config = new SparkFlexConfig();


    public LauncherSubsystem() {
        flywheelMotor1 = new SparkFlex(Constants.CanConstants.FlywheelMotor1CanID, MotorType.kBrushless);
        flywheelMotor2 = new SparkFlex(Constants.CanConstants.FlywheelMotor2CanID, MotorType.kBrushless);

        velocityController = flywheelMotor1.getClosedLoopController();
        flywheelEncoder = flywheelMotor1.getEncoder();

        // Configure motors if needed
        flywheelMotor1Config
            .idleMode(IdleMode.kCoast)
            .smartCurrentLimit(40)
            .openLoopRampRate(.25)
            .closedLoopRampRate(.25);

        flywheelMotor2Config
            .idleMode(IdleMode.kCoast)
            .smartCurrentLimit(40)
            .follow(flywheelMotor1, true);

        flywheelMotor1Config.closedLoop
            .p(0.0001)
            .velocityFF(0.00017);

        flywheelMotor1.configure(flywheelMotor1Config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        flywheelMotor2.configure(flywheelMotor2Config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

   public void setVelocity(double rpm) {
        this.targetRPM = rpm;
        velocityController.setReference(rpm, SparkFlex.ControlType.kVelocity);
    }

    public void stop() {
        this.targetRPM = 0;
        flywheelMotor1.stopMotor();
    }

    public double getActualRPM() {
        return flywheelEncoder.getVelocity();
    }

    public boolean isAtTarget() {
        return targetRPM > 0 && Math.abs(getActualRPM() - targetRPM) < VELOCITY_TOLERANCE;
    }

    @Override
    public void periodic() {
        // Elastic will pick these up automatically. 
        // Tip: Use a "/" to create a sub-folder in Elastic's network tree.
        SmartDashboard.putNumber("Launcher/Target RPM", targetRPM);
        SmartDashboard.putNumber("Launcher/Actual RPM", getActualRPM());
        SmartDashboard.putBoolean("Launcher/At Velocity", isAtTarget());
        SmartDashboard.putNumber("Launcher/Output Amps", flywheelMotor1.getOutputCurrent());
    }
}