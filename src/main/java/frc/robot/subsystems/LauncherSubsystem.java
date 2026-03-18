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
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class LauncherSubsystem extends SubsystemBase {

    private final SparkFlex flywheelMotor1;
    private final SparkFlex flywheelMotor2;
    private final SparkClosedLoopController velocityController;
    private final RelativeEncoder flywheelEncoder;
    private final RelativeEncoder flywheelEncoder2;

    private double targetRPM = 0;
    private final double VELOCITY_TOLERANCE = 500.0;

    // Ramp and clamping helpers
    private final SlewRateLimiter rpmSlew = new SlewRateLimiter(4000.0); // RPM per second
    private double desiredTargetRPM = 0.0;
    private double appliedTargetRPM = 0.0;

    private SparkFlexConfig flywheelMotor1Config = new SparkFlexConfig();
    private SparkFlexConfig flywheelMotor2Config = new SparkFlexConfig();
    // Live-tuning cache
    private double m_lastP = 0.0001;
    private double m_lastFF = 0.00015;
    private double m_lastD = 0.00;

    public LauncherSubsystem() {
        flywheelMotor1 = new SparkFlex(Constants.CanConstants.FlywheelMotor1CanID, MotorType.kBrushless);
        flywheelMotor2 = new SparkFlex(Constants.CanConstants.FlywheelMotor2CanID, MotorType.kBrushless);

        velocityController = flywheelMotor1.getClosedLoopController();
        flywheelEncoder = flywheelMotor1.getEncoder();
        flywheelEncoder2 = flywheelMotor2.getEncoder();

        // Configure motors if needed
        flywheelMotor1Config
            .idleMode(IdleMode.kCoast)
            .smartCurrentLimit(40)
            .openLoopRampRate(.25)
            .closedLoopRampRate(.25)
            .inverted(true);
            

        flywheelMotor2Config
            .idleMode(IdleMode.kCoast)
            .smartCurrentLimit(40)
            .follow(flywheelMotor1, true);

        flywheelMotor1Config.closedLoop
            .p(m_lastP)
            .d(m_lastD)
            .velocityFF(m_lastFF);

        flywheelMotor1.configure(flywheelMotor1Config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        flywheelMotor2.configure(flywheelMotor2Config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // Note: PID tuning values are intentionally kept in fields but we do not
    // expose live tuning via SmartDashboard in this build to avoid runtime
    // reconfiguration complexity. The initial closed-loop gains are still
    // applied from the cached m_last* values above.
    }

public void setVelocity(double rpm) {
    double max = Constants.Launcher.kMaxSafeRpm;
    // Clamp to safe range
    double clamped = Math.signum(rpm) * Math.min(Math.abs(rpm), max);
    
    this.targetRPM = clamped;
    this.desiredTargetRPM = clamped;
}

    public void stop() {
    this.targetRPM = 0;
    this.desiredTargetRPM = 0; // CRITICAL: Tells periodic() to stop PID
    this.appliedTargetRPM = 0;
    rpmSlew.reset(0);           // Resets the "memory" of the ramp
    flywheelMotor1.stopMotor(); // Immediate hardware stop
}

    public double getActualRPM() {
        return flywheelEncoder.getVelocity();
    }

    public boolean isAtTarget() {
        return targetRPM > 0 && Math.abs(getActualRPM() - targetRPM) < VELOCITY_TOLERANCE;
    }

    @Override
    public void periodic() {
    
    // 1. Calculate the ramped value based on our goal
    double rampedValue = rpmSlew.calculate(desiredTargetRPM);

    // 2. The Guard Clause: 
    // If the goal is 0, stay stopped. Otherwise, run the PID.
    if (desiredTargetRPM == 0) {
        flywheelMotor1.stopMotor();
    } else {
        velocityController.setReference(rampedValue, SparkFlex.ControlType.kVelocity);
        appliedTargetRPM = rampedValue;
    }

    // Logging for debugging the "Toggle" feel
    SmartDashboard.putNumber("Launcher/Actual RPM", flywheelEncoder.getVelocity());
    SmartDashboard.putNumber("Launcher/Target RPM", targetRPM);
    SmartDashboard.putNumber("Launcher/Applied RPM", appliedTargetRPM);
    SmartDashboard.putBoolean("Launcher/At Velocity", isAtTarget());

            // PID live-tuning has been removed. Gains are set from the cached
            // m_lastP/m_lastFF/m_lastD values at initialization and are not
            // re-read from SmartDashboard during runtime.
    }
}