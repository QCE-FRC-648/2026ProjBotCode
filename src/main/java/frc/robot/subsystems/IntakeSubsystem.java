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

public class IntakeSubsystem extends SubsystemBase {

    private final SparkFlex IntakeSpinMotor1;
    private final SparkFlex IntakeSpinMotor2;
    private final SparkClosedLoopController velocityController;
    private final RelativeEncoder IntakeSpinEncoder;

    private double targetRPM = 0;
    private final double VELOCITY_TOLERANCE = 100.0;
    // Last open-loop percent commanded (leader motor). Updated by runAtPercent()
    private double lastPercentCommanded = 0.0;

    // Live-tuning cache for closed-loop gains
    private double m_lastP = 0.0001;
    private double m_lastFF = 0.00017;

    private SparkFlexConfig IntakeSpinMotor1Config = new SparkFlexConfig();
    private SparkFlexConfig IntakeSpinMotor2Config = new SparkFlexConfig();


    public IntakeSubsystem() {
        IntakeSpinMotor1 = new SparkFlex(Constants.CanConstants.IntakeSpinMotor1CanID, MotorType.kBrushless);
        IntakeSpinMotor2 = new SparkFlex(Constants.CanConstants.IntakeSpinMotor2CanID, MotorType.kBrushless);

        velocityController = IntakeSpinMotor1.getClosedLoopController();
        IntakeSpinEncoder = IntakeSpinMotor1.getEncoder();

        // Configure motors if needed
        IntakeSpinMotor1Config
            .idleMode(IdleMode.kCoast)
            .smartCurrentLimit(40)
            .openLoopRampRate(.25)
            .closedLoopRampRate(.25)
            .inverted(true);

        IntakeSpinMotor2Config
            .idleMode(IdleMode.kCoast)
            .smartCurrentLimit(40)
            .follow(IntakeSpinMotor1, true);

        IntakeSpinMotor1Config.closedLoop
            .p(0.0001)
            .velocityFF(0.00017);

        IntakeSpinMotor1.configure(IntakeSpinMotor1Config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        IntakeSpinMotor2.configure(IntakeSpinMotor2Config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // Publish initial tuning values to SmartDashboard for live tuning
        SmartDashboard.putNumber("Intake/P", m_lastP);
        SmartDashboard.putNumber("Intake/FF", m_lastFF);
    }

   public void setVelocity(double rpm) {
        this.targetRPM = rpm;
        velocityController.setReference(rpm, SparkFlex.ControlType.kVelocity);
       // Clear open-loop percent when returning to closed-loop control
       lastPercentCommanded = 0.0;
    }

    public void stop() {
        this.targetRPM = 0;
        IntakeSpinMotor1.stopMotor();
        lastPercentCommanded = 0.0;
    }

    /**
     * Open-loop percent control for debugging/testing. percent is -1.0..1.0.
     *
     * NOTE: This helper is safe to use for short manual tests. It clears the
     * closed-loop target so periodic() telemetry won't be confused by a lingering
     * PID target. If you prefer to temporarily bind a controller button to this
     * behavior, see the commented example in RobotContainer.
     */
    public void runAtPercent(double percent) {
        // Disable closed-loop target so periodic() doesn't immediately reissue a PID command.
        this.targetRPM = 0;
        // Direct open-loop command to the leader motor; follower is configured to follow.
        IntakeSpinMotor1.set(percent);
        lastPercentCommanded = percent;
    }

    public double getActualRPM() {
        return IntakeSpinEncoder.getVelocity();
    }

    public boolean isAtTarget() {
        return targetRPM > 0 && Math.abs(getActualRPM() - targetRPM) < VELOCITY_TOLERANCE;
    }

    @Override
    public void periodic() {
        // Elastic will pick these up automatically. 
        // Tip: Use a "/" to create a sub-folder in Elastic's network tree.
        SmartDashboard.putNumber("Intake/Target RPM", targetRPM);
        SmartDashboard.putNumber("Intake/Actual RPM", getActualRPM());
        SmartDashboard.putBoolean("Intake/At Velocity", isAtTarget());
        SmartDashboard.putNumber("Intake/Output Amps", IntakeSpinMotor1.getOutputCurrent());
        SmartDashboard.putNumber("Intake/Percent", lastPercentCommanded);

        // Live tuning: read P/FF values and reconfigure controller when changed
        double newP = SmartDashboard.getNumber("Intake/P", m_lastP);
        double newFF = SmartDashboard.getNumber("Intake/FF", m_lastFF);
        if (newP != m_lastP || newFF != m_lastFF) {
            m_lastP = newP;
            m_lastFF = newFF;
            IntakeSpinMotor1Config.closedLoop.p(m_lastP).velocityFF(m_lastFF);
            IntakeSpinMotor1.configure(IntakeSpinMotor1Config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        }
    }
}