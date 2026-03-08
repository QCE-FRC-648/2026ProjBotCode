package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
//import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class LauncherHoodSubsystem extends SubsystemBase {
    private final SparkMax m_motor;
    private final SparkAbsoluteEncoder m_encoder;
    private final SparkClosedLoopController m_controller;
    private double m_targetAngleDeg = 0.0;
    // The angle requested by callers; we will ramp m_targetAngleDeg toward this at a limited rate
    private double m_requestedAngleDeg = 0.0;
    private double m_lastTimestamp = 0.0;
    private static final double kAngleToleranceDeg = 3.0;
    private boolean m_isActive = true;

    private final InterpolatingDoubleTreeMap m_angleTable = new InterpolatingDoubleTreeMap();

    public LauncherHoodSubsystem() {
        m_motor = new SparkMax(Constants.CanConstants.LauncherHoodMotorCanID, MotorType.kBrushless);
        m_encoder = m_motor.getAbsoluteEncoder();
        m_controller = m_motor.getClosedLoopController();

        setupInterpolationTable();

        SparkMaxConfig config = new SparkMaxConfig();
        
        config.softLimit
            .forwardSoftLimit(Constants.Launcher.kMaxHoodAngle)
            .forwardSoftLimitEnabled(true)
            .reverseSoftLimit(Constants.Launcher.kMinHoodAngle)
            .reverseSoftLimitEnabled(true);

        config.absoluteEncoder
            .positionConversionFactor(360.0)
            .velocityConversionFactor(360.0 / 60.0)
            .inverted(true);

        config.idleMode(IdleMode.kBrake);

        config.closedLoop
            // Using the most direct 2026 package path
            .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
            .p(0.05);

        m_motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // Initialize target/request to current encoder reading
        m_targetAngleDeg = m_encoder.getPosition();
        m_requestedAngleDeg = m_targetAngleDeg;
        m_lastTimestamp = Timer.getFPGATimestamp();
    // Immediately tell the controller that the current position is the reference so
    // it does not try to move the hood when the robot is enabled.
    m_controller.setReference(m_targetAngleDeg, SparkMax.ControlType.kPosition);
    }

    private void setupInterpolationTable() {
        m_angleTable.put(1.0, 10.0); 
        m_angleTable.put(2.0, 25.0);
        m_angleTable.put(3.0, 38.0);
        m_angleTable.put(5.0, 55.0);
    }

    public void setAngleFromPose(Translation2d robotPose, Translation2d targetPose) {
        double distance = robotPose.getDistance(targetPose);
        setAngle(m_angleTable.get(distance));
    }

    public void setAngle(double degrees) {
        m_isActive = true; // Re-enable control when a new angle is requested
        m_requestedAngleDeg = MathUtil.clamp(degrees, Constants.Launcher.kMinHoodAngle, Constants.Launcher.kMaxHoodAngle);
    }

    public void stop() {
        m_isActive = false; // Tells periodic to stop sending PID commands
        m_motor.stopMotor();
    }

    public double getAngle() {
        return m_encoder.getPosition();
    }

    public boolean isAtTarget() {
        return Math.abs(getAngle() - m_targetAngleDeg) <= kAngleToleranceDeg;
    }

    @Override
    public void periodic() {
        double now = Timer.getFPGATimestamp();
        double dt = Math.max(1e-6, now - m_lastTimestamp);
        m_lastTimestamp = now;

        // 1. Calculate the rate-limited target
        double maxDelta = Constants.Launcher.kMaxHoodSpeedDegPerSec * dt;
        m_targetAngleDeg = MathUtil.clamp(m_requestedAngleDeg, m_targetAngleDeg - maxDelta, m_targetAngleDeg + maxDelta);

        // 2. THE GUARD CLAUSE
        if (!m_isActive) {
            m_motor.stopMotor();
            // Sync target so it doesn't "snap" back when re-enabled
            m_targetAngleDeg = getAngle(); 
            m_requestedAngleDeg = getAngle();
        } else {
            // Only command the motor if we are active
            m_controller.setReference(m_targetAngleDeg, SparkMax.ControlType.kPosition);
        }

        SmartDashboard.putNumber("Launcher/Hood Angle", getAngle());
        SmartDashboard.putNumber("Launcher/Hood Target Angle", m_targetAngleDeg);
        SmartDashboard.putNumber("Launcher/Hood Requested Angle", m_requestedAngleDeg);
        SmartDashboard.putNumber("Launcher/Hood MaxSpeedDegPerSec", Constants.Launcher.kMaxHoodSpeedDegPerSec);
        SmartDashboard.putBoolean("Launcher/Hood At Target", isAtTarget());
    }
}