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
    private static final double kAngleToleranceDeg = 4.0;
    private boolean m_isActive = true;

    private final InterpolatingDoubleTreeMap m_angleTable = new InterpolatingDoubleTreeMap();

    public LauncherHoodSubsystem() {
        m_motor = new SparkMax(Constants.CanConstants.LauncherHoodMotorCanID, MotorType.kBrushless);
        m_encoder = m_motor.getAbsoluteEncoder();
        m_controller = m_motor.getClosedLoopController();

        setupInterpolationTable();

        SparkMaxConfig config = new SparkMaxConfig();
        
        // 1. Maintain Safety: Soft limits prevent mechanical damage
        config.softLimit
            .forwardSoftLimit(Constants.Launcher.kMaxHoodAngle)
            .forwardSoftLimitEnabled(true)
            .reverseSoftLimit(Constants.Launcher.kMinHoodAngle)
            .reverseSoftLimitEnabled(true);

        // 2. Maintain Physics: Must convert rotations to degrees
        config.absoluteEncoder
            .positionConversionFactor(360.0)
            .velocityConversionFactor(360.0 / 60.0)
            .inverted(true);

        config.idleMode(IdleMode.kBrake);

        config.closedLoop
            .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
            .p(0.05);

        m_motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        m_targetAngleDeg = m_encoder.getPosition();
        m_requestedAngleDeg = m_targetAngleDeg;
        m_lastTimestamp = Timer.getFPGATimestamp();
    }


    private void setupInterpolationTable() {
        m_angleTable.put(1.75, 5.0); 
        m_angleTable.put(2.1, 20.0);
        m_angleTable.put(2.7, 40.0);
        m_angleTable.put(3.44, 50.0);
        m_angleTable.put(4.1, 86.0);
    }

    public void setAngleFromPose(Translation2d robotPose, Translation2d targetPose) {
        double distance = robotPose.getDistance(targetPose);
        setAngle(m_angleTable.get(distance));
    }

    public void setAngle(double degrees) {
        m_isActive = true; 
        m_requestedAngleDeg = MathUtil.clamp(degrees, Constants.Launcher.kMinHoodAngle, Constants.Launcher.kMaxHoodAngle);
    }

   public void stop() {
        m_isActive = false;
        m_motor.stopMotor();
    }


    /**
     * Sync the internal expected/requested positions to the current encoder reading and
     * update the controller reference so the hood does not move unexpectedly when enabled.
     * Call this during robot enable (teleop/auton/test) or after any encoder reset.
     */
    public void syncToEncoder() {
        double pos = getAngle();
        m_targetAngleDeg = pos;
        m_requestedAngleDeg = pos;
        // Update controller reference to current position to prevent immediate motion
        m_controller.setReference(m_targetAngleDeg, SparkMax.ControlType.kPosition);
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

        double currentAngle = getAngle();

        if(currentAngle < Constants.Launcher.kMinHoodAngle || currentAngle > Constants.Launcher.kMaxHoodAngle){
            m_isActive = false;
            m_motor.stopMotor();
            SmartDashboard.putBoolean("Launcher/Hood out of bounds", true);
        } else {
            SmartDashboard.putBoolean("Launcher/Hood out of bounds", false);
            if (m_isActive) {
                // Calculate rate-limited target
                double maxDelta = Constants.Launcher.kMaxHoodSpeedDegPerSec * dt;
                m_targetAngleDeg = MathUtil.clamp(m_requestedAngleDeg, m_targetAngleDeg - maxDelta, m_targetAngleDeg + maxDelta);
                
                // Only send PID references when active
                m_controller.setReference(m_targetAngleDeg, SparkMax.ControlType.kPosition);
            }
        }

        // SmartDashboard logging remains identical
    

        SmartDashboard.putNumber("Launcher/Hood Angle", getAngle());
        SmartDashboard.putNumber("Launcher/Hood Target Angle", m_targetAngleDeg);
        SmartDashboard.putNumber("Launcher/Hood Requested Angle", m_requestedAngleDeg);
        SmartDashboard.putNumber("Launcher/Hood MaxSpeedDegPerSec", Constants.Launcher.kMaxHoodSpeedDegPerSec);
        SmartDashboard.putBoolean("Launcher/Hood At Target", isAtTarget());
    }
}