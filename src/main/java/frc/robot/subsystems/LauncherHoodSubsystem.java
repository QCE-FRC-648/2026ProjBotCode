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
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class LauncherHoodSubsystem extends SubsystemBase {
    private final SparkMax m_motor;
    private final SparkAbsoluteEncoder m_encoder;
    private final SparkClosedLoopController m_controller;
    private double m_targetAngleDeg = 0.0;
    private static final double kAngleToleranceDeg = 1.0;

    private final InterpolatingDoubleTreeMap m_angleTable = new InterpolatingDoubleTreeMap();

    public LauncherHoodSubsystem() {
        m_motor = new SparkMax(Constants.CanConstants.LauncherHoodMotorCanID, MotorType.kBrushless);
        m_encoder = m_motor.getAbsoluteEncoder();
        m_controller = m_motor.getClosedLoopController();

        setupInterpolationTable();

        SparkMaxConfig config = new SparkMaxConfig();
        
        config.absoluteEncoder
            .positionConversionFactor(360.0)
            .velocityConversionFactor(360.0 / 60.0);

        config.idleMode(IdleMode.kBrake);

        config.closedLoop
            // Using the most direct 2026 package path
            .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
            .p(0.05);

        m_motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
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
        m_targetAngleDeg = MathUtil.clamp(degrees, 0.0, Constants.Launcher.kMaxHoodAngle);
        // Use SparkMax.ControlType for the reference
        m_controller.setReference(m_targetAngleDeg, SparkMax.ControlType.kPosition);
    }

    public void stop() {
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
        SmartDashboard.putNumber("Launcher/Hood Angle", getAngle());
        SmartDashboard.putNumber("Launcher/Hood Target Angle", m_targetAngleDeg);
        SmartDashboard.putBoolean("Launcher/Hood At Target", isAtTarget());
    }
}