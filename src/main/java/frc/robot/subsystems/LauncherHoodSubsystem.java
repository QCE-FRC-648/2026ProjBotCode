package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class LauncherHoodSubsystem extends SubsystemBase {

    private final SparkMax m_motor;
    private final SparkClosedLoopController m_controller;
    private final RelativeEncoder m_encoder;

    // The Lookup Table: Keys are Distance (meters), Values are Hood Angles (degrees)
    private final InterpolatingDoubleTreeMap m_angleTable = new InterpolatingDoubleTreeMap();

    public LauncherHoodSubsystem() {
        m_motor = new SparkMax(Constants.CanConstants.LauncherHoodMotorCanID, MotorType.kBrushless);
        m_controller = m_motor.getClosedLoopController();
        m_encoder = m_motor.getEncoder();

        setupInterpolationTable();
        
        SparkMaxConfig config = new SparkMaxConfig();
        double conversionFactor = 360.0 / Constants.Launcher.kHoodGearRatio;
        
        config.encoder
            .positionConversionFactor(conversionFactor)
            .velocityConversionFactor(conversionFactor / 60.0);

        config.idleMode(IdleMode.kBrake).smartCurrentLimit(30);

        config.softLimit
            .forwardSoftLimitEnabled(true)
            .forwardSoftLimit(Constants.Launcher.kMaxHoodAngle)
            .reverseSoftLimitEnabled(true)
            .reverseSoftLimit(0);

        config.closedLoop.p(0.05).outputRange(-0.4, 0.4);

        m_motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    private void setupInterpolationTable() {
        // Syntax: m_angleTable.put(DistanceInMeters, HoodAngleInDegrees);
        // Add your testing data here!
        m_angleTable.put(1.0, 10.0); 
        m_angleTable.put(2.0, 25.0);
        m_angleTable.put(3.0, 38.0);
        m_angleTable.put(5.0, 55.0);
    }

    /**
     * Calculates the distance to the target and sets the hood angle accordingly.
     * @param robotPose Current Translation2d of the robot from Odometry
     * @param targetPose Translation2d of the Speaker/Target
     */
    public void setAngleFromPose(Translation2d robotPose, Translation2d targetPose) {
        double distance = robotPose.getDistance(targetPose);
        double targetAngle = m_angleTable.get(distance);
        
        setAngle(targetAngle);
        
        SmartDashboard.putNumber("Launcher/Auto Distance", distance);
        SmartDashboard.putNumber("Launcher/Auto Target Angle", targetAngle);
    }

    public void setAngle(double degrees) {
        m_controller.setReference(degrees, SparkMax.ControlType.kPosition);
    }

    public void stop() {
        m_motor.stopMotor();
    }

    public double getAngle() {
        return m_encoder.getPosition();
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Launcher/Hood Angle", getAngle());
    }
}