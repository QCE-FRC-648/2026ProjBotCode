package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class LauncherHoodSubsystem extends SubsystemBase {

    private final SparkMax m_motor;
    private final SparkClosedLoopController m_controller;
    private final RelativeEncoder m_encoder;

    public LauncherHoodSubsystem() {
        m_motor = new SparkMax(Constants.CanConstants.LauncherHoodMotorCanID, MotorType.kBrushless);
        m_controller = m_motor.getClosedLoopController();
        m_encoder = m_motor.getEncoder();

        SparkMaxConfig config = new SparkMaxConfig();

        // 1. Encoder Conversion (Degrees)
        double conversionFactor = 360.0 / Constants.Launcher.kHoodGearRatio;
        config.encoder
            .positionConversionFactor(conversionFactor)
            .velocityConversionFactor(conversionFactor / 60.0);

        // 2. Motor Limits & Safety
        config.idleMode(IdleMode.kBrake) // Must be brake to hold angle against vibrations
              .smartCurrentLimit(30);

        // 3. Soft Limits (Crucial for a hood!)
        config.softLimit
            .forwardSoftLimitEnabled(true)
            .forwardSoftLimit(Constants.Launcher.kMaxHoodAngle)
            .reverseSoftLimitEnabled(true)
            .reverseSoftLimit(0); // 0 is usually the "stowed" or "bottom" position

        // 4. Position PID
        config.closedLoop
            .p(0.05) 
            .outputRange(-0.4, 0.4); // Limit speed to prevent mechanical damage

        m_motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    /** @param degrees Target angle for the hood */
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