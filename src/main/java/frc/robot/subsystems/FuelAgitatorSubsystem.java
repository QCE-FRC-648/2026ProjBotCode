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

public class FuelAgitatorSubsystem extends SubsystemBase {

    private final SparkMax m_motor;
    private final SparkClosedLoopController m_controller;
    private final RelativeEncoder m_encoder;

    public FuelAgitatorSubsystem() {
        m_motor = new SparkMax(Constants.CanConstants.FuelAgitatorMotorCanID, MotorType.kBrushless);
        m_controller = m_motor.getClosedLoopController();
        m_encoder = m_motor.getEncoder();

        SparkMaxConfig config = new SparkMaxConfig();

        config.idleMode(IdleMode.kCoast)
              .smartCurrentLimit(30);

        // Velocity PID tuning for a NEO
        config.closedLoop
            .p(0.0001)
            .velocityFF(0.00017); 

        m_motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    /** @param rpm Target velocity in RPM */
    public void setVelocity(double rpm) {
        m_controller.setReference(rpm, SparkMax.ControlType.kVelocity);
    }

    public void stop() {
        m_motor.stopMotor();
    }

    // Inside FuelAgitatorSubsystem.java

    /** Checks if the motor is struggling to spin despite having a target set */
    public boolean isStalled() {
        // If we are trying to spin faster than 1000 RPM but moving slower than 100 RPM
        // and drawing significant current, we are likely jammed.
        return Math.abs(m_encoder.getVelocity()) < 100 && 
            m_motor.getOutputCurrent() > 25; // 25 Amps is a typical stall threshold for a NEO
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Agitator/RPM", m_encoder.getVelocity());
    }
}