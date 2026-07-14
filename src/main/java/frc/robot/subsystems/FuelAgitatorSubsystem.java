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
import edu.wpi.first.math.filter.SlewRateLimiter;
 
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class FuelAgitatorSubsystem extends SubsystemBase {

    private final SparkMax m_motor;
  //  private final SparkMax m_motor2;
    private final SparkClosedLoopController m_controller;
    private final RelativeEncoder m_encoder;

    public FuelAgitatorSubsystem() {
        m_motor = new SparkMax(Constants.CanConstants.FuelAgitatorMotor1CanID, MotorType.kBrushless);
   //     m_motor2 = new SparkMax(Constants.CanConstants.FuelAgitatorMotor2CanID, MotorType.kBrushless);
   
        m_controller = m_motor.getClosedLoopController();
        m_encoder = m_motor.getEncoder();

        SparkMaxConfig config = new SparkMaxConfig();
        SparkMaxConfig followerConfig = new SparkMaxConfig();

        config.idleMode(IdleMode.kCoast)
              .smartCurrentLimit(30);

        // Velocity PID tuning for a NEO
        config.closedLoop
            .p(0.0001)
            .velocityFF(0.00017); 

        followerConfig
            .idleMode(IdleMode.kCoast)
            .smartCurrentLimit(30)
            .follow(m_motor, false);

        m_motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    //    m_motor2.configure(followerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    /** @param rpm Target velocity in RPM */
    public void setVelocity(double rpm) {
        double max = Constants.Agitator.kMaxSafeRpm;
        double clamped = Math.signum(rpm) * Math.min(Math.abs(rpm), max);
        // store desired target and ramp in periodic
        this.desiredTargetRPM = clamped;
    }

    public void stop() {
        this.desiredTargetRPM = 0; // Essential: Stops the ramp logic in periodic
        this.appliedTargetRPM = 0;
        rpmSlew.reset(0);           // Clear the limiter's memory
        m_motor.stopMotor();       // Immediate hardware cut
    
    }

    // Ramp helpers
    private final SlewRateLimiter rpmSlew = new SlewRateLimiter(3000.0);
    private double desiredTargetRPM = 0.0;
    private double appliedTargetRPM = 0.0;

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
        // Ramp and apply desired RPM to avoid sudden current spikes
        double next = rpmSlew.calculate(desiredTargetRPM);

        // GUARD CLAUSE:
        // If the target is 0, don't let the PID re-enable the motor.
        if (desiredTargetRPM == 0) {
            m_motor.stopMotor();
        } else {
            // Only update the controller if the value has changed significantly
            if (Math.abs(next - appliedTargetRPM) > 0.5) {
                m_controller.setReference(next, SparkMax.ControlType.kVelocity);
                appliedTargetRPM = next;
            }
        }

        SmartDashboard.putNumber("Agitator/Requested RPM", desiredTargetRPM);
        SmartDashboard.putNumber("Agitator/Applied RPM", appliedTargetRPM);
        SmartDashboard.putNumber("Agitator/RPM", m_encoder.getVelocity());
    }
}