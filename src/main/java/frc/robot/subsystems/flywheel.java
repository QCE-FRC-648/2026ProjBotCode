package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.RelativeEncoder;

import frc.robot.Constants.CANConfig;

public class flywheel extends SubsystemBase {
    
    
    SparkMaxConfig config = new SparkMaxConfig();
    private final SparkMax m_flywheel;
    private final SparkClosedLoopController pidController;
    private final RelativeEncoder encoder;
    // track last requested setpoint for debugging
    private double lastRequestedRpm = 0.0;

    public flywheel() {
        
    m_flywheel = new SparkMax(CANConfig.FLYWHEEL_LEFT, MotorType.kBrushless);
    pidController = m_flywheel.getClosedLoopController();
    encoder = m_flywheel.getEncoder();
        // 1. Configure PID and Feedforward directly in the config object
        config.closedLoop.pid(0.001, 0, 0.001); 

         config.idleMode(IdleMode.kCoast); // Use Coast for shooters to prevent heat/wear
        config.inverted(true);
        config.smartCurrentLimit(40);

        // 2. Apply config to leader
      
        m_flywheel.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        // 3. Make the right motor follow the left
       
    }

    
    public void setVelocity(double rpm) {
        lastRequestedRpm = rpm;
        pidController.setSetpoint(rpm, SparkMax.ControlType.kVelocity);
        SmartDashboard.putNumber("Flywheel/LastRequestedRPM", lastRequestedRpm);
    }

    public double getVelocity() {
        try {
            return encoder.getVelocity();
        } catch (Throwable t) {
            // If encoder API differs, return NaN to indicate unavailable
            return Double.NaN;
        }
    }

    @Override
    public void periodic() {
        double actual = getVelocity();
        SmartDashboard.putNumber("Flywheel/ActualRPM", actual);
        SmartDashboard.putNumber("Flywheel/LastRequestedRPM", lastRequestedRpm);
        // motor applied output (best-effort; API varies between wrappers)
        try {
            // some Spark wrappers expose a get() returning applied output
            double motorOut = m_flywheel.get();
            SmartDashboard.putNumber("Flywheel/MotorPercent", motorOut);
        } catch (Throwable ignored) {
            // ignore if method doesn't exist
        }
    }
}
