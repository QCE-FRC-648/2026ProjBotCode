package frc.robot.subsystems;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;

import frc.robot.Constants.CANConfig;

public class intake extends SubsystemBase {
    SparkMaxConfig config = new SparkMaxConfig();
    private final SparkMax m_intake;
    private final SparkClosedLoopController pidController;
    private final RelativeEncoder encoder;
    // track last requested setpoint for debugging
    private double lastRequestedRpm = 0.0;
     public intake() {
        
    m_intake = new SparkMax(CANConfig.INTAKE, MotorType.kBrushless);
    pidController = m_intake.getClosedLoopController();
    encoder = m_intake.getEncoder();
        // 1. Configure PID and Feedforward directly in the config object
        config.closedLoop.pid(0.001, 0, 0.001); 

         config.idleMode(IdleMode.kCoast); // Use Coast for shooters to prevent heat/wear
        config.inverted(true);
        config.smartCurrentLimit(40);

        // 2. Apply config to leader
      
        m_intake.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        // 3. Make the right motor follow the left
       
    }
     public void setVelocity(double PickupRPM) {
        lastRequestedRpm = PickupRPM;
        pidController.setSetpoint(2000, null);
     }
      
}
