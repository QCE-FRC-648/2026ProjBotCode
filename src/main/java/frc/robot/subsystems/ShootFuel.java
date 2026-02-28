package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANConfig;

public class ShootFuel extends SubsystemBase {
    SparkMaxConfig config = new SparkMaxConfig();
    private final SparkFlex m_shoot_left;
    private final SparkFlex m_shoot_right;
    private final SparkClosedLoopController pidController;
    private final RelativeEncoder encoder;
    // track last requested setpoint for debugging
    private double lastRequestedRpm = 0.0;

    public ShootFuel() {
        
    m_shoot_left = new SparkFlex(CANConfig.SHOOT_LEFT, MotorType.kBrushless);
    // Initialize the follower motor so the final field is assigned
    m_shoot_right = new SparkFlex(CANConfig.SHOOT_RIGHT, MotorType.kBrushless);
    pidController = m_shoot_left.getClosedLoopController();
    encoder = m_shoot_left.getEncoder();
        // 1. Configure PID and Feedforward directly in the config object
        config.closedLoop.pid(0.00007, 0.0000000001, 0); 

         config.idleMode(IdleMode.kCoast); // Use Coast for shooters to prevent heat/wear
        config.inverted(true);
        config.smartCurrentLimit(40);

        // 2. Apply config to leader
      
        m_shoot_left.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        // 3. Make the right motor follow the left
        SparkFlexConfig rightConfig = new SparkFlexConfig();
        rightConfig.follow(m_shoot_left, true); // Set true if it needs to spin opposite direction
       rightConfig.idleMode(IdleMode.kCoast);
       m_shoot_right.configure(rightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    
    public void setVelocity(double FireRPM) {
        pidController.setSetpoint(FireRPM, SparkFlex.ControlType.kVelocity);
        SmartDashboard.putNumber("Flywheel/LastRequestedRPM", lastRequestedRpm);
    }
}
