package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.LimitSwitchConfig.Type;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ClimberSubsystem extends SubsystemBase {

    private final SparkMax ClimberMotor1;
    //private final SparkMax ClimberMotor2;
    private final SparkClosedLoopController controller;
    private final RelativeEncoder encoder;

    private double targetPosition = 0;

    public ClimberSubsystem() {
        ClimberMotor1 = new SparkMax(Constants.CanConstants.ClimbGoUpMotor1CanID, MotorType.kBrushless);
        //ClimberMotor2 = new SparkMax(Constants.CanConstants.ClimbGoUpMotor2CanID, MotorType.kBrushless);

        controller = ClimberMotor1.getClosedLoopController();
        encoder = ClimberMotor1.getEncoder();

        SparkMaxConfig ClimberMotor1Config = new SparkMaxConfig();
        //SparkMaxConfig ClimberMotor2Config = new SparkMaxConfig();

        // --- ENCODER CONVERSION (INCHES) ---
        // This makes 1.0 in code equal 1 inch on the robot
        double conversionFactor = (Math.PI * Constants.Climber.kDrumDiameterInches) / Constants.Climber.kGearRatio;
        
        ClimberMotor1Config.encoder
            .positionConversionFactor(conversionFactor)
            .velocityConversionFactor(conversionFactor / 60.0); // Converts RPM to Inches per Second

        // Leader: Brake mode is critical to hold weight
        ClimberMotor1Config
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(50); 

        // Position PID setup
        ClimberMotor1Config.closedLoop
            .p(0.1) 
            .outputRange(-1.0, 1.0);

        // Hardware Limit Switches (Normally Closed is safer!)
        ClimberMotor1Config.limitSwitch
            .forwardLimitSwitchType(Type.kNormallyClosed)
            .reverseLimitSwitchType(Type.kNormallyClosed)
            .forwardLimitSwitchEnabled(true)  
            .reverseLimitSwitchEnabled(true); 

        // Soft limits (Now using INCHES instead of rotations)
        ClimberMotor1Config.softLimit
            .forwardSoftLimitEnabled(true)
            .forwardSoftLimit(Constants.Climber.kMaxHeightInches);

        // Follower
        //ClimberMotor2Config
        //    .idleMode(IdleMode.kBrake)
        //    .follow(ClimberMotor1, false); 

        // Apply Configurations
        ClimberMotor1.configure(ClimberMotor1Config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        //ClimberMotor2.configure(ClimberMotor2Config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        encoder.setPosition(0);
    }

    /** @param inches Target height in inches */
    public void setHeight(double inches) {
        targetPosition = inches;
        controller.setReference(inches, SparkMax.ControlType.kPosition);
    }

    public void stop() {
        ClimberMotor1.stopMotor();
    }

    public void runManual(double speed) {
        ClimberMotor1.set(speed);
    }

    public boolean isAtBottom() {
        return ClimberMotor1.getReverseLimitSwitch().isPressed();
    }

    public void resetEncoder() {
        encoder.setPosition(0);
    }

    public double getPosition() {
       return encoder.getPosition();
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Climber/Position Inches", encoder.getPosition());
        SmartDashboard.putBoolean("Climber/At Bottom", isAtBottom());
        
        if (isAtBottom()) {
            resetEncoder();
        }
    }
}