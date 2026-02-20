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

public class IntakeDeploySubsystem extends SubsystemBase {

    private final SparkMax IntakeDeployMotor1;
    private final SparkMax IntakeDeployMotor2;
    private final SparkClosedLoopController positionController;
    private final RelativeEncoder IntakeDeployEncoder;

    public IntakeDeploySubsystem() {
        IntakeDeployMotor1 = new SparkMax(Constants.CanConstants.IntakeDeployMotor1CanID, MotorType.kBrushless);
        IntakeDeployMotor2 = new SparkMax(Constants.CanConstants.IntakeDeployMotor2CanID, MotorType.kBrushless);
        positionController = IntakeDeployMotor1.getClosedLoopController();
        IntakeDeployEncoder = IntakeDeployMotor1.getEncoder();

        SparkMaxConfig IntakeDeployMotor1Config = new SparkMaxConfig();
        SparkMaxConfig IntakeDeployMotor2Config = new SparkMaxConfig();

        // --- LINEAR CONVERSION (INCHES) ---
        // Converts 1 motor rotation into inches of linear travel
        double conversionFactor = Constants.IntakeDeploy.kTravelPerRotation / Constants.IntakeDeploy.kGearRatio;
        
        IntakeDeployMotor1Config.encoder
            .positionConversionFactor(conversionFactor)
            .velocityConversionFactor(conversionFactor / 60.0);

        // Leader Config
        IntakeDeployMotor1Config
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(40);

        // Position PID (Tuned for Inches)
        IntakeDeployMotor1Config.closedLoop
            .p(0.5) // Linear actuators often need a higher P than swing arms
            .outputRange(-0.6, 0.6); // Cap speed for mechanical safety

        // Hardware Limit Switches
        IntakeDeployMotor1Config.limitSwitch
            .forwardLimitSwitchType(Type.kNormallyClosed)
            .reverseLimitSwitchType(Type.kNormallyClosed)
            .forwardLimitSwitchEnabled(true)
            .reverseLimitSwitchEnabled(true);

        // Soft Limits (Prevent over-traveling the screw/rack)
        IntakeDeployMotor1Config.softLimit
            .forwardSoftLimitEnabled(true)
            .forwardSoftLimit(Constants.IntakeDeploy.kMaxExtensionInches);

        // Follower Config
        IntakeDeployMotor2Config
            .idleMode(IdleMode.kBrake)
            .follow(IntakeDeployMotor1, true); 

        // Apply Configurations
        IntakeDeployMotor1.configure(IntakeDeployMotor1Config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        IntakeDeployMotor2.configure(IntakeDeployMotor2Config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        
        IntakeDeployEncoder.setPosition(0);
    }

    /** @param inches Target extension in inches */
    public void setLinearPosition(double inches) {
        positionController.setReference(inches, SparkMax.ControlType.kPosition);
    }

    public void extend() {
        setLinearPosition(Constants.IntakeDeploy.kExtendedInches);
    }

    public void retract() {
        setLinearPosition(0);
    }

    public void runAtPower(double power) {
        IntakeDeployMotor1.set(power);
    }

    public void stopPivot() {
        IntakeDeployMotor1.stopMotor();
    }

    public void resetEncoder() {
        IntakeDeployEncoder.setPosition(0);
    }

    public boolean isReverseLimitPressed() {
        return IntakeDeployMotor1.getReverseLimitSwitch().isPressed();
    }

    public double getPosition() {
        return IntakeDeployEncoder.getPosition();
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Intake/Extension Inches", IntakeDeployEncoder.getPosition());
        
        if (isReverseLimitPressed()) {
            resetEncoder();
        }
    }
}