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
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeDeploySubsystem extends SubsystemBase {

    private final SparkMax IntakeDeployMotor1;
    private final SparkMax IntakeDeployMotor2;
    private final SparkClosedLoopController positionController;
    private final RelativeEncoder IntakeDeployEncoder;
    // Two magnetic limit switches wired directly to the RoboRIO DIO.
    private final DigitalInput lowerLimitSwitch;
    private final DigitalInput upperLimitSwitch;

    private boolean hasHomed = false;

    public IntakeDeploySubsystem() {
        IntakeDeployMotor1 = new SparkMax(Constants.CanConstants.IntakeDeployMotor1CanID, MotorType.kBrushless);
        IntakeDeployMotor2 = new SparkMax(Constants.CanConstants.IntakeDeployMotor2CanID, MotorType.kBrushless);
        positionController = IntakeDeployMotor1.getClosedLoopController();
        IntakeDeployEncoder = IntakeDeployMotor1.getEncoder();

     SparkMaxConfig IntakeDeployMotor1Config = new SparkMaxConfig();
        SparkMaxConfig IntakeDeployMotor2Config = new SparkMaxConfig();

        // 1. CONVERSION FACTOR
        double conversionFactor = Math.abs(Constants.IntakeDeploy.kTravelPerRotation / Constants.IntakeDeploy.kGearRatio);
        IntakeDeployMotor1Config.encoder
            .positionConversionFactor(conversionFactor)
            .velocityConversionFactor(conversionFactor / 60.0);

        // 2. MOTOR INVERSION
        // Set this to 'true' or 'false' based on which direction is "Forward" (Extension)
        // Once this is set, the encoder will automatically follow this direction.
        IntakeDeployMotor1Config
            .idleMode(IdleMode.kBrake)
            .inverted(true) 
            .smartCurrentLimit(40);

        // 3. PID & SOFT LIMITS
        IntakeDeployMotor1Config.closedLoop
            .p(0.5)
            .outputRange(-0.5, 0.5); // Bumped to 0.5 to ensure it can overcome friction
/*
        IntakeDeployMotor1Config.softLimit
            .reverseSoftLimitEnabled(true)
            .reverseSoftLimit(0.0) // Bottom (Retracted)
            .forwardSoftLimitEnabled(true)
            .forwardSoftLimit(Constants.IntakeDeploy.kMaxExtensionInches); // Top (Extended)
*/
        // 4. FOLLOWER
        IntakeDeployMotor2Config
            .idleMode(IdleMode.kBrake)
            .follow(IntakeDeployMotor1, true); // Set 'true' if Motor 2 is physically mirrored

        // Apply Configurations
        IntakeDeployMotor1.configure(IntakeDeployMotor1Config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        IntakeDeployMotor2.configure(IntakeDeployMotor2Config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        
        lowerLimitSwitch = new DigitalInput(Constants.IntakeDeploy.kLowerLimitDIO);
        upperLimitSwitch = new DigitalInput(Constants.IntakeDeploy.kUpperLimitDIO);
    
    }

    /** @param inches Target extension in inches */
    public void setLinearPosition(double inches) {
        positionController.setReference(inches, SparkMax.ControlType.kPosition);
    }

    public void extend() {
        if (isUpperSwitchActive()) {
            stopPivot(); // Already at the top, don't move.
        } else {
            // Only use PID if we aren't at the limit.
            setLinearPosition(Constants.IntakeDeploy.kExtendedInches);
        }
    }

    public void retract() {
        if (isLowerSwitchActive()) {
            stopPivot(); // Don't drive into the wall!
        } else if (!hasHomed) {
            runAtPower(-0.2); 
        } else {
            setLinearPosition(0);
        }
    }

    public void runAtPower(double power) {
        // Flip sign so manual power matches the new encoder direction.
        // Scale manual open-loop power down by 25% (i.e., run at 75% commanded).
        double cmd = power;
        IntakeDeployMotor1.set(cmd);
    }

    public void stopPivot() {
        IntakeDeployMotor1.stopMotor();
    }

    public void resetEncoder() {
        IntakeDeployEncoder.setPosition(0);
    }


    /** Returns true when the lower magnetic limit switch is triggered. */
    public boolean isLowerSwitchActive() {
        // Invert if your sensor wiring returns false when pressed. Adjust as needed.
        return !lowerLimitSwitch.get();
    }

    /** Returns true when the upper magnetic limit switch is triggered. */
    public boolean isUpperSwitchActive() {
        return !upperLimitSwitch.get();
    }

    public double getPosition() {
        // Invert encoder reading so positive inches correspond to extension
        return IntakeDeployEncoder.getPosition();
    }

    @Override
    public void periodic() {
        double posInches = getPosition();
        boolean lowerRaw = isLowerSwitchActive();
        boolean upperRaw = isUpperSwitchActive();

        // We add a velocity check so it only zeroes when the intake has actually stopped.
        if (lowerRaw && Math.abs(IntakeDeployEncoder.getVelocity()) < 0.1) {
            resetEncoder();
            hasHomed = true; 
        }
        // Two possible interpretations of the encoder mounting:
        // - If the encoder measures motor rotations, travel per motor rotation = kTravelPerRotation / kGearRatio
        // - If the encoder measures output (screw) rotations, travel per output rotation = kTravelPerRotation
        double motorTravelPerRotation = Constants.IntakeDeploy.kTravelPerRotation / Constants.IntakeDeploy.kGearRatio;
        double outputTravelPerRotation = Constants.IntakeDeploy.kTravelPerRotation;
        double interpretedMotorRotations = motorTravelPerRotation > 0 ? posInches / motorTravelPerRotation : 0.0;
        double interpretedOutputRotations = outputTravelPerRotation > 0 ? posInches / outputTravelPerRotation : 0.0;
        double expectedMotorRotationsForFull = Constants.IntakeDeploy.kExtendedInches / motorTravelPerRotation;
        double expectedOutputRotationsForFull = Constants.IntakeDeploy.kExtendedInches / outputTravelPerRotation;
        /*/
        SmartDashboard.putNumber("IntakeDeploy/Extension Inches", posInches);
        SmartDashboard.putNumber("IntakeDeploy/AssumedMotorRotations", interpretedMotorRotations);
        SmartDashboard.putNumber("IntakeDeploy/AssumedOutputRotations", interpretedOutputRotations);
        SmartDashboard.putNumber("IntakeDeploy/ExpectedMotorRotationsForFull", expectedMotorRotationsForFull);
        SmartDashboard.putNumber("IntakeDeploy/ExpectedOutputRotationsForFull", expectedOutputRotationsForFull);
        */
    // Publish compatibility booleans using inference so existing dashboards continue to work
        SmartDashboard.putBoolean("IntakeDeploy/LowerLimit", lowerRaw);
        SmartDashboard.putBoolean("IntakeDeploy/UpperLimit", upperRaw);

        SmartDashboard.putBoolean("IntakeDeploy/HasHomed", hasHomed);


    }

    
}