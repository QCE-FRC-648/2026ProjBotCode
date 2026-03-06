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
    private final DigitalInput lowerLimitSwitch;
    private final DigitalInput upperLimitSwitch;

    public IntakeDeploySubsystem() {
        IntakeDeployMotor1 = new SparkMax(Constants.CanConstants.IntakeDeployMotor1CanID, MotorType.kBrushless);
        IntakeDeployMotor2 = new SparkMax(Constants.CanConstants.IntakeDeployMotor2CanID, MotorType.kBrushless);
        positionController = IntakeDeployMotor1.getClosedLoopController();
        IntakeDeployEncoder = IntakeDeployMotor1.getEncoder();

        SparkMaxConfig IntakeDeployMotor1Config = new SparkMaxConfig();
        SparkMaxConfig IntakeDeployMotor2Config = new SparkMaxConfig();

    // --- LINEAR CONVERSION (INCHES) ---
    // Converts 1 motor rotation into inches of linear travel.
    // Spark hardware requires a positive position conversion factor; use
    // absolute value here to avoid invalid-parameter errors at configure().
    double conversionFactor = Math.abs(Constants.IntakeDeploy.kTravelPerRotation / Constants.IntakeDeploy.kGearRatio);
        
        IntakeDeployMotor1Config.encoder
            .positionConversionFactor(conversionFactor)
            .velocityConversionFactor(conversionFactor / 60.0);

        // Leader Config
        IntakeDeployMotor1Config
            .idleMode(IdleMode.kBrake)
            .inverted(true)
            .smartCurrentLimit(40);

        // Position PID (Tuned for Inches)
        IntakeDeployMotor1Config.closedLoop
            .p(0.5) // Linear actuators often need a higher P than swing arms
            // Reduce closed-loop maximum by 25% (was +/-0.6)
            .outputRange(-0.3, 0.3); // Cap speed for mechanical safety

        // We're using two magnetic limit switches wired directly to the RoboRIO DIO.
        // Disable the motor controller's onboard limit switches to avoid conflicting behavior.
        IntakeDeployMotor1Config.limitSwitch
            .forwardLimitSwitchType(Type.kNormallyClosed)
            .reverseLimitSwitchType(Type.kNormallyClosed)
            .forwardLimitSwitchEnabled(false)
            .reverseLimitSwitchEnabled(false);

        // Soft Limits (Prevent over-traveling the screw/rack)
        IntakeDeployMotor1Config.softLimit
            .forwardSoftLimitEnabled(true)
            .forwardSoftLimit(Constants.IntakeDeploy.kMaxExtensionInches);

        // Follower Config: configure motor2 to follow motor1 (mirrored)
        IntakeDeployMotor2Config
            .idleMode(IdleMode.kBrake)
            .follow(IntakeDeployMotor1, true);

        // Apply Configurations
        IntakeDeployMotor1.configure(IntakeDeployMotor1Config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        IntakeDeployMotor2.configure(IntakeDeployMotor2Config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // NOTE: We reversed the encoder conversion factor above and will invert
    // manual power commands so that both closed-loop and open-loop control
    // move the mechanism in the expected (reversed) direction without
    // relying on deprecated motor inversion APIs.
        
        IntakeDeployEncoder.setPosition(0);

        // Initialize DIO-connected magnetic limit switches for intake deploy
        lowerLimitSwitch = new DigitalInput(Constants.IntakeDeploy.kLowerLimitDIO);
        upperLimitSwitch = new DigitalInput(Constants.IntakeDeploy.kUpperLimitDIO);
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
        // Flip sign so manual power matches the new encoder direction.
        // Scale manual open-loop power down by 25% (i.e., run at 75% commanded).
        double cmd = -power * 0.75;
        IntakeDeployMotor1.set(cmd);
    }

    public void stopPivot() {
        IntakeDeployMotor1.stopMotor();
    }

    public void resetEncoder() {
        IntakeDeployEncoder.setPosition(0);
    }

    public boolean isReverseLimitPressed() {
        return isLowerSwitchActive();
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
        return IntakeDeployEncoder.getPosition();
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Intake/Extension Inches", IntakeDeployEncoder.getPosition());
        SmartDashboard.putBoolean("Intake/LowerLimit", isLowerSwitchActive());
        SmartDashboard.putBoolean("Intake/UpperLimit", isUpperSwitchActive());
        SmartDashboard.putNumber("IntakeDeployCurrent", IntakeDeployMotor1.getOutputCurrent());
        // Reset encoder when lower switch is pressed AND position is near zero to avoid accidental resets.
        double pos = IntakeDeployEncoder.getPosition();
        if (isLowerSwitchActive() && pos < (Constants.IntakeDeploy.kMaxExtensionInches / 10.0)) {
            resetEncoder();
        }
    }
}