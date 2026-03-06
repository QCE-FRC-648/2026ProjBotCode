package frc.robot.subsystems;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkBase.PersistMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase {

    private final SparkFlex IntakeSpinMotor1;
    private final SparkFlex IntakeSpinMotor2;
    private final SparkClosedLoopController velocityController;
    private final RelativeEncoder IntakeSpinEncoder;

    private double targetRPM = 0;
    private final double VELOCITY_TOLERANCE = 100.0;

    private SparkFlexConfig IntakeSpinMotor1Config = new SparkFlexConfig();
    private SparkFlexConfig IntakeSpinMotor2Config = new SparkFlexConfig();


    public IntakeSubsystem() {
        IntakeSpinMotor1 = new SparkFlex(Constants.CanConstants.IntakeSpinMotor1CanID, MotorType.kBrushless);
        IntakeSpinMotor2 = new SparkFlex(Constants.CanConstants.IntakeSpinMotor2CanID, MotorType.kBrushless);

        velocityController = IntakeSpinMotor1.getClosedLoopController();
        IntakeSpinEncoder = IntakeSpinMotor1.getEncoder();

        // Configure motors if needed
        IntakeSpinMotor1Config
            .idleMode(IdleMode.kCoast)
            .smartCurrentLimit(40)
            .openLoopRampRate(.25)
            .closedLoopRampRate(.25)
            .inverted(true);

        IntakeSpinMotor2Config
            .idleMode(IdleMode.kCoast)
            .smartCurrentLimit(40)
            .follow(IntakeSpinMotor1, true);

        IntakeSpinMotor1Config.closedLoop
            .p(0.0001)
            .velocityFF(0.00017);

        IntakeSpinMotor1.configure(IntakeSpinMotor1Config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        IntakeSpinMotor2.configure(IntakeSpinMotor2Config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

   public void setVelocity(double rpm) {
        this.targetRPM = rpm;
        velocityController.setReference(rpm, SparkFlex.ControlType.kVelocity);
    }

    public void stop() {
        this.targetRPM = 0;
        IntakeSpinMotor1.stopMotor();
    }

    public double getActualRPM() {
        return IntakeSpinEncoder.getVelocity();
    }

    public boolean isAtTarget() {
        return targetRPM > 0 && Math.abs(getActualRPM() - targetRPM) < VELOCITY_TOLERANCE;
    }

    @Override
    public void periodic() {
        // Elastic will pick these up automatically. 
        // Tip: Use a "/" to create a sub-folder in Elastic's network tree.
        SmartDashboard.putNumber("Intake/Target RPM", targetRPM);
        SmartDashboard.putNumber("Intake/Actual RPM", getActualRPM());
        SmartDashboard.putBoolean("Intake/At Velocity", isAtTarget());
        SmartDashboard.putNumber("Intake/Output Amps", IntakeSpinMotor1.getOutputCurrent());
    }
}