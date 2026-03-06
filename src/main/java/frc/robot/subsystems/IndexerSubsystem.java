package frc.robot.subsystems;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;;

public class IndexerSubsystem extends SubsystemBase {

    private final SparkFlex indexerMotor;
    private final SparkClosedLoopController velocityController;
    private final RelativeEncoder indexerEncoder;

    private double targetRPM = 0;
    // Ramp helper
    private final SlewRateLimiter rpmSlew = new SlewRateLimiter(1000.0);
    private double desiredTargetRPM = 0.0;
    private double appliedTargetRPM = 0.0;

    public IndexerSubsystem() {
        // Initialize the Spark Flex / Vortex
        indexerMotor = new SparkFlex(Constants.CanConstants.IndexerMotor1CanID, MotorType.kBrushless);

        velocityController = indexerMotor.getClosedLoopController();
        indexerEncoder = indexerMotor.getEncoder();

        // 2026.0.1 Config Object for Spark Flex
        SparkFlexConfig indexerConfig = new SparkFlexConfig();

        // Configure Motor Settings
        indexerConfig
            .idleMode(IdleMode.kBrake) 
            .smartCurrentLimit(40); // Vortex can handle more than Max, but 40 is safe for indexers

        // PID & Feed Forward
        indexerConfig.closedLoop
            .p(0.0001)
            .velocityFF(0.00018); // Note: You may need to retune this for the Vortex's power profile

        // Apply Configuration
        indexerMotor.configure(indexerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    /**
     * Set the indexer to a specific RPM.
     */
    public void setVelocity(double rpm) {
        double max = Constants.Indexer.kMaxSafeRpm;
        double clamped = Math.signum(rpm) * Math.min(Math.abs(rpm), max);
        this.targetRPM = clamped;
        this.desiredTargetRPM = clamped;
    }
    /**
     * Set the indexer to a specific RPM.
     */

    public void stop() {
        this.targetRPM = 0;
        this.desiredTargetRPM = 0; // Tell periodic() to stop the PID loop
        this.appliedTargetRPM = 0;
        rpmSlew.reset(0);           // Clear the ramp "memory"
        indexerMotor.stopMotor();  // Immediate hardware stop
    }

    @Override
    public void periodic() {
        // Calculate what the ramped speed should be
        double next = rpmSlew.calculate(desiredTargetRPM);
        
        // GUARD CLAUSE: 
        // If we want 0 RPM, force a stop. Otherwise, update the PID controller.
        if (desiredTargetRPM == 0) {
            indexerMotor.stopMotor();
        } else {
            // Only update the motor controller if the ramped value has changed significantly
            if (Math.abs(next - appliedTargetRPM) > 0.5) {
                velocityController.setReference(next, ControlType.kVelocity);
                appliedTargetRPM = next;
            }
        }

        SmartDashboard.putNumber("Indexer/Actual RPM", indexerEncoder.getVelocity());
        SmartDashboard.putNumber("Indexer/Target RPM", targetRPM);
        SmartDashboard.putNumber("Indexer/Applied RPM", appliedTargetRPM);
    }
}