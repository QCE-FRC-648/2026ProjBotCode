package frc.robot.subsystems;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;;

public class IndexerSubsystem extends SubsystemBase {

    private final SparkMax indexerMotor;
    private final SparkClosedLoopController velocityController;
    private final RelativeEncoder indexerEncoder;

    private double targetRPM = 0;

    public IndexerSubsystem() {
        // Initialize the SparkMax
        indexerMotor = new SparkMax(Constants.CanConstants.IndexerMotor1CanID, MotorType.kBrushless);

        velocityController = indexerMotor.getClosedLoopController();
        indexerEncoder = indexerMotor.getEncoder();

        // 2026.0.1 Config Object for SparkMax
        SparkMaxConfig indexerConfig = new SparkMaxConfig();

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
        this.targetRPM = rpm;
        velocityController.setReference(rpm, ControlType.kVelocity);
    }

    public void stop() {
        this.targetRPM = 0;
        indexerMotor.stopMotor();
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Indexer/Actual RPM", indexerEncoder.getVelocity());
        SmartDashboard.putNumber("Indexer/Target RPM", targetRPM);
    }
}