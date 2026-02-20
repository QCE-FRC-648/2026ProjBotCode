package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IndexerSubsystem extends SubsystemBase {

    private final SparkMax IndexerMotor1;
    private final SparkMax IndexerMotor2;
    private final SparkClosedLoopController velocityController;
    private final RelativeEncoder indexerEncoder;

    private double targetRPM = 0;

    public IndexerSubsystem() {
        // Initialize Motors (SparkMax for Indexer)
        IndexerMotor1 = new SparkMax(Constants.CanConstants.IndexerMotor1CanID, MotorType.kBrushless);
        IndexerMotor2 = new SparkMax(Constants.CanConstants.IndexerMotor2CanID, MotorType.kBrushless);

        velocityController = IndexerMotor1.getClosedLoopController();
        indexerEncoder = IndexerMotor1.getEncoder();
        // 2026.0.1 Config Objects
        SparkMaxConfig IndexerMotor1Config = new SparkMaxConfig();
        SparkMaxConfig IndexerMotor2Config = new SparkMaxConfig();

        // Leader Config: Typically indexers use 'Brake' mode for precision feeding
        IndexerMotor1Config
            .idleMode(IdleMode.kBrake) 
            .smartCurrentLimit(30); // Indexers don't usually need as much juice as flywheels

        IndexerMotor1Config.closedLoop
            .p(0.0001)
            .velocityFF(0.00018); // Adjust based on your gearing

        // Follower Config
        IndexerMotor2Config
            .idleMode(IdleMode.kBrake)
            .follow(IndexerMotor1, true); // true = inverted to pinch the ball

        // Apply Configurations
        IndexerMotor1.configure(IndexerMotor1Config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        IndexerMotor2.configure(IndexerMotor2Config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    /**
     * Set the indexer to a specific RPM.
     */
    public void setVelocity(double rpm) {
        this.targetRPM = rpm;
        velocityController.setReference(rpm, SparkMax.ControlType.kVelocity);
    }

    public void stop() {
        this.targetRPM = 0;
        IndexerMotor1.stopMotor();
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Indexer/Actual RPM", indexerEncoder.getVelocity());
        SmartDashboard.putNumber("Indexer/Target RPM", targetRPM);
    }
}