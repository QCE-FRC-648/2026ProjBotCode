package frc.robot.subsystems.ClimberSubsystem;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import frc.robot.Constants.CanConstants;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.ClimberConstants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;

    public class ClimberSubsystem extends SubsystemBase{
        
        private final SparkMax ClimbGoUpMotor = new SparkMax(CanConstants.ClimbGoUpMotorCanID, MotorType.kBrushless);
        private final SparkMax ClimbDeployMotor = new SparkMax(CanConstants.ClimbDeployMotorCanID, MotorType.kBrushless);
        
        private SparkMaxConfig ClimbGoUpMotorConfig = new SparkMaxConfig();
        private SparkMaxConfig ClimbDeployMotorConfig = new SparkMaxConfig();

        
        private final DigitalInput limitSwitchRight = new DigitalInput(0);
        private final DigitalInput limitSwitchLeft = new DigitalInput(1);



        public boolean getLimitSwitchRight()
        {
            return limitSwitchRight.get();
        }

        public boolean getLimitSwitchLeft()
        {
            return limitSwitchLeft.get();
        }
    }









