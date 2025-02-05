package frc.robot.subsystems;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.ElevatorConstants;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;





public class Elevator extends SubsystemBase{
    private final SparkMax motorLeft = new SparkMax(ElevatorConstants.elevatorLCanId, MotorType.kBrushless);
    private final SparkMax leadMotorRight = new SparkMax(ElevatorConstants.elevatorRCanId, MotorType.kBrushless);
    private boolean configured = false;
    public Elevator(){
    }
    public void setFollower(){
        SparkMaxConfig globalConfig = new SparkMaxConfig();
        SparkMaxConfig FollowerConfig = new SparkMaxConfig();
    
        /*
         * Set parameters that will apply to all SPARKs. We will also use this as
         * the left leader config.
         */
        globalConfig
            .smartCurrentLimit(50)
            .idleMode(IdleMode.kBrake);
    
        // Apply the global config and invert since it is on the opposite side
    
        // Apply the global config and set the leader SPARK for follower mode
        FollowerConfig
            .apply(globalConfig)
            .follow(motorLeft);
             /*
     * Apply the configuration to the SPARKs.
     *
     * kResetSafeParameters is used to get the SPARK MAX to a known state. This
     * is useful in case the SPARK MAX is replaced.
     *
     * kPersistParameters is used to ensure the configuration is not lost when
     * the SPARK MAX loses power. This is useful for power cycles that may occur
     * mid-operation.
     */
        leadMotorRight.configure(globalConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        motorLeft.configure(FollowerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        configured = true;
    }
    public void checkIfSetFollow(){
        if(!configured){
            setFollower();
        }
    }

    @Override
  public void periodic(){
    checkIfSetFollow();
  } 
       
}
