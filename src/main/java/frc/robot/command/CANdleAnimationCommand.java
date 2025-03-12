package frc.robot.command;

import com.ctre.phoenix.led.CANdle;

//activate when locked on to april tag
import edu.wpi.first.units.DistanceUnit;
import edu.wpi.first.wpilibj.AnalogTriggerOutput.AnalogTriggerType;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CANdleSystem;
import frc.robot.subsystems.CANdleSystem.AnimationTypes;
import frc.robot.subsystems.CANdleSystem.Color;

public class CANdleAnimationCommand extends Command {
    private CANdleSystem m_CandleSystem;
    private String locationLED;
    private int id;
    private AnimationTypes m_animation;
    private Color m_ledColor;

    // What to put into robot container
    // new CANdleAnimationCOmmands(caNdlesystem, limelightHelpers.getAprilTagID);
    public CANdleAnimationCommand(CANdleSystem CANDLE, String whereLED, AnimationTypes animation, Color ledColor) {

        m_animation = animation;
        m_CandleSystem = CANDLE;
        locationLED = whereLED;
        addRequirements(m_CandleSystem);
    }

    @Override
    // Called when the command is initially scheduled.
    public void initialize() {
        animationID();
        m_CandleSystem.numControl(id);
        m_CandleSystem.changeAnimation(m_animation, m_ledColor);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end and then calls on end to end the
    // command
    @Override
    public boolean isFinished() {
        return true;
    }

    /**
     * gives id to led locations
     */
    private void animationID(){
        if(locationLED.equals("crossbar")){
            id = 0;
        }else if(locationLED.equals("elevatorL")){
            id = 1;
        }else if(locationLED.equals("elevatorR")){
            id = 2;
        }

    }
}
