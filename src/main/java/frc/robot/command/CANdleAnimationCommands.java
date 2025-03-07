package frc.robot.command;

import com.ctre.phoenix.led.StrobeAnimation;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagDetection;
import edu.wpi.first.units.DistanceUnit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.command.CANdleConfigCommands;
import frc.robot.command.CANdlePrintCommands;
import frc.robot.subsystems.CANdleSystem;
import frc.robot.subsystems.CANdleSystem.AnimationTypes;

public class CANdleAnimationCommands extends Command {
    public double distanceFromAprilTag = 0;
    public boolean isItCloseEnough = false;
    public int aprilTagID = 0;
    public boolean aprilTagDetected = false;
    public CANdleSystem m_CaNdleSystem;

    public CANdleAnimationCommands(CANdleSystem CANDLE) {
        m_CaNdleSystem = CANDLE;
    }

    @Override
    // Called when the command is initially scheduled.
    public void initialize() {
        // getAprilTagID();
        aprilAnimation();
    }

    @Override
    public void execute() {
    //getAprilTag Distance
    //adjust animation speed based on distance from april tag
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        if (distanceFromAprilTag < 2) {
            isItCloseEnough = true;
        }
        return isItCloseEnough;
    }

    private void aprilAnimation() {
        switch (aprilTagID) {
            case 1:
                m_CaNdleSystem.clearAllAnims();
                m_CaNdleSystem.changeAnimation(AnimationTypes.Strobe);
                break;
            case 2:

                break;
            case 3:

                break;
            default:
                m_CaNdleSystem.clearAllAnims();
                break;

        }

    }
}