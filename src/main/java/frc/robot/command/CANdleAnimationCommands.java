package frc.robot.command;
import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagDetection;
import edu.wpi.first.units.DistanceUnit;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CANdleSystem;


public class CANdleAnimationCommands extends Command {
    public double distanceFromAprilTag = 0;
    public boolean isItCloseEnough = false;
    public int m_aprilTagID = 0;
    public boolean aprilTagDetected = false;
    public CANdleSystem m_CaNdleSystem;
    public int LEDS_PER_ANIMATION = 300;
    // What to put into robot container
    // new CANdleAnimationCOmmands(caNdlesystem, limelightHelpers.getAprilTagID);
    public CANdleAnimationCommands(CANdleSystem CANDLE, int aprilTagID) {
        m_CaNdleSystem = CANDLE;
        m_aprilTagID = aprilTagID;
        addRequirements(m_CaNdleSystem);
    }

    @Override
    // Called when the command is initially scheduled.
    public void initialize() {
        aprilAnimation();
    }

    @Override
    public void execute() {
        m_CaNdleSystem.setSpeedOfStrobeAnimations(distanceFromAprilTag);  //adjust animation speed based on distance from april tag. when closer to april tag flash slower
    //getAprilTag Distance
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end and then calls on end to end the command
    @Override
    public boolean isFinished() {
        if (distanceFromAprilTag < 2) {
            isItCloseEnough = true;
        }
        return isItCloseEnough;
    }
//1 is pink 2 is green 3 is white
    private void aprilAnimation() {
        m_CaNdleSystem.clearAllAnims();
        m_CaNdleSystem.setColors();
        switch (m_aprilTagID) {
            case 1,2,12,13:
            
            m_CaNdleSystem.runStrobeAnimations(2);

                break;
            case 3,4,5,14,15,16:
            m_CaNdleSystem.runStrobeAnimations(3);

                break;
            case 6,7,8,9,10,11,17,18,19,20,21,22:
            m_CaNdleSystem.runStrobeAnimations(1);

                break;
            default:
                m_CaNdleSystem.clearAllAnims();
                break;

        }

    }
    

    
}