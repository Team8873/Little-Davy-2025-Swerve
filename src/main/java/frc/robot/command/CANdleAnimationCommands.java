package frc.robot.command;
//activate when locked on to april tag
import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagDetection;
import edu.wpi.first.units.DistanceUnit;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CANdleSystem;
import frc.robot.subsystems.LimeLightFace;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.LimelightTarget_Barcode;


public class CANdleAnimationCommands extends Command {
    public LimeLightFace m_LimeLightFace;
    public double distanceFromAprilTag = 1/LimelightHelpers.getTA("limelight");
    public boolean isItCloseEnough = false;
    public int m_aprilTagID = (int) LimelightHelpers.getFiducialID("limelight");
    public CANdleSystem m_CaNdleSystem;
    // What to put into robot container
    // new CANdleAnimationCOmmands(caNdlesystem, limelightHelpers.getAprilTagID);
    public CANdleAnimationCommands(CANdleSystem CANDLE, int aprilTagID, LimeLightFace LIMELIGHTFACE) {
        m_CaNdleSystem = CANDLE;
        m_aprilTagID = aprilTagID;
        addRequirements(m_CaNdleSystem);
        m_LimeLightFace = LIMELIGHTFACE;
    }

    @Override
    // Called when the command is initially scheduled.
    public void initialize() {
        aprilAnimation();
    }

    @Override
    public void execute() {
        m_CaNdleSystem.setSpeedOfStrobeAnimations(distanceFromAprilTag);  //adjust animation speed based on distance from april tag. when closer to april tag flash slower

    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end and then calls on end to end the command
    //if its close enough it wil return true and end the command
    @Override
    public boolean isFinished() {
        if (distanceFromAprilTag < 1.5) {
            isItCloseEnough = true;
        }
        return isItCloseEnough;
    }
//1 is pink 2 is green 3 is white
    private void aprilAnimation() {
        m_CaNdleSystem.clearAllAnims();
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
            m_CaNdleSystem.runStrobeAnimations(0);
                break;

        }
    }
}