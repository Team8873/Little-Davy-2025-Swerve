package frc.robot;

import static frc.robot.Constants.Vision.*;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class PhotonHelp {
static PhotonCamera cameraR = new PhotonCamera(kCameraName);
    // Read in relevant data from the Camera
boolean RtargetVisible = false;
static List<PhotonPipelineResult> resultR = cameraR.getAllUnreadResults();
double RtargetYaw = 0.0;
double RtargetRange = 0.0;
static boolean RhasTargets = resultR.get(0).hasTargets();
static PhotonTrackedTarget Rtarget = resultR.get(0).getBestTarget();
static double rightyaw = Rtarget.getYaw();
static double rightpitch = Rtarget.getPitch();
static double rightarea = Rtarget.getArea();
public static double RightTX(){
    return rightyaw;
}
public static double RightTY(){
    return rightpitch;
}

public static PhotonCamera cameraL = new PhotonCamera(kCameraName);
    // Read in relevant data from the Camera
boolean targetVisible = false;
static List<PhotonPipelineResult> resultL = cameraL.getAllUnreadResults();
double LtargetYaw = 0.0;
double LtargetRange = 0.0;
static boolean LhasTargets = resultL.get(0).hasTargets();
static PhotonTrackedTarget Ltarget = resultL.get(0).getBestTarget();
static double leftyaw = Ltarget.getYaw();
static double leftpitch = Ltarget.getPitch();
static double leftarea = Ltarget.getArea();
public static double LeftTX(){
    return rightyaw;
}
public static double LeftTY(){
    return leftpitch;
}
    
}