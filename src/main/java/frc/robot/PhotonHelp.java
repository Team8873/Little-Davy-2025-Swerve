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
import frc.robot.LimelightHelpers.RawFiducial;

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
static PhotonTrackedTarget Rtarget = resultR.get(0).getBestTarget();
static double rightyaw = 0;
static double rightpitch = 0;
static double rightarea = 0;

public static PhotonCamera cameraL = new PhotonCamera(kCameraName);
    // Read in relevant data from the Camera
boolean targetVisible = false;
static List<PhotonPipelineResult> resultL = cameraL.getAllUnreadResults();
double LtargetYaw = 0.0;
double LtargetRange = 0.0;
static PhotonTrackedTarget Ltarget = resultL.get(0).getBestTarget();
static double leftyaw = 0;
static double leftpitch = 0;
static double leftarea = 0;
public PhotonHelp(){
    updateTargets();
    updateResults();
}
public static void updateTargets(){
    if(resultL.toArray().length < 2){}
    else{
    Ltarget = resultL.get(1).getBestTarget();
    }
    if(resultR.toArray().length < 2){}else{
    Rtarget = resultR.get(1).getBestTarget();
    }
}

public static void updateResults(){
if(resultR.get(0).getBestTarget()==null){}
else{
rightyaw = Rtarget.getYaw();
rightpitch = Rtarget.getPitch();
rightarea = Rtarget.getArea();
}

if(resultL.get(0).getBestTarget()==null){}
else{
leftyaw = Ltarget.getYaw();
leftpitch = Ltarget.getPitch();
leftarea = Ltarget.getArea();
}
}

public static double TX(){
    updateTargets();

    updateResults();
    return ((rightyaw+leftyaw)/2);
}

public static double TY(){
    updateTargets();

    updateResults();

    return ((rightpitch+leftpitch)/2);
}
    
}