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

public class PhotonHelpRight {
   PhotonCamera camera = new PhotonCamera(kCameraName);
    // Read in relevant data from the Camera
    boolean targetVisible = false;
    List<PhotonPipelineResult> result = camera.getAllUnreadResults();
    double targetYaw = 0.0;
    double targetRange = 0.0;
    boolean hasTargets = result.get(0).hasTargets();
    PhotonTrackedTarget target = result.get(0).getBestTarget();
    double rightyaw = target.getYaw();
double rightpitch = target.getPitch();
double rightarea = target.getArea();
    
}