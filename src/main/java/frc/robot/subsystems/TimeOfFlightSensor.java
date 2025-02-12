package frc.robot.subsystems;
import frc.robot.Constants.ToFConstants;

import java.util.function.BooleanSupplier;


import com.playingwithfusion.TimeOfFlight;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.*;

public class TimeOfFlightSensor extends SubsystemBase {

    private final TimeOfFlight sensor = new TimeOfFlight(ToFConstants.SensorId);
    
    private BooleanSupplier inDistance = () -> false;
    private double distanceInmm = 0;
    private double distanceInInches = 0;
    private final double mmToInches = 25.4;
    private ShuffleboardTab tab = Shuffleboard.getTab("Subsystems");
    //creates the shuffleboard widget
    private GenericEntry distanceEntry =
      tab.add("distance", 0)
         .withWidget(BuiltInWidgets.kDial)
         .withPosition(1,1)
         .getEntry();
    
    
    private GenericEntry coralEntry =
      tab.add("coralInRange", false)
         .withWidget(BuiltInWidgets.kBooleanBox)
         .withPosition(1,2)
         .getEntry();

    /**
     * runs command
     * @return Gets distance from tofsensor and converts it to inches
     */
    public Command getDistance(){
        return this.run(
            ()->{
                distanceInmm = sensor.getRange();
                convertMmToInches();
            });
    }
    /**
     * If you dont understand GET OUT
     * @return 
     */
    private void convertMmToInches(){
        distanceInInches = distanceInmm/mmToInches;
        distanceEntry.setDouble(distanceInInches);  //updates shuffleboard
    }

    /**
     * checks if object is close to sensor
     * @return booleansupplier
     */
    private BooleanSupplier checkInRange(){
        if(distanceInInches < 5){
            inDistance = () -> true;
        }else{inDistance = () -> false;}
        coralEntry.setBoolean(inDistance.getAsBoolean());   //shuffleboard updater
        return inDistance;    
    }

    //creates a trigger for a condition
    public final Trigger coralInRange = new Trigger(checkInRange());    //requires booleansupplier?
 
    @Override
  public void periodic() {
    // This method will be called once per scheduler run
    getDistance();
    checkInRange();
    
  }

}

