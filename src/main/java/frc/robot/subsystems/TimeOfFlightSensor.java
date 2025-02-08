package frc.robot.subsystems;
import frc.robot.Constants.ToFConstants;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import com.playingwithfusion.TimeOfFlight;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj.shuffleboard.*;

public class TimeOfFlightSensor extends SubsystemBase {

    private final TimeOfFlight sensor = new TimeOfFlight(ToFConstants.SensorId);
    
    private BooleanSupplier inDistance = () -> false;
    private double distanceInmm = 0;
    private double distanceInInches = 0;
    private final double mmToInches = 25.4;
    

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
    }

    /**
     * checks if object is close to sensor
     * @return booleansupplier
     */
    private BooleanSupplier checkInRange(){
        if(distanceInInches < 5){
            inDistance = () -> true;
        }else{inDistance = () -> false;}
        return inDistance;    
    }

    //creates a trigger for a condition
    public final Trigger coralInRange = new Trigger(checkInRange());    //requires booleansupplier?


    public void createUISensorTab(){
        Shuffleboard.getTab("Sensors").add("distance", distanceInInches).withWidget(BuiltInWidgets.kDial).withPosition(1, 1).getEntry();
        Shuffleboard.getTab("Sensors").add("coralInRange",inDistance.getAsBoolean()).withWidget(BuiltInWidgets.kBooleanBox).withPosition(2, 1).getEntry();
    }
 
    @Override
  public void periodic() {
    // This method will be called once per scheduler run
    getDistance();
    checkInRange();
  }

}

