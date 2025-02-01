package frc.robot.subsystems;
import frc.robot.Constants.ToFConstants;

import java.util.function.BooleanSupplier;

import com.playingwithfusion.TimeOfFlight;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
public class TimeOfFlightSensor extends SubsystemBase {
    private boolean inDistance = false;
    private double distanceInmm = 0;
    private TimeOfFlight sensor = new TimeOfFlight(ToFConstants.SensorId);

    public Command getDistance(){
        return this.run(
            ()->{
                distanceInmm = sensor.getRange();
            });
    }
    public Command runTestMotor(){
        return this.run(
            ()->{
                getDistance();
                checkInRange();
            });
    }
    public Boolean checkInRange(){
        if(distanceInmm < 20){
            inDistance = true;
        }else{inDistance = false;}
        
        return inDistance;
}
}

