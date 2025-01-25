package frc.robot.subsystems;
import frc.robot.Constants.ToFConstants;;
import com.playingwithfusion.TimeOfFlight;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
public class TimeOfFlightSensor extends SubsystemBase{
    private TimeOfFlight sensor = new TimeOfFlight(ToFConstants.SensorId);
    public TimeOfFlightSensor(){

    }

}
