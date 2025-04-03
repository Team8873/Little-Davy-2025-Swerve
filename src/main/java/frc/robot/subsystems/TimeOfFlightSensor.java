package frc.robot.subsystems;

import frc.robot.Constants.ToFConstants;

import java.util.function.BooleanSupplier;
import com.playingwithfusion.TimeOfFlight;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj.shuffleboard.*;
import edu.wpi.first.epilogue.Logged;

@Logged
public class TimeOfFlightSensor extends SubsystemBase {

  private final TimeOfFlight sensor = new TimeOfFlight(ToFConstants.SensorId);

  private BooleanSupplier inDistance = () -> false;
  private double distanceInmm = 0;
  private double distanceInInches = 0;
  private final double mmToInches = 25.4;
  private ShuffleboardTab tab = Shuffleboard.getTab("Subsystems");

  public TimeOfFlightSensor() {
    tab.addDouble("Distance From Coral", () -> distanceInInches).withPosition(0, 3).withWidget(BuiltInWidgets.kDial);
    tab.addBoolean("Coral In Range", () -> inDistance.getAsBoolean()).withPosition(3, 3).withWidget(BuiltInWidgets.kBooleanBox);
  }

  /**
   * runs command
   * 
   * @return Gets distance from tofsensor and converts it to inches
   */
  public void getDistance() {
    distanceInmm = sensor.getRange();
    convertMmToInches();
  }

  /**
   * If you dont understand GET OUT
   */
  private void convertMmToInches() {
    distanceInInches = distanceInmm / mmToInches;
  }

  /**
   * checks if object is close to sensor
   * 
   * @return booleansupplier
   */
  public BooleanSupplier checkInRange() {
    if (distanceInInches < 8) {
      inDistance = () -> true;
    } else {
      inDistance = () -> false;
    }
    return inDistance;
  }

  // creates a trigger for a condition
  public final Trigger coralInRange = new Trigger(checkInRange()); // requires booleansupplier?

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    getDistance();
    checkInRange();

  }

}
