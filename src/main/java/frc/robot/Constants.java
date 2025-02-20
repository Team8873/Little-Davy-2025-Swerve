package frc.robot;

public class Constants {
    public static class ToFConstants{
        public static final int SensorId = 16;
    }
    public static class ElevatorConstants{
        public static final int elevatorLCanId = 15;
        public static final int elevatorRCanId = 20;
        public static final double kP = 0.1;
        public static final double kI = 0.0;
        public static final double kD = 0.0;
        public static final double gearRatio = .05;

    }
    public static class IntakeConstants{
        public static final int intakeCanId = 19;
        public static final int humanIntakeCanId = 30;
        public static final double kP = 0.1;
        public static final double kI = 0.0;
        public static final double kD = 0.0;

    }
    public static class ArmConstants {
        public static final int armCanId = 17;
        public static final int wristCanId = 18;
        public static final double armkP = 0.1;
        public static final double armkI = 0.0;
        public static final double armkD = 0.0;
        public static final double wristkP = 0.1;
        public static final double wristkI = 0.0;
        public static final double wristkD = 0.0;
        
    }
    public static class PresetConstants {
        public static final double lvl1to3ArmPos = 0;
        public static final double lvl4ArmPos = 0; 
        public static final double wristFlatPos = 0;
        public static final double lvl1Elevator = 0;
        public static final double lvl2Elevator = 0;
        public static final double lvl3Elevator = 0;
        public static final double lvl4Elevator = 0;

        public static final double wristSidePos = 0;
        public static final double lvl1to3ArmPosSide = 0;
        public static final double lvl1ElevatorSide = 0;
        public static final double lvl2ElevatorSide = 0;
        public static final double lvl3ElevatorSide = 0;

    }

}
     

