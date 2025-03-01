package frc.robot;


public class Constants {
    public static class CANdleConstants {
        public static final int CANdleID = 14;
    }
    public static class ToFConstants{
        public static final int SensorId = 16;
    }
    public static class ElevatorConstants{
        public static final int elevatorLCanId = 17;
        public static final int elevatorRCanId = 19;
        public static final double kP = 0.375;
        public static final double kI = 0.015;
        public static final double kD = 0.0;
        public static final double gearRatio = .05;
        public static final double maxElevatorInput = 50;
        public static final double maxVelocity = 2.5;
        public static final double maxAcceleration = 2;
        public static final double kS = 0;
        public static final double kG = 0.246;
        public static final double kV = 60;

    }
    public static class IntakeConstants{
        public static final int intakeCanId = 15;
        public static final int humanIntakeCanId = 18;
        public static final double humankP = 0.1;
        public static final double humankI = 0.0;
        public static final double humankD = 0.0;
        public static final double velocitykP = 0.1;
        public static final double velocitykI = 0.0;
        public static final double velocitykD = 0.0;

    }
    public static class ArmConstants {
        public static final int armCanId = 13;
        public static final int wristCanId = 14;
        public static final double armkP = 4;
        public static final double armkI = 0.025;
        public static final double armkD = 0.0;
        public static final double wristkP = 0.05;
        public static final double wristkI = 0.0;
        public static final double wristkD = 0.0;
        public static final double wristGearRatio = 16;
        public static final double armGearRatio = 0;
        public static final int encoderId = 0;
        public static final double maxVelocity = 1.5;
        public static final double maxAcceleration = .5;
        public static final double kS = 0.025;
        public static final double kG = 0.06;
        public static final double kV = 0;
        
        
    }
    public static class PresetConstants {
        //default flat wrist pos
        public static final double lvl1to3ArmPos = 0.186;
        public static final double lvl4ArmPos = 3.7; 
        public static final double wristFlatPos = 0;
        public static final double lvl1Elevator = 2.55;
        public static final double lvl2Elevator = 3.55;
        public static final double lvl3Elevator = 4;
        public static final double lvl4Elevator = 4;

        //side wrist pos
        public static final double wristSidePos = 0;
        public static final double lvl1to3ArmPosSide = 0;

        //public static final double lvl1ElevatorSide = 0;
        public static final double lvl2ElevatorSide = 0;
        public static final double lvl3ElevatorSide = 0;

        //dock pos
        public static final double wristDockPos = 0;
        public static final double armDockPos = 0;
        public static final double humanIntakeDockPos = 0;

        //active intake pos
        public static final double humanIntakeActivePos = 0; 

        //ground intake 
        public static final double goundElevator = 0;
        public static final double groundArm = 0.251;

        public static final double wristSidePosNeg = 1-wristSidePos;
        public static final double wristFlatPosNeg = 0;

        public static final double travelArm = 0.451;

    }
    public static class ClimberConstants {
        //gearbox stuff
        //every deg is about 42/360 which is 0.1167
        //Ticks per deg converts from deg to ticks
        //public static final double ticksPerDegClimber = 42/360;

        //a 100:1 gearbox
        public static final double motorRatioClimberMultiplier = 100;

        //it takes approxamitely a little over 3 spool rotations to go from engaged-> resting
        //a little less than 2 to go from climbed -> engaged
        public static final double spoolRotationsRestingToEngaged = 3.2;
        public static final double spoolRotationsEngagedToClimbed = 1.9;
        //360 deg is one spool rotations       
   
        //defined positions, getPosition measures in rotations
        //360 deg is 1 spool rotation so multiply 360 by that many spool rotations
        public static final double restingPosition = 0;
        public static final double engagedPosition = 90;
        public static final double climbedPositon = 45;

        //PID constants:
        public static final double ClimberkP = 0.1;
        public static final double ClimberkI = 0;
        public static final double ClimberkD = 0;


        //Identification
        public static final int motorForClimberID = 16;
        public static final int servoID = 0;
        public static final int encoderForClimberDIOPort = 10;
    }
}
     

