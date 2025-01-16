package frc.robot.subsystems;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
public class TestMotor {
    private SparkMax motor;
    public TestMotor(int id){
        motor = new SparkMax(id, MotorType.kBrushless);

    }
    
}
