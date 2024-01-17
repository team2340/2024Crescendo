package frc.robot.subsystems;
import edu.wpi.first.wpilibj.CAN;
import edu.wpi.first.wpilibj.Joystick;

import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;


public class EmilyTest extends SubsystemBase {
    private final Joystick joystick;
    private CANSparkMax emilyTest = new CANSparkMax(12,MotorType.kBrushless);
   
    public EmilyTest(Joystick joystick){
      this.joystick = joystick;
    }

    public void startmotor(){
        
        double factor = 0.0; 

        factor = (joystick.getTwist()-1)/2; 
        factor= (factor*-1);
    
        emilyTest.set(factor);

    }

    public void stopmotor (){
        emilyTest.set(0);
    }


}
