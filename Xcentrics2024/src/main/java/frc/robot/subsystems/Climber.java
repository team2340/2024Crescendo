package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Climber extends SubsystemBase{
    private final CANSparkMax climberMotor = new CANSparkMax(Constants.CLIMBER_MOTOR_LEFT_CAN_ID, MotorType.kBrushless);

    private final Joystick joystick;

    public Climber(Joystick joystick)
    {
        this.joystick = joystick;
        climberMotor.getEncoder().setPosition(0);
        SmartDashboard.putBoolean("Reset Climber Encoder", false);
    }

    public void doClimber()
    {
        if( SmartDashboard.getBoolean("Reset Climber Encoder", false) ) 
        {
            climberMotor.getEncoder().setPosition(0);
            SmartDashboard.putBoolean("Reset Climber Encoder", false);
        }

        double encoderPosition = climberMotor.getEncoder().getPosition();
        SmartDashboard.putNumber("Climber Encoder", encoderPosition);
        if( ( joystick.getY() > 0.1 || joystick.getY() < -0.1 ) )
        {

            if( ( encoderPosition > Constants.CLIMBER_MOTOR_MAX_ENCODER && joystick.getY() > 0.1 ) ||( encoderPosition < -Constants.CLIMBER_MOTOR_MAX_ENCODER && joystick.getY() < 0.1 ))
            {
                stopClimber();
            }
            else
            {
                climberMotor.set(joystick.getY());
            }
            
        }
        else
        {
            stopClimber();
        }
    }
    public void goUp()
    {
        climberMotor.set(0.5);
    }

    public void goDown()
    {
        climberMotor.set(-0.5);
    }

    public void stopClimber()
    {
        climberMotor.set(0);
    }
}
