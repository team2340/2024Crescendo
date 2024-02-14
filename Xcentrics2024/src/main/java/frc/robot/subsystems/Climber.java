package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Climber extends SubsystemBase{
    private final WPI_TalonSRX climberMotor = new WPI_TalonSRX(Constants.CLIMBER_MOTOR_LEFT_CAN_ID);
    private final WPI_TalonSRX climberRightMotor = new WPI_TalonSRX(Constants.CLIMBER_MOTOR_RIGHT_CAN_ID);


    public Climber()
    {
        climberRightMotor.follow(climberMotor);
        
        climberMotor.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen);
        climberMotor.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen);
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
