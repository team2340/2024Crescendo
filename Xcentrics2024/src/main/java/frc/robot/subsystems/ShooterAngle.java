package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.utils.OperationMode;

public class ShooterAngle extends SubsystemBase {
    public enum ANGLE {
        STOWED(Constants.SHOOTER_ANGLE_STOWED),
        SPEAKER(Constants.SHOOTER_ANGLE_SPEAKER),
        SOURCE(Constants.SHOOTER_ANGLE_SOURCE),
        AMPLIFIER(Constants.SHOOTER_ANGLE_AMPLIFIER),
        UNKNOWN(0.0);

        public double angle;
        ANGLE(double angle){
            this.angle = angle;
        }
    }

    private final Drivetrain drivetrain;
    private final PIDController controller = new PIDController(80, 5, 0.05);
    private final WPI_TalonSRX pivotMotor = new WPI_TalonSRX(Constants.SHOOTER_MOTOR_PIVOT_CONTROLLER_CAN_ID);

    DutyCycleEncoder encoder = new DutyCycleEncoder(0);

    private ANGLE aprilTagAngleSet = ANGLE.STOWED;
    private ANGLE automaticAngleSet = ANGLE.STOWED;
    
    private double jogSpeed = 0;

    public ShooterAngle(Drivetrain drivetrain)
    {
        this.drivetrain = drivetrain;
        controller.setTolerance(0.01);

        pivotMotor.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen);
        pivotMotor.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen);

        SmartDashboard.putData("Shooter Angle PID Controller", controller);
    }

    public void periodic()
    {
        OperationMode operationMode = RobotContainer.getInstance().shooterAngleOperationMode.getSelected();

        double setAngle = 0;
        double currentAngle = encoder.getAbsolutePosition();
        double motorPower = 0;     
        
        if( operationMode == OperationMode.AUTOMATIC )
        {
            setAngle = automaticAngleSet.angle;
            double tmpMotorPower = -controller.calculate(currentAngle, setAngle);
            if( !controller.atSetpoint() )
                motorPower = tmpMotorPower;
        } 
        else if ( operationMode == OperationMode.AUTOMATIC_WITH_APRILTAG )
        {
            for(Integer i : Constants.amplifierAprilTagIds )
            {
                if( drivetrain.hasTarget(i) )
                {  
                    //aprilTagAngleSet = ANGLE.AMPLIFIER;
                    break;
                }
            }
            for(Integer i : Constants.speakerAprilTagIds )
            {
                if( drivetrain.hasTarget(i) )
                {  
                    aprilTagAngleSet = ANGLE.SPEAKER;
                    break;
                }
            }

            for(Integer i : Constants.sourceAprilTagIds )
            {
                if( drivetrain.hasTarget(i) )
                {  
                    aprilTagAngleSet = ANGLE.SOURCE;
                    break;
                }
            }

            setAngle = aprilTagAngleSet.angle;
            double tmpMotorPower = -controller.calculate(currentAngle, setAngle);
            if( !controller.atSetpoint() )
                motorPower = tmpMotorPower;
        }
        else 
        {
            motorPower = jogSpeed;
        }
        
        // This function should continuously keep the shooter at the correct angle
        SmartDashboard.putString("DesiredPivotAngle", automaticAngleSet.name() + " : " + setAngle);
        SmartDashboard.putNumber("PivotMotorPowerOutput", motorPower);
        SmartDashboard.putString("SensedAngle", getCurrentAngle().name() + " : " + currentAngle);

        pivotMotor.set(MathUtil.clamp(motorPower, -1.0, 1.0));
    }

    public void cycleNextPosition() {
        switch (automaticAngleSet) {
            case STOWED:
                moveToAngle(ANGLE.SPEAKER);
                break;
            case SPEAKER:
                moveToAngle(ANGLE.SOURCE);
                break;
            case SOURCE:
                moveToAngle(ANGLE.AMPLIFIER);
                break;
            case AMPLIFIER:
                moveToAngle(ANGLE.STOWED);
            default:
                break;
        }
    }

    public void cyclePrevPosition() {
        switch (automaticAngleSet) {
            case STOWED:
                moveToAngle(ANGLE.AMPLIFIER);
                break;
            case SPEAKER:
                moveToAngle(ANGLE.STOWED);
                break;
            case SOURCE:
                moveToAngle(ANGLE.SPEAKER);
                break;
            case AMPLIFIER:
                moveToAngle(ANGLE.SOURCE);
            default:
                break;
        }    
    }

    /*
     * Move the shooter to a desired angle
     */
    public void moveToAngle( ANGLE angle )
    {
        this.aprilTagAngleSet = angle;
        this.automaticAngleSet = angle;
    }

    public void setJogSpeed(double speed)
    {
        this.jogSpeed = speed;
    }

    /*
     * Get the current angle of the shooter
     */
    public ANGLE getCurrentAngle()
    {
        double currentAngle = encoder.getAbsolutePosition();

        double variance = 100000;
        ANGLE bestAngle = ANGLE.UNKNOWN;

        for( ANGLE angle : ANGLE.values() )
        {
            double testVariance = Math.abs( angle.angle - currentAngle);
            if( testVariance < variance )
            {
                variance = testVariance;
                bestAngle = angle;
            }
        }
        return bestAngle;
    }
}
