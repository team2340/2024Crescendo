package frc.robot.subsystems;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.Joystick;


public class Drivetrain extends SubsystemBase{
    private final WPI_TalonSRX leftFront = new WPI_TalonSRX(Constants.DRIVETRAIN_LEFT_FRONT_MOTOR);
    private final WPI_TalonSRX leftRear = new WPI_TalonSRX(Constants.DRIVETRAIN_LEFT_REAR_MOTOR);
    private final WPI_TalonSRX rightFront = new WPI_TalonSRX(Constants.DRIVETRAIN_RIGHT_FRONT_MOTOR);
    private final WPI_TalonSRX rightRear = new WPI_TalonSRX(Constants.DRIVETRAIN_RIGHT_REAR_MOTOR);

    private final DifferentialDrive robotDrive;
    private final Joystick joystick;

    // Makes it so the motors don't suddenly start when someone gives it full throttle
    SlewRateLimiter forwardBackwardLimiter = new SlewRateLimiter(Constants.FORWARD_BACKWARD_SLEW_RATE);
    SlewRateLimiter rotationLimiter = new SlewRateLimiter(Constants.ROTATION_SLEW_RATE);

    public Drivetrain(Joystick joystick) {
        this.joystick = joystick;

        leftFront.setInverted(false);
        leftRear.setInverted(false);
        rightFront.setInverted(true);
        rightRear.setInverted(true);

        leftRear.follow(leftFront);
        rightRear.follow(rightFront);

        robotDrive = new DifferentialDrive(leftFront, rightFront);
        robotDrive.setSafetyEnabled(true);
    }

    public void drive(){
        double forwardValue = 0.0;
        double rotationValue = 0.0;
        double factor = 0.0; 

        factor = (joystick.getTwist()-1)/2; 
        factor= (factor*-1);
        forwardValue=joystick.getY() * factor;
        rotationValue=joystick.getX() * factor;
       

        robotDrive.arcadeDrive(
            forwardBackwardLimiter.calculate(-forwardValue), 
            rotationLimiter.calculate(-rotationValue)
        );
    }

    private void driveLeft(double speed) {
        leftFront.set(speed);
        leftRear.set(speed);
        System.out.println(speed);
    }

    private void driveRight(double speed) {
        rightFront.set(speed);
        rightRear.set(speed);
        System.out.println(speed);
    }
}
