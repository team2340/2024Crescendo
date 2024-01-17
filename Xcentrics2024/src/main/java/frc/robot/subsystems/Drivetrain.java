package frc.robot.subsystems;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonTrackedTarget;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Joystick;


public class Drivetrain extends SubsystemBase{
    private final WPI_TalonSRX leftFront = new WPI_TalonSRX(Constants.DRIVETRAIN_LEFT_FRONT_MOTOR);
    private final WPI_TalonSRX leftRear = new WPI_TalonSRX(Constants.DRIVETRAIN_LEFT_REAR_MOTOR);
    private final WPI_TalonSRX rightFront = new WPI_TalonSRX(Constants.DRIVETRAIN_RIGHT_FRONT_MOTOR);
    private final WPI_TalonSRX rightRear = new WPI_TalonSRX(Constants.DRIVETRAIN_RIGHT_REAR_MOTOR);
    private final PhotonCamera camera = new PhotonCamera("Microsoft_LifeCam_HD-3000");
    private final PIDController controller = new PIDController(0.1,0,0.0);
    private final PIDController turnController = new PIDController(0.1,0,0.0);
    

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

         var result = camera.getLatestResult();

         PhotonTrackedTarget target = result.getBestTarget();
        if(target != null && joystick.getRawButton(2)){
        
            double range =
            PhotonUtils.calculateDistanceToTargetMeters(
                0.2,
                1,0,
                Units.degreesToRadians(result.getBestTarget().getPitch()));
            System.out.println(range);
           // forwardValue= controller.calculate(range,-10.169);
            rotationValue = -turnController.calculate(result.getBestTarget().getYaw(), 0);
        }

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
