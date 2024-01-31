package frc.robot.subsystems;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonTrackedTarget;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Joystick;


public class Drivetrain extends SubsystemBase{
    private final WPI_TalonSRX leftFront = new WPI_TalonSRX(Constants.DRIVETRAIN_LEFT_FRONT_MOTOR);
    private final WPI_TalonSRX leftRear = new WPI_TalonSRX(Constants.DRIVETRAIN_LEFT_REAR_MOTOR);
    private final WPI_TalonSRX rightFront = new WPI_TalonSRX(Constants.DRIVETRAIN_RIGHT_FRONT_MOTOR);
    private final WPI_TalonSRX rightRear = new WPI_TalonSRX(Constants.DRIVETRAIN_RIGHT_REAR_MOTOR);
    private final PhotonCamera camera = new PhotonCamera("Arducam_OV9281_USB_Camera");
    
    private final PIDController controller = new PIDController(
        Constants.FORWARD_REVERSE_CONTROLLER_P,
        Constants.FORWARD_REVERSE_CONTROLLER_I,
        Constants.FORWARD_REVERSE_CONTROLLER_D);

    private final PIDController turnController = new PIDController(
        Constants.TURN_CONTROLLER_P,
        Constants.TURN_CONTROLLER_I,
        Constants.TURN_CONTROLLER_D
    );
    
    private final DifferentialDrive robotDrive;
    private final Joystick joystick;

    // Makes it so the motors don't suddenly start when someone gives it full throttle
    SlewRateLimiter forwardBackwardLimiter = new SlewRateLimiter(Constants.FORWARD_BACKWARD_SLEW_RATE);
    SlewRateLimiter rotationLimiter = new SlewRateLimiter(Constants.ROTATION_SLEW_RATE);

    public Drivetrain(Joystick joystick) {
        this.joystick = joystick;
        controller.setTolerance(0.1,10);
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
        forwardValue= forwardBackwardLimiter.calculate(joystick.getY()*factor);
        rotationValue=  rotationLimiter.calculate(joystick.getX()*factor);

        var result = camera.getLatestResult();

        SmartDashboard.putBoolean("AprilTagFound", result.hasTargets());
        if(result.hasTargets()){
        
            PhotonTrackedTarget target = result.getBestTarget();
        
            double range = PhotonUtils.calculateDistanceToTargetMeters(
                0.177,
                1.029,
                0,
                Units.degreesToRadians(target.getPitch()));
            
            SmartDashboard.putNumber("AprilTagDistance", range);
            
            if (joystick.getRawButton(2)){
                forwardValue = controller.calculate(range,4.0);
               
                forwardValue= MathUtil.clamp(forwardValue,-Constants.AUTO_TARGET_DRIVE_SPEED, Constants.AUTO_TARGET_DRIVE_SPEED);

            }
            if (joystick.getRawButton(5)){
                rotationValue = -turnController.calculate(result.getBestTarget().getYaw(), 0);
                 rotationValue= MathUtil.clamp(rotationValue,-Constants.AUTO_TARGET_DRIVE_SPEED,Constants.AUTO_TARGET_DRIVE_SPEED);
            }
        }

        robotDrive.arcadeDrive(-forwardValue, -rotationValue );
    }

}
