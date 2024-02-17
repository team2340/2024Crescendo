package frc.robot.subsystems;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonTrackedTarget;

import com.ctre.phoenix.motorcontrol.NeutralMode;
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
        controller.setTolerance(0.05,20);
        leftFront.setInverted(false);
        leftRear.setInverted(false);
        rightFront.setInverted(true);
        rightRear.setInverted(true);

        //leftFront.setNeutralMode(NeutralMode.Coast);
        //leftRear.setNeutralMode(NeutralMode.Coast);
        //rightFront.setNeutralMode(NeutralMode.Coast);
        //rightRear.setNeutralMode(NeutralMode.Coast);

        leftRear.follow(leftFront);
        rightRear.follow(rightFront);

        robotDrive = new DifferentialDrive(leftFront, rightFront);
        robotDrive.setSafetyEnabled(true);

        SmartDashboard.putBoolean("AprilTagFound", false);
        SmartDashboard.putNumber("AprilTagDistance", 0);
    }

    public void drive(){
        if(Robot.getInstance().isTeleop()){
            double forwardValue = 0.0;
            double rotationValue = 0.0;
            double factor = 0.0; 

            factor = (joystick.getTwist()-1)/2; 
            factor= (factor*-1);
            forwardValue = forwardBackwardLimiter.calculate(joystick.getY()*factor);
            rotationValue = rotationLimiter.calculate(joystick.getX()*factor);

            var result = camera.getLatestResult();
            SmartDashboard.putBoolean("AprilTagFound", result.hasTargets());
            if(result.hasTargets()){
                PhotonTrackedTarget target = result.getBestTarget();
                double range = PhotonUtils.calculateDistanceToTargetMeters(
                    Constants.CAMERA_HEIGHT_METERS,
                    1.029,
                    Constants.CAMERA_PITCH_RADIANS,
                    Units.degreesToRadians(target.getPitch()));
                
                SmartDashboard.putNumber("AprilTagDistance", range);
                

                int targetId = target.getFiducialId();
                if( Constants.targetDistanceMap.containsKey( targetId) ) {
                    double desiredRange = Constants.targetDistanceMap.get(targetId)[0];
                    double desiredYaw = Constants.targetDistanceMap.get(targetId)[0];

                    if (joystick.getRawButton(2)){
                        forwardValue = controller.calculate(range, desiredRange);
                        forwardValue= MathUtil.clamp(forwardValue,-Constants.AUTO_TARGET_DRIVE_SPEED, Constants.AUTO_TARGET_DRIVE_SPEED);

                        rotationValue = -turnController.calculate(result.getBestTarget().getYaw(), desiredYaw);
                        rotationValue= MathUtil.clamp(rotationValue,-Constants.AUTO_TARGET_DRIVE_SPEED,Constants.AUTO_TARGET_DRIVE_SPEED);
                    }
                }
            }

            drive(forwardValue, rotationValue);
        }
        
    }

    public boolean hasTarget(Integer ... targetIDs) {
        var result = camera.getLatestResult();
        for( Integer targetID : targetIDs )
        {
            if( result != null && result.getTargets() != null )
            {
                return result.getTargets().stream().anyMatch(c -> { return c.getFiducialId() == targetID; });
            }
        }

        return false;

    }

    public void driveToTarget(int targetId) {
        if( hasTarget(targetId) == false )
        {
            return;
        }

        var result = camera.getLatestResult();
        SmartDashboard.putBoolean("AprilTagFound", result.hasTargets());
        if(result.hasTargets()){
            PhotonTrackedTarget target = result.getBestTarget();
            double range = PhotonUtils.calculateDistanceToTargetMeters(
                Constants.CAMERA_HEIGHT_METERS,
                1.029,
                Constants.CAMERA_PITCH_RADIANS,
                Units.degreesToRadians(target.getPitch()));
            
            SmartDashboard.putNumber("AprilTagDistance", range);
            
            if( Constants.targetDistanceMap.containsKey( targetId) ) {
                double desiredRange = Constants.targetDistanceMap.get(targetId)[0];
                double desiredYaw = Constants.targetDistanceMap.get(targetId)[0];

                double forwardValue = controller.calculate(range, desiredRange);
                forwardValue= MathUtil.clamp(forwardValue,-Constants.AUTO_TARGET_DRIVE_SPEED, Constants.AUTO_TARGET_DRIVE_SPEED);

                double rotationValue = -turnController.calculate(result.getBestTarget().getYaw(), desiredYaw);
                rotationValue= MathUtil.clamp(rotationValue,-Constants.AUTO_TARGET_DRIVE_SPEED,Constants.AUTO_TARGET_DRIVE_SPEED);

                drive(forwardValue, rotationValue);
            }
        }
    }

    public boolean isAtTarget(int targetId) {
        double targetThreshold = 0.2;

        var result = camera.getLatestResult();
        SmartDashboard.putBoolean("AprilTagFound", result.hasTargets());
        if(result.hasTargets()){
            PhotonTrackedTarget target = result.getBestTarget();
            double range = PhotonUtils.calculateDistanceToTargetMeters(
                Constants.CAMERA_HEIGHT_METERS,
                1.029,
                Constants.CAMERA_PITCH_RADIANS,
                Units.degreesToRadians(target.getPitch()));
            
            SmartDashboard.putNumber("AprilTagDistance", range);
            
            if( Constants.targetDistanceMap.containsKey( targetId) ) {
                double desiredRange = Constants.targetDistanceMap.get(targetId)[0];
                double desiredYaw = Constants.targetDistanceMap.get(targetId)[0];

                double forwardValue = controller.calculate(range, desiredRange);
                forwardValue= MathUtil.clamp(forwardValue,-Constants.AUTO_TARGET_DRIVE_SPEED, Constants.AUTO_TARGET_DRIVE_SPEED);

                double rotationValue = -turnController.calculate(result.getBestTarget().getYaw(), desiredYaw);
                rotationValue= MathUtil.clamp(rotationValue,-Constants.AUTO_TARGET_DRIVE_SPEED,Constants.AUTO_TARGET_DRIVE_SPEED);

                if(forwardValue < targetThreshold && forwardValue > -targetThreshold && rotationValue < targetThreshold && rotationValue > -targetThreshold) {
                    return true;
                }
            }
        }

        return false;
    }

    public void drive(double forwardValue, double rotationValue){

         robotDrive.arcadeDrive(-forwardValue, -rotationValue );
    
}

}
