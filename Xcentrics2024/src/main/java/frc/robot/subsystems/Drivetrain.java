package frc.robot.subsystems;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.AutoDriveData;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.AutoDriveData.PLANE;
import frc.robot.vision.Vision;

import java.util.Optional;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonTrackedTarget;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.Joystick;


public class Drivetrain extends SubsystemBase{
    private final WPI_TalonSRX leftFront = new WPI_TalonSRX(Constants.DRIVETRAIN_LEFT_FRONT_MOTOR);
    private final WPI_TalonSRX leftRear = new WPI_TalonSRX(Constants.DRIVETRAIN_LEFT_REAR_MOTOR);
    private final WPI_TalonSRX rightFront = new WPI_TalonSRX(Constants.DRIVETRAIN_RIGHT_FRONT_MOTOR);
    private final WPI_TalonSRX rightRear = new WPI_TalonSRX(Constants.DRIVETRAIN_RIGHT_REAR_MOTOR);
    private final PhotonCamera camera = new PhotonCamera(Constants.Vision.kCameraName);

    /// Odometry and Pose Estimation variables
    private ADXRS450_Gyro m_gyro = new ADXRS450_Gyro();
    private final DifferentialDrivePoseEstimator m_poseEstimator;
    private Field2d m_field = new Field2d();
    private final Vision vision;

    private static final double meters_per_pulse = (Math.PI * Constants.WHEEL_DIAMETER_METERS) / Constants.DRIVETRAIN_ENCODER_PULSES_PER_REVOLUTION; 

    private final PIDController controller = new PIDController(
        Constants.FORWARD_REVERSE_CONTROLLER_P,
        Constants.FORWARD_REVERSE_CONTROLLER_I,
        Constants.FORWARD_REVERSE_CONTROLLER_D);

    private final PIDController turnController = new PIDController(
        Constants.TURN_CONTROLLER_P,
        Constants.TURN_CONTROLLER_I,
        Constants.TURN_CONTROLLER_D
    );

    /// Automatic driving speed/rotation values for tracking targets. Pass these
    /// two variables into the drive() function to make the robot drive to a AprilTag
    private double AUTO_DRIVE_FORWARD_DRIVE_VALUE = 0.0;
    private double AUTO_DRIVE_ROTATION_DRIVE_VALUE = 0.0;
    
    private final DifferentialDrive robotDrive;
    private final Joystick joystick;

    // Makes it so the motors don't suddenly start when someone gives it full throttle
    SlewRateLimiter forwardBackwardLimiter = new SlewRateLimiter(Constants.FORWARD_BACKWARD_SLEW_RATE);
    SlewRateLimiter rotationLimiter = new SlewRateLimiter(Constants.ROTATION_SLEW_RATE);

    private boolean hasAnyTargets = false;
    private boolean isDrivingToTarget = false;

    public Drivetrain(Joystick joystick) {
        this.joystick = joystick;
        vision = new Vision();
        controller.setTolerance(0.1,20);
        //turnController.setTolerance(0.05, 20);
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

        leftFront.setSelectedSensorPosition(0);
        rightFront.setSelectedSensorPosition(0);
        
        robotDrive = new DifferentialDrive(leftFront, rightFront);
        robotDrive.setSafetyEnabled(true);

        SmartDashboard.putBoolean("AprilTagFound", false);
        SmartDashboard.putNumber("AprilTagDistance", 0);
        SmartDashboard.putData("Auto Drive Forward PID Controller", controller);
        SmartDashboard.putData("Auto Drive Rotation PID Controller", turnController);

        DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(Constants.DRIVETRAIN_WIDTH_METERS);
        
        m_poseEstimator = new DifferentialDrivePoseEstimator(
            kinematics,
            m_gyro.getRotation2d(),
            leftFront.getSelectedSensorPosition(),
            rightFront.getSelectedSensorPosition(),
            new Pose2d(),
            VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(5)),
            VecBuilder.fill(0.5, 0.5, Units.degreesToRadians(30)));
        SmartDashboard.putData("Field", m_field);
    }

    /*
     * Main drive loop. This should get called often
     */
    public void drive(){
        // Update the pose estimator with the gyro and encoder positions
        m_poseEstimator.update(m_gyro.getRotation2d(), getLeftEncoderDistanceMeters(), getRightEncoderDistanceMeters());
        var visionEst = vision.getEstimatedGlobalPose();
        visionEst.ifPresent(
            est -> {
                var estPose = est.estimatedPose.toPose2d();
                // Change our trust in the measurement based on the tags we can see
                var estStdDevs = vision.getEstimationStdDevs(estPose);
                m_poseEstimator.addVisionMeasurement(est.estimatedPose.toPose2d(), est.timestampSeconds, estStdDevs);
            });
        m_field.setRobotPose(m_poseEstimator.getEstimatedPosition());

        // Update auto drive values if the camera sees any AprilTags that it knows how to drive to
        calculateAutoDriveSpeedAndRotation();

        if(Robot.getInstance().isTeleop()){
            double forwardValue = 0.0;
            double rotationValue = 0.0;
            double factor = 0.0; 

            factor = ( joystick.getThrottle() - 1 ) / 2; 
            factor = factor * -1;
            forwardValue = forwardBackwardLimiter.calculate( joystick.getY() * factor );
            rotationValue = rotationLimiter.calculate( joystick.getX() * factor );

            var result = camera.getLatestResult();

            SmartDashboard.putBoolean("AprilTagFound", result.hasTargets());
            
            if (joystick.getRawButton(1)){
                forwardValue = AUTO_DRIVE_FORWARD_DRIVE_VALUE;
                rotationValue = AUTO_DRIVE_ROTATION_DRIVE_VALUE;
            }

            drive(forwardValue, rotationValue);
        }
        
    }

    public boolean hasAnyTargets()
    {
        return hasAnyTargets;
    }

    public boolean isDrivingToTarget() 
    {
        return hasAnyTargets() && joystick.getRawButton(1);
    }

    /*
     *  Given a list of targets that it is expecting, check if the camera currently sees any of the targets
     */
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

    /*
     * Drives to a specified target
     */
    public void driveToTarget(int targetId) {
        if( hasTarget(targetId) == false )
        {
            return;
        }
        calculateAutoDriveSpeedAndRotation();
        drive(AUTO_DRIVE_FORWARD_DRIVE_VALUE, AUTO_DRIVE_ROTATION_DRIVE_VALUE);
    }

    /*
     * Returns true if the robot has reached a target
     */
    public boolean isAtTarget(int targetId) {
        double targetThreshold = Constants.IS_AT_TARGET_THRESHOLD;

        calculateAutoDriveSpeedAndRotation();
        if( hasTarget(targetId) == false )
        {
            return false;
        }
        
        // Calculate that the robot is at a target when the drive values have slowed down to a slow enough point where it's effectively not moving
        if( AUTO_DRIVE_FORWARD_DRIVE_VALUE < targetThreshold && 
            AUTO_DRIVE_FORWARD_DRIVE_VALUE > -targetThreshold && 
            AUTO_DRIVE_ROTATION_DRIVE_VALUE < targetThreshold && 
            AUTO_DRIVE_ROTATION_DRIVE_VALUE > -targetThreshold ) {
            return true;
        }
        return false;
    }

    /*
     * This takes vision information and calculates the forward/reverse value and rotation value just if the robot needs to drive automatically to the target
     */
    private void calculateAutoDriveSpeedAndRotation() {
        var result = camera.getLatestResult();

        SmartDashboard.putBoolean("AprilTagFound", result.hasTargets());
        if(result.hasTargets()){                 
            for( Integer knownTargetId : Constants.targetDistanceMap.keySet() ) {
                for( PhotonTrackedTarget target : result.getTargets() ) {
                    int targetId = target.getFiducialId();
                    if( knownTargetId == targetId ) {
                        Optional<Pose3d> tagPose = Constants.Vision.kTagLayout.getTagPose(targetId);
                        
                        double targetHeightMeters = tagPose.isEmpty() ? 1.0 : tagPose.get().getZ();

                        double range = PhotonUtils.calculateDistanceToTargetMeters(
                            Constants.Vision.kRobotToCam.getZ(),
                            targetHeightMeters,
                            Constants.Vision.kRobotToCam.getRotation().getY(),
                            Units.degreesToRadians(target.getPitch()));
                        
                                
                        double desiredRange = Constants.targetDistanceMap.get(targetId).targetDistance;
                        double desiredYaw = calculateDesiredYawForAutoDrive(m_poseEstimator.getEstimatedPosition(), Constants.targetDistanceMap.get(targetId), range, tagPose.get() );
                        desiredYaw =0;
                        SmartDashboard.putNumber("AprilTagDistance", range);
                        SmartDashboard.putNumber("Desired Range", desiredRange);
                        SmartDashboard.putNumber("Desired Yaw", desiredYaw);
                        SmartDashboard.putNumber("Current Range", range);
                        SmartDashboard.putNumber("Current Yaw", target.getYaw());

                        AUTO_DRIVE_FORWARD_DRIVE_VALUE = controller.calculate(range, desiredRange);
                        AUTO_DRIVE_FORWARD_DRIVE_VALUE = MathUtil.clamp(AUTO_DRIVE_FORWARD_DRIVE_VALUE,-Constants.AUTO_TARGET_DRIVE_SPEED, Constants.AUTO_TARGET_DRIVE_SPEED);
                        SmartDashboard.putNumber("AUTO_DRIVE_FORWARD_VALUE", AUTO_DRIVE_FORWARD_DRIVE_VALUE);

                        AUTO_DRIVE_ROTATION_DRIVE_VALUE = -turnController.calculate(target.getYaw(), desiredYaw);
                        AUTO_DRIVE_ROTATION_DRIVE_VALUE = MathUtil.clamp(AUTO_DRIVE_ROTATION_DRIVE_VALUE,-Constants.AUTO_TARGET_DRIVE_SPEED,Constants.AUTO_TARGET_DRIVE_SPEED);
                        SmartDashboard.putNumber("AUTO_DRIVE_ROTATION_VALUE", AUTO_DRIVE_ROTATION_DRIVE_VALUE);

                        hasAnyTargets = true;
                        return;
                    }
                }
            }
        }
        
        hasAnyTargets = false;
        AUTO_DRIVE_FORWARD_DRIVE_VALUE = 0.0;
        AUTO_DRIVE_ROTATION_DRIVE_VALUE = 0.0;
    }

    /*
     * When driving to a target, set either to drive further to the right or further to the left to make it so the robot
     * will end up more perpendicular to the april tag
     */ 
    private double calculateDesiredYawForAutoDrive(Pose2d currentRobotPose, AutoDriveData autoDriveData, double currentRange, Pose3d aprilTagPose)
    {
        double targetPosition = ( autoDriveData.plane == PLANE.Y ? aprilTagPose.getY() : aprilTagPose.getX() );
        double currentPosition = ( autoDriveData.plane == PLANE.Y  ? currentRobotPose.getY() : currentRobotPose.getX() );

        // We're close enough, just turn to 0
        if( autoDriveData.plane == PLANE.NONE || Math.abs( autoDriveData.targetDistance - currentRange) < 0.5 || Math.abs( currentPosition - targetPosition) < 0.5 )
        {
            return 0.0;
        }
        else
        {
            if( autoDriveData.plane == PLANE.Y && currentPosition < targetPosition )
            {
                return 20;
            }

            if( autoDriveData.plane == PLANE.Y && currentPosition > targetPosition )
            {
                return -20;
            }          

            if( autoDriveData.plane == PLANE.X && currentPosition < targetPosition )
            {
                return -20;
            }

            if( autoDriveData.plane == PLANE.X && currentPosition > targetPosition )
            {
                return 20;
            } 
        }

        return 0.0;
    }

    public void drive(double forwardValue, double rotationValue){
         robotDrive.arcadeDrive(-forwardValue, -rotationValue );
    }

    private double getLeftEncoderDistanceMeters()
    {
        return leftFront.getSelectedSensorPosition() * meters_per_pulse * -1;
    }

    private double getRightEncoderDistanceMeters()
    {        
        return rightFront.getSelectedSensorPosition() * meters_per_pulse * -1;
    }   
}
