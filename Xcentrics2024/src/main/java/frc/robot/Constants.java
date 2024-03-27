package frc.robot;

import java.util.LinkedHashMap;
import java.util.Map;
import java.util.Set;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import frc.robot.AutoDriveData.PLANE;

public class Constants {
    public static class Vision {
        public static final String kCameraName = "Arducam_OV9281_USB_Camera";
        // Cam mounted facing forward, half a meter forward of center, half a meter up from center.
        public static final Transform3d kRobotToCam = new Transform3d(new Translation3d(0.5, 0.0, 1.120775), new Rotation3d(0, 0.174533, 0));

        // The layout of the AprilTags on the field
        public static final AprilTagFieldLayout kTagLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();

        // The standard deviations of our vision estimated poses, which affect correction rate
        // (Fake values. Experiment and determine estimation noise on an actual robot.)
        public static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(4, 4, 8);
        public static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(0.5, 0.5, 1);
    }
    
    /**** START DRIVETRAIN RELATED VARIABLES ****/

    // Drivetrain wheelbase width
    public final static double DRIVETRAIN_WIDTH_METERS = 0.556;

    // Port number for the driver's joystick
    public final static int DRIVER_JOYSTICK_PORT = 0;
    public final static int SECOND_JOYSTICK_PORT = 1;

    // Output pins for the drivetrain
    public final static int DRIVETRAIN_LEFT_FRONT_MOTOR = 11;
    public final static int DRIVETRAIN_LEFT_REAR_MOTOR = 13;
    public final static int DRIVETRAIN_RIGHT_FRONT_MOTOR = 10;
    public final static int DRIVETRAIN_RIGHT_REAR_MOTOR = 12;

    // Odometry variables
    public final static double DRIVETRAIN_ENCODER_PULSES_PER_REVOLUTION = 11000;
    public final static double WHEEL_DIAMETER_METERS = 0.1524;

    // "Soft start" rates for forward/backwards, and also rotation
    public final static double FORWARD_BACKWARD_SLEW_RATE = 10;
    public final static double ROTATION_SLEW_RATE = 10;

    /**** END DRIVETRAIN RELATED VARIABLES ****/

    /**** START AUTO TARGETING VARIABLES ****/
    public final static double FORWARD_REVERSE_CONTROLLER_P = 1.5;
    public final static double FORWARD_REVERSE_CONTROLLER_I = 0.005;
    public final static double FORWARD_REVERSE_CONTROLLER_D = 0.006;

    public final static double TURN_CONTROLLER_P = 0.04;
    public final static double TURN_CONTROLLER_I = 0.005;
    public final static double TURN_CONTROLLER_D = 0.002;

    public final static double AUTO_TARGET_DRIVE_SPEED = 0.6;

    // Variable for autonomous mode that determines when the robot is "close enough" to the target and will switch on the shooter
    // Smaller values will make the robot be more precise to get to it's location, larger value will allow the robot to start shooting at a less precise of a location
    public final static double IS_AT_TARGET_THRESHOLD = 0.2;

    // Map for target IDs => desired distance, desired yaw
    public final static Map<Integer, AutoDriveData> targetDistanceMap = new LinkedHashMap<>();
    static
    {
        targetDistanceMap.put(4, new AutoDriveData(PLANE.Y, 0.9091) ); // Red Speaker Center
        targetDistanceMap.put(3, new AutoDriveData(PLANE.Y, 0.9091) ); // Red Speaker Right
        
        targetDistanceMap.put(7, new AutoDriveData(PLANE.Y, 0.9091) ); // Blue Speaker Center
        targetDistanceMap.put(8, new AutoDriveData(PLANE.Y, 0.9091) ); // Blue Speaker Left

        targetDistanceMap.put(9, new AutoDriveData(PLANE.Y, 0.479) ); // Red Source Right
        targetDistanceMap.put(10, new AutoDriveData(PLANE.Y, 0.479) ); // Red Source Left

        targetDistanceMap.put(1, new AutoDriveData(PLANE.Y, 0.479) ); // Blue Source Right
        targetDistanceMap.put(2, new AutoDriveData(PLANE.Y, 0.479) ); // Blue Source Left

        targetDistanceMap.put(5, new AutoDriveData(PLANE.Y, 0.38) ); // Red Amplifier
        targetDistanceMap.put(6, new AutoDriveData(PLANE.Y, 0.38) ); // Blue Amplifier
    }

    public final static Set<Integer> amplifierAprilTagIds = Set.of( 6, 5 );
    public final static Set<Integer> speakerAprilTagIds = Set.of( 3, 4, 7, 8 );
    public final static Set<Integer> sourceAprilTagIds = Set.of( 1, 2, 9, 10 );
    
    /**** END AUTO TARGETING VARIABLES ****/

    /**** START SHOOTER RELATED VARIABLES ****/

    // Wait time to for the shooter motors to ramp up
    public final static double SHOOTER_RAMP_UP_WAIT_SECONDS = 0.2;

    // How long to run the feed motor for when shooting to the speaker
    public final static double SHOOTER_FEED_SPEAKER_DURATION_SECONDS = 1.0;

    // How long to run the feed motor for when shooting to the amplifier
    public final static double SHOOTER_FEED_AMPLIFIER_DURATION_SECONDS = 5.0;

    // Once the note is detected while ingesting, stop the shooter motors, but keep running the feeder motors for X time
    public final static double SHOOTER_INGEST_FEEDER_DWELL_TIME_MILLISECONDS = 10;

    // Color sensor reading for when the note has been detected
    public final static double NOTE_PRESENT_DISTANCE = 1000;

    // Speed for ingesting rings
    public final static double FEED_SPEED_SHOOTER_MOTOR = -0.2;
    public final static double FEED_SPEED_FEEDER_MOTOR = -0.2;

    // Angle values for the shooter
    public final static double SHOOTER_ANGLE_STOWED = 0.667;
    public final static double SHOOTER_ANGLE_SPEAKER = 0.762;
    public final static double SHOOTER_ANGLE_SOURCE = 0.790;
    public final static double SHOOTER_ANGLE_AMPLIFIER = 0.530;

    public final static int SHOOTER_MOTOR_CONTROLLER_1_CAN_ID = 57;
    public final static int SHOOTER_MOTOR_CONTROLLER_2_CAN_ID = 56;
    public final static int FEED_MOTOR_CONTROLLER_1_CAN_ID = 55;
    public final static int FEED_MOTOR_CONTROLLER_2_CAN_ID = 58;
    public final static int SHOOTER_MOTOR_PIVOT_CONTROLLER_CAN_ID = 8;
    
    /**** END SHOOTER RELATED VARIABLES ****/


    /**** START CLIMBER RELATED VARIABLES ****/
    public final static int CLIMBER_MOTOR_LEFT_CAN_ID = 5;
    public final static int CLIMBER_MOTOR_RIGHT_CAN_ID = 6;

    public final static double CLIMBER_MOTOR_MAX_ENCODER = 120.0;
    /**** END CLIMBER RELATED VARIABLES ****/

}
