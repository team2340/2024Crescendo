package frc.robot;

public class Constants {

    /**** DRIVETRAIN RELATED VARIABLES ****/

    // Port number for the driver's joystick
    public static int DRIVER_JOYSTICK_PORT = 0;

    // Output pins for the drivetrain
    public static int DRIVETRAIN_LEFT_FRONT_MOTOR = 5;
    public static int DRIVETRAIN_LEFT_REAR_MOTOR = 3;
    public static int DRIVETRAIN_RIGHT_FRONT_MOTOR = 4;
    public static int DRIVETRAIN_RIGHT_REAR_MOTOR = 2;

    // "Soft start" rates for forward/backwards, and also rotation
    public static double FORWARD_BACKWARD_SLEW_RATE = 3;
    public static double ROTATION_SLEW_RATE = 3;

    // Auto Targeting Variables //
    public static double FORWARD_REVERSE_CONTROLLER_P = 3.0;
    public static double FORWARD_REVERSE_CONTROLLER_I = 0.0;
    public static double FORWARD_REVERSE_CONTROLLER_D = 5.0;

    public static double TURN_CONTROLLER_P = 0.04;
    public static double TURN_CONTROLLER_I = 0.0;
    public static double TURN_CONTROLLER_D = 0.01;

    public static double AUTO_TARGET_DRIVE_SPEED = 0.6;
    
    // Auto Targeting Variables //

    /**** DRIVETRAIN RELATED VARIABLES ****/

    /**** SHOOTER RELATED VARIABLES ****/

    // Wait to for the shooter motors to ramp up
    public static double SHOOTER_RAMP_UP_WAIT_SECONDS = 1.0;

    // How long to run the feed motor for when shooting
    public static double SHOOTER_FEED_DURATION_SECONDS = 1.0;

    // Speed for ingesting rings
    public static double FEED_SPEED_SHOOTER_MOTOR = -0.1;
    public static double FEED_SPEED_FEEDER_MOTOR = -0.1;


    public static int SHOOTER_MOTOR_CONTROLLER_1_CAN_ID = 57;
    public static int SHOOTER_MOTOR_CONTROLLER_2_CAN_ID = 56;
    public static int FEED_MOTOR_CONTROLLER_1_CAN_ID = 55;
    public static int FEED_MOTOR_CONTROLLER_2_CAN_ID = 58;
    
    /**** SHOOTER RELATED VARIABLES ****/

}
