package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.ColorSensorV3.ColorSensorMeasurementRate;
import com.revrobotics.ColorSensorV3.ColorSensorResolution;
import com.revrobotics.ColorSensorV3.GainFactor;
import com.revrobotics.CANSparkMax;

import com.revrobotics.ColorSensorV3;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ShooterAngle.ANGLE;
import frc.robot.utils.OperationMode;

public class Shooter extends SubsystemBase{
    private boolean isIntaking = false;
    private boolean isShooting = false;
    private boolean noteSeen = false;


    private final ShooterAngle shooterAngle;
    private CANSparkMax shooterMotor1 = new CANSparkMax(Constants.SHOOTER_MOTOR_CONTROLLER_1_CAN_ID,MotorType.kBrushless);
    private CANSparkMax shooterMotor2 = new CANSparkMax(Constants.SHOOTER_MOTOR_CONTROLLER_2_CAN_ID,MotorType.kBrushless);
    private CANSparkMax feedMotor1 = new CANSparkMax(Constants.FEED_MOTOR_CONTROLLER_1_CAN_ID,MotorType.kBrushless);
    private CANSparkMax feedMotor2 = new CANSparkMax(Constants.FEED_MOTOR_CONTROLLER_2_CAN_ID,MotorType.kBrushless);

    ColorSensorV3 colorSensorV3 = new ColorSensorV3(I2C.Port.kOnboard);

    public Shooter( ShooterAngle shooterAngle ) {
        this.shooterAngle = shooterAngle;
        SmartDashboard.putBoolean("Auto Reposition Note", true);
        feedMotor1.setInverted(false);
        feedMotor2.setInverted(true);
        shooterMotor1.setInverted(false);
        shooterMotor2.setInverted(true);

        shooterMotor1.setOpenLoopRampRate(0.3);
        shooterMotor2.setOpenLoopRampRate(0.3);

        shooterMotor1.setClosedLoopRampRate(0.2);
        shooterMotor2.setClosedLoopRampRate(0.2);

        feedMotor1.setIdleMode(IdleMode.kBrake);
        feedMotor2.setIdleMode(IdleMode.kBrake);
        shooterMotor1.setIdleMode(IdleMode.kBrake);
        shooterMotor2.setIdleMode(IdleMode.kBrake);

        colorSensorV3.configureColorSensor(
            ColorSensorResolution.kColorSensorRes20bit,
            ColorSensorMeasurementRate.kColorRate25ms,
            GainFactor.kGain18x);

        SmartDashboard.putString("ShooterFeederStatus", "Idle");
        SmartDashboard.putString("ShooterShooterStatus", "Idle");
    }

    /*
     * Move the note forward slowly
     */
    public void moveNoteForward() {
        SmartDashboard.putString("ShooterFeederStatus", "Moving note forward");
        feedMotor1.set(0.2);
        feedMotor2.set(0.2);
        isShooting = true;
    }

    /*
     * Move the note rearward slowly
     */
    public void moveNoteRearward() {
        SmartDashboard.putString("ShooterFeederStatus", "Moving note rearward");
        feedMotor1.set(-0.2);
        feedMotor2.set(-0.2);
        isShooting = true;
    }

    /*
     * Run the motors in reverse to feed the note into the amplifier
     */
    public void feedAmplifier() {
        SmartDashboard.putString("ShooterFeederStatus", "Reversing");
        SmartDashboard.putString("ShooterShooterStatus", "Reversing");
        shooterMotor1.set(-0.3);
        shooterMotor2.set(-0.3);
        feedMotor1.set(-1);
        feedMotor2.set(-1);
        isShooting = true;
    }

    /*
     * Start spinning the shooter motors to get ready to
     * shoot the note
     */
    public void startShooter() {
        SmartDashboard.putString("ShooterShooterStatus", "Shooting");
        shooterMotor1.set(1);
        shooterMotor2.set(1);
        isShooting = true;
    }

    /*
     * Stops all the motors in the shooter
     */
    public void stopShooter() {
        SmartDashboard.putString("ShooterFeederStatus", "Idle");
        SmartDashboard.putString("ShooterShooterStatus", "Idle");
        shooterMotor1.set(0);
        shooterMotor2.set(0);
        feedMotor1.set(0);
        feedMotor2.set(0);
        isShooting = false;
        isIntaking = false;

    }

    /*
     * Stops the front shooter motors
     */
    public void stopShooterMotors() {
        SmartDashboard.putString("ShooterShooterStatus", "Idle");
        shooterMotor1.set(0);
        shooterMotor2.set(0); 
        isIntaking = false;
    }

    /*
     * Start the feeder motors to send note forward into the shooter motors
     */
    public void feedToShoot() {
        SmartDashboard.putString("ShooterFeederStatus", "Feeding to shoot");
        feedMotor1.set(1);
        feedMotor2.set(1);
    }

    public void ingest() {
        SmartDashboard.putString("ShooterFeederStatus", "Ingesting");
        SmartDashboard.putString("ShooterShooterStatus", "Ingesting");

        shooterMotor1.set(Constants.FEED_SPEED_SHOOTER_MOTOR);
        shooterMotor2.set(Constants.FEED_SPEED_SHOOTER_MOTOR);
        feedMotor1.set(Constants.FEED_SPEED_FEEDER_MOTOR);
        feedMotor2.set(Constants.FEED_SPEED_FEEDER_MOTOR);
        isIntaking = true;
    }

    public void doPeriodic() {
        SmartDashboard.putNumber("ColorSensorDistance", colorSensorV3.getProximity() );
        SmartDashboard.putBoolean("ColorSensorHasNote", isColorSensorHasNote() );

        // This runs when the shooter isn't ingesting or shooting, and when the automatic modes are enabled
        if( RobotContainer.getInstance().ingestingOperationMode.getSelected() == OperationMode.AUTOMATIC &&
            (RobotContainer.getInstance().shooterAngleOperationMode.getSelected() == OperationMode.AUTOMATIC_WITH_APRILTAG || RobotContainer.getInstance().shooterAngleOperationMode.getSelected() == OperationMode.AUTOMATIC ) &&
             SmartDashboard.getBoolean("Auto Reposition Note", true))
        {
            /*
             * If the shooter angle is in the amplifier position AND there is a note, then jog the feeder motors forward to center the note.
             * If the shooter angle is in the speaker position AND there is no note , then jog the feeder motors reverse until there is a note
            */
            if( isNotePresent() && shooterAngle.getCurrentAngle() == ANGLE.AMPLIFIER && isColorSensorHasNote() )
            {
                SmartDashboard.putString("Auto Reposition Note Status", "Moving note forward");
                moveNoteForward();
            }
            else if( isNotePresent() && (shooterAngle.getCurrentAngle() == ANGLE.SPEAKER || shooterAngle.getCurrentAngle() == ANGLE.SOURCE)  && !isColorSensorHasNote() )
            {
                SmartDashboard.putString("Auto Reposition Note Status", "Moving note rearward");
                moveNoteRearward();
            }
            else
            {
                SmartDashboard.putString("Auto Reposition Note Status", "No Task");
                SmartDashboard.putString("ShooterFeederStatus", "Idle");
                feedMotor1.set(0);
                feedMotor2.set(0);
            }            
        } 
        else 
        {
            SmartDashboard.putString("Auto Reposition Note Status", "Not Running");
        }

    }

    public boolean isShooting() {
        return isShooting;
    }

    public boolean isIntaking() {
        return isIntaking;
    }
    public boolean isColorSensorHasNote()
    {
        return colorSensorV3.getProximity() > Constants.NOTE_PRESENT_DISTANCE;
    }

    public void setIsNotePresent(boolean seen) 
    {
        this.noteSeen = seen;
    }

    public boolean isNotePresent()
    {
        return this.noteSeen;
    }
}

