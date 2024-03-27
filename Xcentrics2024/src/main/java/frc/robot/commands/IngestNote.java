package frc.robot.commands;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Shooter;
import frc.robot.utils.OperationMode;

public class IngestNote extends Command {
    private boolean isIngesting = true;
    
    private long startTime;
    private long noteSeenTime = -1;
    private final Shooter shooter;
    private final Joystick joystick;
    
    public IngestNote (Shooter shooter, Joystick joystick ){
        addRequirements(shooter);
        this.shooter = shooter;
        this.joystick = joystick;
        SmartDashboard.putString("ShooterSubsystemStatus", "Idle");
    }

    @Override
    public void execute()
    {
        if( RobotContainer.getInstance().ingestingOperationMode.getSelected() == OperationMode.AUTOMATIC ) {
            executeAutoOperationMode();
        } else if( RobotContainer.getInstance().ingestingOperationMode.getSelected() == OperationMode.MANUAL ) {
            executeManualOperationMode();
        }
    }

    /*
     * Run this function when the ingest is in automatic mode. Automatic mode does the following:
     *  - Run both the shooter motor and the feeder motors in reverse
     *  - When a note is detected by the color sensor, stop the shooter motor
     *  - Keep the feeder motor running backwards for Constants.SHOOTER_INGEST_FEEDER_DWELL_TIME_MILLISECONDS
     *  - After Constants.SHOOTER_INGEST_FEEDER_DWELL_TIME_MILLISECONDS, stop the feeder motor
     */
    private void executeAutoOperationMode() {
        // Cancel the command
        if( joystick.getPOV() == 180 && (System.currentTimeMillis() - startTime) > 1000) {
            isIngesting = false;
        }

        // When we see the note for the first time, stop the shooter motors but keep the feed motors running
        if( shooter.isColorSensorHasNote() && !shooter.isNotePresent() ) {
            shooter.setIsNotePresent( true );
            noteSeenTime = System.currentTimeMillis();
            shooter.stopShooterMotors();
        }

        if( !shooter.isNotePresent() ) {
            SmartDashboard.putString("ShooterSubsystemStatus", "Ingesting... Waiting for Note...");
            shooter.ingest();
        } else {
            SmartDashboard.putString("ShooterSubsystemStatus", "Note Detected... Dwelling feeder motors...");
            if( System.currentTimeMillis() - noteSeenTime > Constants.SHOOTER_INGEST_FEEDER_DWELL_TIME_MILLISECONDS) {
                isIngesting = false;
                shooter.stopShooter();
            }
        }
    }

    /*
     * Run this function when the ingest is in manual mode. Manual mode should be used if something isn't working correctly
     * during a match. This allows a driver to manually jog the shooter motors and feeder motors
     */
    private void executeManualOperationMode() {
        // Manual Override mode
        if( joystick.getPOV() == 180 ) {
            SmartDashboard.putString("ShooterSubsystemStatus", "Ingesting... Manual mode...");
            isIngesting = true;
            shooter.ingest();
        } else {
            isIngesting = false;
            shooter.stopShooter();
        }
    }

    @Override
    public void end(boolean interrupted)
    {
        shooter.stopShooter();
        SmartDashboard.putString("ShooterSubsystemStatus", "Idle");
    }

    @Override
    public void initialize()
    {
        shooter.setIsNotePresent( false );
        noteSeenTime = -1;
        isIngesting = true;
        startTime = System.currentTimeMillis();
    }

    @Override
    public boolean isFinished()
    {
        return isIngesting == false;
    }

}