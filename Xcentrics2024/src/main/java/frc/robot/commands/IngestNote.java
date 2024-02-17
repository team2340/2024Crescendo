package frc.robot.commands;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;

public class IngestNote extends Command {
    private boolean isIngesting = true;    
    private long startTime;
    private final Shooter shooter;
    private final Joystick joystick;
    
    public IngestNote (Shooter shooter, Joystick joystick ){
        addRequirements(shooter);
        this.shooter = shooter;
        this.joystick = joystick;

    }

    @Override
    public void execute()
    {
        shooter.ingest();
        if( joystick.getPOV() == 180 && (System.currentTimeMillis() - startTime) > 1000) {
            isIngesting = false;
        }
    }

    @Override
    public void end(boolean interrupted)
    {
        shooter.stopShooter();
    }

    @Override
    public void initialize()
    {
        isIngesting = true;
        startTime = System.currentTimeMillis();
    }

    @Override
    public boolean isFinished()
    {
      return shooter.isNoteLoaded() || isIngesting == false;
    }



}