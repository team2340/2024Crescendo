package frc.robot.commands.autonomous;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain;

public class AutonomousDriveForward extends Command {
    long startTime;
    private final Drivetrain driveTrain;
    
    public AutonomousDriveForward (Drivetrain drivetrain){

        this.driveTrain = drivetrain;
        startTime = System.currentTimeMillis();

    }

    @Override
    public void execute()
    {
        driveTrain.drive(0.6, 0.0);

    }

    @Override
    public void end(boolean interrupted)
    {
        driveTrain.drive(0.0, 0.0);
    }

    @Override
    public void initialize()
    {
        startTime = System.currentTimeMillis();
    }

    @Override
    public boolean isFinished()
    {
        return System.currentTimeMillis() - startTime > 1000;
    }



}