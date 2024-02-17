package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain;

public class AutonomousDriveToTarget extends Command {
    long startTime;
    private final Drivetrain driveTrain;
    
    public AutonomousDriveToTarget (Drivetrain drivetrain){

        this.driveTrain = drivetrain;
        startTime = System.currentTimeMillis();

    }

    @Override
    public void execute()
    {
        driveTrain.driveToTarget(1);

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
      return System.currentTimeMillis() - startTime > 10000 || driveTrain.isAtTarget(1);
    }



}