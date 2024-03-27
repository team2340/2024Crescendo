package frc.robot.commands.autonomous;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
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
        if( DriverStation.getAlliance().get() == Alliance.Blue )
        {
            driveTrain.driveToTarget(7);
        }
        else
        {
            driveTrain.driveToTarget(4);
        }
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