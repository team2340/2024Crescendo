package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterAngle;
import frc.robot.subsystems.ShooterAngle.ANGLE;

public class AutonomousDeployShooterAngle extends Command {
    long startTime;
    private final ShooterAngle shooterAngle;
    
    public AutonomousDeployShooterAngle (ShooterAngle shooterAngle){

        this.shooterAngle = shooterAngle;
        startTime = System.currentTimeMillis();

    }

    @Override
    public void execute()
    {
        shooterAngle.moveToAngle(ANGLE.SPEAKER);
    }

    @Override
    public void end(boolean interrupted)
    {

    }

    @Override
    public void initialize()
    {
        startTime = System.currentTimeMillis();
    }

    @Override
    public boolean isFinished()
    {
        return System.currentTimeMillis() - startTime > 500;
    }



}