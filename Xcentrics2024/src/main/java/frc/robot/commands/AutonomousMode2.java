package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.autonomous.AutonomousDeployShooterAngle;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.ShooterAngle;

public class AutonomousMode2 extends SequentialCommandGroup{
    
public AutonomousMode2(Drivetrain drivetrain, Shooter shooter, ShooterAngle shooterAngle){

    addRequirements(shooter);
    addRequirements(drivetrain);
    addCommands( 
        new AutonomousDeployShooterAngle(shooterAngle),
        new ShootSpeaker(shooter)
    );
}

}
