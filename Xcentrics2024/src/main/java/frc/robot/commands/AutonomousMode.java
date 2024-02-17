package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Shooter;

public class AutonomousMode extends SequentialCommandGroup{
    
public AutonomousMode(Drivetrain drivetrain, Shooter shooter){

    addRequirements(shooter);
    addRequirements(drivetrain);
    addCommands( 
        new AutonomousDriveForward(drivetrain),
        new AutonomousRotate(drivetrain),
        new AutonomousDriveToTarget(drivetrain),
        new Shoot(shooter)
    );
}

}
