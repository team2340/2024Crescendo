package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.autonomous.AutonomousDeployShooterAngle;
import frc.robot.commands.autonomous.AutonomousDriveForward;
import frc.robot.commands.autonomous.AutonomousDriveToTarget;
import frc.robot.commands.autonomous.AutonomousRotate;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.ShooterAngle;

public class AutonomousMode1 extends SequentialCommandGroup{
    
public AutonomousMode1(Drivetrain drivetrain, Shooter shooter, ShooterAngle shooterAngle){

    addRequirements(shooter);
    addRequirements(drivetrain);
    addCommands( 
        new AutonomousDeployShooterAngle(shooterAngle),
        new ShootSpeaker(shooter),
        new WaitCommand(1),
        new AutonomousDriveForward(drivetrain)
    );
}

}
