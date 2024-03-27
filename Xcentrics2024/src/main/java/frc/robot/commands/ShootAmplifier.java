package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.subsystems.Shooter;

public class ShootAmplifier extends SequentialCommandGroup{

    public ShootAmplifier( Shooter shooter ) {
        addRequirements(shooter);
        addCommands(
            new InstantCommand( () -> SmartDashboard.putString("ShooterSubsystemStatus", "Shooting Amplifier...") ),
            new InstantCommand( () -> shooter.feedAmplifier() ),
            new WaitCommand(Constants.SHOOTER_FEED_AMPLIFIER_DURATION_SECONDS),
            new InstantCommand( () -> shooter.stopShooter() ),
            new InstantCommand( () -> shooter.setIsNotePresent(false) ),
            new InstantCommand( () -> SmartDashboard.putString("ShooterSubsystemStatus", "Idle") )
        );
    }
}
