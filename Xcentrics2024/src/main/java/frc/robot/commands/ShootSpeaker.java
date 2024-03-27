package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.subsystems.Shooter;

public class ShootSpeaker extends SequentialCommandGroup {

    public ShootSpeaker( Shooter shooter ) {
        addRequirements(shooter);
        addCommands(
            new InstantCommand( () -> SmartDashboard.putString("ShooterSubsystemStatus", "Shooting Speaker...") ),
            new InstantCommand( () -> shooter.startShooter() ),
            new WaitCommand(Constants.SHOOTER_RAMP_UP_WAIT_SECONDS),
            new InstantCommand( () -> shooter.feedToShoot() ).onlyIf( () -> shooter.isShooting() ),
            new WaitCommand(Constants.SHOOTER_FEED_SPEAKER_DURATION_SECONDS),
            new InstantCommand( () -> shooter.stopShooter() ),
            new InstantCommand( () -> shooter.setIsNotePresent(false) ),
            new InstantCommand( () -> SmartDashboard.putString("ShooterSubsystemStatus", "Idle") )
        );
    }
}
