package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.subsystems.Shooter;

public class Shoot extends SequentialCommandGroup {

    public Shoot( Shooter shooter ) {
        addCommands(
            new InstantCommand( () -> shooter.startShooter() ),
            new WaitCommand(Constants.SHOOTER_RAMP_UP_WAIT_SECONDS),
            new InstantCommand( () -> shooter.feed() ).onlyIf( () -> shooter.isShooting() ),
            new WaitCommand(Constants.SHOOTER_FEED_DURATION_SECONDS),
            new InstantCommand( () -> shooter.stopShooter())
        );
    }
}
