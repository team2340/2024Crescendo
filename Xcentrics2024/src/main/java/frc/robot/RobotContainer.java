package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.commands.Shoot;
import frc.robot.subsystems.*;


public class RobotContainer {
    
    private static RobotContainer instance;
    private final Joystick joystick = new Joystick(Constants.DRIVER_JOYSTICK_PORT);
    private final Drivetrain drivetrain = new Drivetrain(joystick);
    private final Shooter shooter = new Shooter();

    private final Shoot shootCommand = new Shoot(shooter);

    public RobotContainer() {
        instance = this;

        drivetrain.setDefaultCommand(
            new RepeatCommand(
                new InstantCommand(() -> drivetrain.drive(), drivetrain)
            )
        );

        new POVButton(joystick, 180)
            .onTrue( new InstantCommand( () -> shooter.ingest() ) )
            .onFalse( new InstantCommand( () -> shooter.stopShooter() ) );

        new POVButton(joystick, 0)
            .onTrue( shootCommand )
            .onFalse(new InstantCommand( () -> shooter.stopShooter() ) );


    }

    public static RobotContainer getInstance(){
		return instance;
	}
}
