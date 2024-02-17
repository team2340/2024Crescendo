package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.commands.AutonomousMode;
import frc.robot.commands.IngestNote;
import frc.robot.commands.Shoot;
import frc.robot.subsystems.*;


public class RobotContainer {
    
    private static RobotContainer instance;
    private final Joystick driverJoyStick = new Joystick(Constants.DRIVER_JOYSTICK_PORT);
    private final Drivetrain drivetrain = new Drivetrain(driverJoyStick);
    private final Shooter shooter = new Shooter();
    private final Climber climber = new Climber();

    private final Shoot shootCommand = new Shoot(shooter);
    private final IngestNote ingestNoteCommand = new IngestNote(shooter, driverJoyStick);

    public RobotContainer() {
        instance = this;

        drivetrain.setDefaultCommand(
            new RepeatCommand(
                new InstantCommand(() -> drivetrain.drive(), drivetrain)
            )
        );
        
        shooter.setDefaultCommand(
            new RepeatCommand(
                new InstantCommand(() -> shooter.test(), shooter)
            )
        );
        
        // Button to make it so the shooter sucks in a ring
        new POVButton(driverJoyStick, 180)
            .onTrue( ingestNoteCommand );


        // Button to run the shoot command
        new POVButton(driverJoyStick, 0)
            .onTrue( shootCommand );
            


        // Button to "center" the ring in the shooter
        new JoystickButton(driverJoyStick, 6)
        .onTrue( new InstantCommand( () -> shooter.moveForward()))
        .onFalse(new InstantCommand( () -> shooter.stopShooter()));
        

        // Button to feed the ring into the amplifier
        new JoystickButton(driverJoyStick, 4)
        .onTrue( new InstantCommand( () -> shooter.feedAmplifier()))
        .onFalse(new InstantCommand( () -> shooter.stopShooter()));


        // Climber Down
        new JoystickButton(driverJoyStick, 5)
        .onTrue( new InstantCommand( () -> climber.goDown()))
        .onFalse(new InstantCommand( () -> climber.stopClimber()));

        // Climber Up
        new JoystickButton(driverJoyStick, 3)
        .onTrue( new InstantCommand( () -> climber.goUp()))
        .onFalse(new InstantCommand( () -> climber.stopClimber()));

    }

    public AutonomousMode getAutonomousCommand(){

        return new AutonomousMode(drivetrain, shooter);
    }

    public static RobotContainer getInstance(){
		return instance;
	}
}
