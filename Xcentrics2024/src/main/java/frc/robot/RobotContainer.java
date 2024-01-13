package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.Drivetrain;

public class RobotContainer {
    private static RobotContainer instance;
    private final Joystick joystick = new Joystick(Constants.DRIVER_JOYSTICK_PORT);
    private final Drivetrain drivetrain = new Drivetrain(joystick);
    

    public RobotContainer() {
        instance = this;

        drivetrain.setDefaultCommand(
            new RepeatCommand(
                new InstantCommand(() -> drivetrain.drive(), drivetrain)
            )
        );

        new JoystickButton(joystick, 1)
        .onTrue( new InstantCommand( () -> {
            System.out.println("Joystick Button 1 Pressed");
        }))
        .onFalse(new InstantCommand( () -> {
            System.out.println("Joystick Button 1 Released");
        }));


    }

    public static RobotContainer getInstance(){
		return instance;
	}
}
