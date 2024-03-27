package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.commands.AutonomousMode1;
import frc.robot.commands.AutonomousMode2;
import frc.robot.commands.AutonomousMode3;
import frc.robot.commands.IngestNote;
import frc.robot.commands.ShootAmplifier;
import frc.robot.commands.ShootSpeaker;
import frc.robot.subsystems.*;
import frc.robot.subsystems.ShooterAngle.ANGLE;
import frc.robot.utils.Lights;
import frc.robot.utils.OperationMode;


public class RobotContainer {
    
    private static RobotContainer instance;
    private final Joystick driverJoyStick = new Joystick(Constants.DRIVER_JOYSTICK_PORT);
    private final Joystick secondJoyStick = new Joystick(Constants.SECOND_JOYSTICK_PORT);

    // Sets the autonomous program
    private final SendableChooser<Command> autonomousModeChooser = new SendableChooser<>();

    // If in Auto, use the sensor to automatically stop the ingestion. If in manual, the driver has to hold down the backwards button to move the motors, and when they let go,
    // the motors stop 
    public final SendableChooser<OperationMode> ingestingOperationMode = new SendableChooser<>();

    // If in Auto, the shooter angle will be set to whatever april tag is found. If in manual, then the user selects the angle using the dashboard
    public final SendableChooser<OperationMode> shooterAngleOperationMode = new SendableChooser<>();

    private final Drivetrain drivetrain = new Drivetrain(driverJoyStick);
    private final ShooterAngle shooterAngle = new ShooterAngle(drivetrain);

    private final Shooter shooter = new Shooter(shooterAngle);
    private final Climber climber = new Climber(secondJoyStick);
    private final Lights lights = new Lights(drivetrain, shooter);

    private final ShootSpeaker shootSpeakerCommand = new ShootSpeaker(shooter);
    private final ShootAmplifier shootAmplifierCommand = new ShootAmplifier(shooter);
    private final IngestNote ingestNoteCommand = new IngestNote(shooter, driverJoyStick);
    private final Command smartShootCommand = new ConditionalCommand(shootSpeakerCommand, shootAmplifierCommand, () -> ( shooterAngle.getCurrentAngle() == ANGLE.SPEAKER || shooterAngle.getCurrentAngle() == ANGLE.SOURCE ) );

    private final Command shooterAngleJogUpManualCommand = new InstantCommand( () -> shooterAngle.setJogSpeed(-1) );
    private final Command shooterAngleJogDownManualCommand = new InstantCommand( () -> shooterAngle.setJogSpeed(1) );
    private final Command shooterAngleCycleNextCommand = new InstantCommand( () -> shooterAngle.cycleNextPosition() );
    private final Command shooterAngleCyclePrevCommand = new InstantCommand( () -> shooterAngle.cyclePrevPosition() );   

    private final Command shooterAngleJogUpCommand = new ConditionalCommand( shooterAngleCycleNextCommand, shooterAngleJogUpManualCommand, () -> shooterAngleOperationMode.getSelected() != OperationMode.MANUAL);
    private final Command shooterAngleJogDownCommand = new ConditionalCommand( shooterAngleCyclePrevCommand, shooterAngleJogDownManualCommand, () -> shooterAngleOperationMode.getSelected() != OperationMode.MANUAL );

    public RobotContainer() {
        CameraServer.startAutomaticCapture();
        
        instance = this;

        autonomousModeChooser.setDefaultOption("Disabled", null );
        autonomousModeChooser.addOption("Shoot, Drive Away", new AutonomousMode1(drivetrain, shooter, shooterAngle) );
        autonomousModeChooser.addOption("Shoot only", new AutonomousMode2(drivetrain, shooter, shooterAngle) );
        autonomousModeChooser.addOption("Drive away", new AutonomousMode3(drivetrain, shooter, shooterAngle) );

        ingestingOperationMode.setDefaultOption("AUTO", OperationMode.AUTOMATIC);
        ingestingOperationMode.addOption("MANUAL OVERRIDE", OperationMode.MANUAL);

        shooterAngleOperationMode.setDefaultOption("AUTO W/ APRIL TAGS", OperationMode.AUTOMATIC_WITH_APRILTAG);
        shooterAngleOperationMode.addOption("AUTO POSITION", OperationMode.AUTOMATIC);
        shooterAngleOperationMode.addOption("MANUAL JOG", OperationMode.MANUAL);

        SmartDashboard.putData("Autonomous Chooser", autonomousModeChooser);
        SmartDashboard.putData("Note Ingest Operation Mode", ingestingOperationMode);
        SmartDashboard.putData("Shooter Angle Operation Mode", shooterAngleOperationMode);
        

        climber.setDefaultCommand(
            new RepeatCommand(
                new InstantCommand( () -> climber.doClimber(), climber)
            )
        );

        drivetrain.setDefaultCommand(
            new RepeatCommand(
                new InstantCommand(() -> drivetrain.drive(), drivetrain)
            )
        );

        shooterAngle.setDefaultCommand(
            new RepeatCommand(
                new InstantCommand( () -> shooterAngle.periodic(), shooterAngle)
            )
        );

        shooter.setDefaultCommand(
            new RepeatCommand(
                new InstantCommand( () -> shooter.doPeriodic(), shooter)
            )
        );

        lights.setDefaultCommand(
            new RepeatCommand(
                new InstantCommand( () -> lights.doPeriodic(), lights)
            )
        );
        
        // Button to make it so the shooter sucks in a note
        new POVButton(driverJoyStick, 180).onTrue( ingestNoteCommand );

        // Button to run the shoot command
        new POVButton(driverJoyStick, 0).onTrue( smartShootCommand );

        // Jog the shooter angle up
        new JoystickButton(driverJoyStick, 3)
        .onTrue( shooterAngleJogUpCommand )
        .onFalse(new InstantCommand( () -> shooterAngle.setJogSpeed(0)));

        // Jog the shooter angle down
        new JoystickButton(driverJoyStick, 5)
        .onTrue( shooterAngleJogDownCommand )
        .onFalse(new InstantCommand( () -> shooterAngle.setJogSpeed(0)));


        // Secondary Joystick operations

        // Jog the shooter angle up
        new JoystickButton(secondJoyStick, 3)
        .onTrue( shooterAngleJogUpCommand )
        .onFalse(new InstantCommand( () -> shooterAngle.setJogSpeed(0)));

        // Jog the shooter angle down
        new JoystickButton(secondJoyStick, 5)
        .onTrue( shooterAngleJogDownCommand )
        .onFalse(new InstantCommand( () -> shooterAngle.setJogSpeed(0)));

        // Jog the note forward
        new JoystickButton(secondJoyStick, 6)
        .onTrue( new InstantCommand( () -> shooter.moveNoteForward(), shooter) )
        .onFalse(new InstantCommand( () -> shooter.stopShooter(), shooter) );

        // Jog the note rearward
        new JoystickButton(secondJoyStick, 4)
        .onTrue( new InstantCommand( () -> shooter.moveNoteRearward(), shooter) )
        .onFalse(new InstantCommand( () -> shooter.stopShooter(), shooter) );

        // Shoot into the amplifier
        new JoystickButton(secondJoyStick, 1)
        .onTrue( new ShootAmplifier(shooter) );
        
        // Button to make it so the shooter sucks in a note
        new POVButton(secondJoyStick, 180)
        .onTrue( new InstantCommand( () -> shooter.ingest(), shooter ) )
        .onFalse( new InstantCommand( () -> shooter.stopShooter(), shooter ) );

        // Button to run the shoot command
        new POVButton(secondJoyStick, 0).onTrue( new ShootSpeaker(shooter) );      

    }

    public Command getAutonomousCommand(){
        return autonomousModeChooser.getSelected();
    }

    public static RobotContainer getInstance(){
		return instance;
	}
}
