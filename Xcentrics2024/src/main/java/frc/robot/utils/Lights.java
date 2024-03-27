package frc.robot.utils;

import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Shooter;

public class Lights extends SubsystemBase {
    PWMSparkMax talon = new PWMSparkMax(0);


    public enum PATTERN {
        INTAKE(0.77),
        HAS_APRIL_TAG(0.65),
        DRIVING_APRIL_TAG(-0.63),
        SHOOTING(0.93),
        HAS_NOTE(0.57),
        DISABLED(0.99),
        DEFAULT(-0.69);

        private double pwmValue;
        PATTERN(double values)
        {
            this.pwmValue = values;
        }
    }

    private Drivetrain drivetrain;
    private Shooter shooter;

    public Lights( Drivetrain drivetrain, Shooter shooter)
    {
        this.drivetrain = drivetrain;
        this.shooter = shooter;
    }

    public void doPeriodic()
    {
        if( !Robot.getInstance().isEnabled() ) 
        {
            talon.set( PATTERN.DISABLED.pwmValue );
            return;
        }

        if( drivetrain.isDrivingToTarget() )
        {
            talon.set( PATTERN.DRIVING_APRIL_TAG.pwmValue );
            return;
        }

        if( drivetrain.hasAnyTargets() )
        {
            talon.set( PATTERN.HAS_APRIL_TAG.pwmValue );
            return;
        }   

        if( shooter.isIntaking() )
        {
            talon.set( PATTERN.INTAKE.pwmValue );
            return;
        }   

        if( shooter.isShooting() )
        {
            talon.set( PATTERN.SHOOTING.pwmValue );
            return;
        }   

        if( shooter.isNotePresent() )
        {
            talon.set(PATTERN.HAS_NOTE.pwmValue);
            return;
        }
        talon.set( PATTERN.DEFAULT.pwmValue );

    }

    
}
