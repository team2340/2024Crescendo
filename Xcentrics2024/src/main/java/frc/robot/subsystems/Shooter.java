package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase{
    private boolean isShooting = false;

    private CANSparkMax shooterMotor1 = new CANSparkMax(Constants.SHOOTER_MOTOR_CONTROLLER_1_CAN_ID,MotorType.kBrushless);
    private CANSparkMax shooterMotor2 = new CANSparkMax(Constants.SHOOTER_MOTOR_CONTROLLER_2_CAN_ID,MotorType.kBrushless);
    private CANSparkMax feedMotor1 = new CANSparkMax(Constants.FEED_MOTOR_CONTROLLER_1_CAN_ID,MotorType.kBrushless);
    private CANSparkMax feedMotor2 = new CANSparkMax(Constants.FEED_MOTOR_CONTROLLER_2_CAN_ID,MotorType.kBrushless);

    public Shooter() {
        feedMotor1.setInverted(true);
        shooterMotor1.setInverted(true);
        shooterMotor2.setInverted(false);
    }

    public void moveForward() {
        shooterMotor1.set(0.1);
        shooterMotor2.set(0.1);
        feedMotor1.set(1);
        feedMotor2.set(1);
        isShooting = true;
    }
    public void feedAmplifier() {
        shooterMotor1.set(-0.3);
        shooterMotor2.set(-0.3);
        feedMotor1.set(-1);
        feedMotor2.set(-1);
        isShooting = true;
    }

    public void startShooter() {
        System.out.println("Starting shooter");
        shooterMotor1.set(1);
        shooterMotor2.set(1);
        isShooting = true;
    }

    public void stopShooter() {
        System.out.println("Stopping shooter");
        shooterMotor1.set(0);
        shooterMotor2.set(0);
        feedMotor1.set(0);
        feedMotor2.set(0);
        isShooting = false;
    }

    public void feed() {
        System.out.println("Feeding");
        feedMotor1.set(1);
        feedMotor2.set(1);
    }

    public void ingest() {
        System.out.println("Ingesting");
        shooterMotor1.set(Constants.FEED_SPEED_SHOOTER_MOTOR);
        shooterMotor2.set(Constants.FEED_SPEED_SHOOTER_MOTOR);
        feedMotor1.set(Constants.FEED_SPEED_FEEDER_MOTOR);
        feedMotor2.set(Constants.FEED_SPEED_FEEDER_MOTOR);
    }

    public boolean isShooting() {
        return isShooting;
    }


}

