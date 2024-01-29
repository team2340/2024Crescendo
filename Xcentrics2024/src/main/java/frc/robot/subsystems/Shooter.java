package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase{
    private boolean isShooting = false;

    private final Joystick joystick;
    private CANSparkMax shooterMotor1 = new CANSparkMax(57,MotorType.kBrushless);
    private CANSparkMax shooterMotor2 = new CANSparkMax(56,MotorType.kBrushless);
    private CANSparkMax feedMotor1 = new CANSparkMax(55,MotorType.kBrushless);
    private CANSparkMax feedMotor2 = new CANSparkMax(58,MotorType.kBrushless);

    public Shooter(Joystick joystick) {
        this.joystick = joystick;
        feedMotor1.setInverted(true);
        shooterMotor1.setInverted(true);
        shooterMotor2.setInverted(false);
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
        shooterMotor1.set(-0.1);
        shooterMotor2.set(-0.1);
        feedMotor1.set(-0.1);
        feedMotor2.set(-0.1);
    }

    public boolean isShooting() {
        return isShooting;
    }


}

