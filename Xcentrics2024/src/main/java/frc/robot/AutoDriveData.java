package frc.robot;

public class AutoDriveData {
    public enum PLANE {
        X, Y, NONE
    }

    public PLANE plane;
    public double targetDistance;

    public AutoDriveData(PLANE plane, double targetDistance)
    {
        this.plane = plane;
        this.targetDistance = targetDistance;
    }
}
