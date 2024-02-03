package frc.robot.Util;

// Shot parameter
public class ShotParameter {

    // Variables
    public final double angle;
    public final double rpm;

    // Constructor
    public ShotParameter(double angle, double rpm) {
        this.angle = angle;
        this.rpm = rpm;
    }   

    // Method equals
    public boolean equals(ShotParameter other) {
        return Math.abs(this.angle - other.angle) < 0.1 &&
        Math.abs(this.rpm - other.rpm) < 0.1;
    }

    // Method to interpolate
    public ShotParameter interpolate(ShotParameter end, double t) {
        return new ShotParameter(
            lerp(angle, end.angle, t), 
            lerp(rpm, end.rpm, t)
        );
    }

    // Method lerp
    private double lerp(double y2, double y1, double t) {
        return y1 + (t * (y2 - y1));
    }
 
}