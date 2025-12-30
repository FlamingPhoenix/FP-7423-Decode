package org.firstinspires.ftc.teamcode.shooter;

public class PerfectShooting {

    /*
    TODO for robot
    * measure the height of the shooter from the ground in inches
    * measure height of GOAL from ground in inches
    * Add Limelight to measure distance to goal in inches using AprilTags
    * tune motor PID (DcMotorEx) for more accurate RPM control
    * 
    
    */
    double angle  = Math.toRadians(60);
    double height;
    final double cos2a = Math.pow(Math.cos(angle), 2);
    final double tana = Math.tan(angle);
    /**
     * Constructor
     * @param height in inches
     */
    public PerfectShooting(double height) {
        this.height = height/39.37; //convert height to meters
    }
    /*
     * Calculates the required velocity to shoot a projectile to a given distance in meters per second
     * @param distance in inches
     * @return
     */
    public double getVelocity(double distance) {
        double distanceInMeters = distance / 39.37;
        double denom = cos2a * (distanceInMeters * tana - height);
        if(denom <= 0){
            return -1;
        }
        return Math.sqrt( 4.9 * Math.pow(distanceInMeters, 2) / denom);
    }
    /**
     * Converts velocity in meters per second to RPM given the diameter of the wheel
     * @param velocity in meters per second
     * @param diameter in mm
     * @return
     */
    public double velocityToRPM(double velocity, double diameter){
        if(velocity>0){

            double diameterInMeters = diameter / 1000;
            double circumference = Math.PI * diameterInMeters;
            double rps = velocity / circumference;
            return rps;
        }
        else{
            return -1;
        }
    }
    /**
     * combined method to get RPM from distance and wheel diameter
     * @param distance in inches
     * @param diameter in mm
     * @return
     */
    public double getVelocityInRPM(double distance, double diameter){
        double velocity = getVelocity(distance);
        return velocityToRPM(velocity, diameter);
    }

}