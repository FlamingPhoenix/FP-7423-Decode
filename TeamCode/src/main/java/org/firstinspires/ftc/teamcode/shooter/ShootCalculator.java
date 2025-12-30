package org.firstinspires.ftc.teamcode.shooter;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;

@Configurable
public class ShootCalculator {
    //CONFIGURABLE PARAMETERS
    public static double velocityMultiplier = 2.17;
    public static double velocityCompensation = 280;
    //END CONFIGURABLE PARAMETERS

    double LLAngle = 26;//degrees
    double LLElevation = 2.2;//inches
    double shooterDiameter = 96; //mm
    double shootHeight = 29.5; //inches
    double distanceCompensation = 0.0;
    final int maxTPS = 28 * 4000; //max shooter speed in ticks per second
    double lastValidSpeed = -1200;
    double ty;
    public double distanceToTarget;
    Pose RedGoalPose = new Pose(144, 144);
    Pose BlueGoalPose = new Pose(0, 144);
    PerfectShooting ps;
    public ShootCalculator(double LLAngle, double LLElevation, double shooterDiameter){
        this.LLAngle = LLAngle;
        this.LLElevation = LLElevation;
        this.shooterDiameter = shooterDiameter;
        this.ps = new PerfectShooting(shootHeight);
    }
    public ShootCalculator(){
        this.ps = new PerfectShooting(shootHeight);
    }

    public double getVelocityRaw(double ty, double distanceCompensation){
        distanceToTarget = LimeLightLocator.getDistanceToTargetStandalone(ty, LLAngle, LLElevation);
        return ps.getVelocityInRPM(distanceToTarget + distanceCompensation,shooterDiameter);
    }
    public double velocityToTPS(double velocity){
        return Math.min(
                velocityMultiplier *
                        (
                                -28 * velocity
                        )
                        - velocityCompensation
                , maxTPS);
    }

    //USE FUNCTIONS
    /**
     * Calculates the RPM needed to shoot at the target given the LLResult from the limelight
     * Safety fallback RPM is -1200
     * @param result Limelight LLResult
     * @param distanceCompensation additional distance compensation in inches based on shooter position
     * @return velocity in TPS
     */
    public double calculateRPMWForTELE(LLResult result, double distanceCompensation){
        if(result != null && result.isValid()){
            ty = result.getTy();
            double velocity = getVelocityRaw(ty, distanceCompensation);
            if(velocity < 0){
                return -1200; // safe speed
            }
            else{
                return velocityToTPS(velocity);
            }

        } else{
            return -1200; // safe speed
        }
    }

    /**
     * Calculates the RPM needed to shoot at the target given the LLResult from the limelight
     * FOR USE IN AUTONOMOUS - FALLS BACK TO POSE ESTIMATION
     *
     * @param result
     * @param distanceCompensation
     * @param robotPose
     * @param isRed
     * @return
     */
    public double calculateRPMWithSafety(LLResult result, double distanceCompensation, Pose robotPose, boolean isRed){
        if(result != null && result.isValid()){
            ty = result.getTy();
            double velocity = getVelocityRaw(ty, distanceCompensation);
            double tps = velocityToTPS(velocity);
            lastValidSpeed = tps;
            return tps;

        } else{
            double distanceToTarget = robotPose.distanceFrom(isRed ? RedGoalPose : BlueGoalPose);
            double velocity = ps.getVelocityInRPM(distanceToTarget + distanceCompensation,shooterDiameter);
            double tps = velocityToTPS(velocity);
            lastValidSpeed = tps;
            return tps;
        }
    }






    //getters and setters
    public double getLLAngle() {
        return LLAngle;
    }
    public void setLLAngle(double LLAngle) {
        this.LLAngle = LLAngle;
    }
    public double getLLElevation() {
        return LLElevation;
    }
    public void setLLElevation(double LLElevation) {
        this.LLElevation = LLElevation;
    }
    public double getShooterDiameter() {
        return shooterDiameter;
    }
    public void setShooterDiameter(double shooterDiameter) {
        this.shooterDiameter = shooterDiameter;
    }
    public double getVelocityMultiplier() {
        return velocityMultiplier;
    }
    public void setVelocityMultiplier(double velocityMultiplier) {
        this.velocityMultiplier = velocityMultiplier;
    }
    public double getVelocityCompensation() {
        return velocityCompensation;
    }
    public void setVelocityCompensation(double velocityCompensation) {
        this.velocityCompensation = velocityCompensation;
    }
}
