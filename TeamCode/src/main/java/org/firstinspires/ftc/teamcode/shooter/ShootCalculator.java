package org.firstinspires.ftc.teamcode.shooter;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;

@Configurable
public class ShootCalculator {
    //CONFIGURABLE PARAMETERS
    public double velocityMultiplier = 2.17;
    public int velocityCompensation = 280;
    //END CONFIGURABLE PARAMETERS

    double LLAngle = 26;//degrees
    double LLElevation = 2.2;//inches
    double shooterDiameter = 96; //mm
    double shootHeight = 29.5; //inches
    double distanceCompensation = 0.0;
    final int maxTPS = 28 * 4000; //max shooter speed in ticks per second
    double lastValidSpeed = -1000;
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
     * @param limelightData Limelight LLResult
     * @param positionCompensation additional distance compensation in inches based on shooter position
     * @return velocity in TPS
     */
    public double calculateRPMForTele(LLResult limelightData, double positionCompensation){
        if(limelightData != null && limelightData.isValid()){
            ty = limelightData.getTy();
            double velocity = getVelocityRaw(ty, positionCompensation);
            if(velocity < 0){
                return -1050; // safe speed
            }
            else{
                double baseTPS = velocityToTPS(velocity);
                return applyDistanceScaling(baseTPS, distanceToTarget);
            }

        } else{
            return -1050; // safe speed
        }
    }

    /**
     * Applies distance-based velocity scaling to prevent overshooting at close range
     * @param baseTPS base velocity in TPS from physics calculation
     * @param distance distance to target in inches
     * @return scaled velocity in TPS
     */
    private double applyDistanceScaling(double baseTPS, double distance) {
        // Close range scaling to prevent overshooting
        if(distance < 20) {
            // Very close: reduce to 40-60% of calculated velocity
            double scaleFactor = 0.4 + (distance / 20.0) * 0.2; // 0.4 at 0", 0.6 at 20"
            return Math.min(baseTPS * scaleFactor, 800); // hard cap at 800 TPS
        }
        else if(distance < 40) {
            // Medium close: reduce to 60-85% of calculated velocity
            double scaleFactor = 0.6 + ((distance - 20) / 20.0) * 0.25; // 0.6 at 20", 0.85 at 40"
            return baseTPS * scaleFactor;
        }
        else if(distance < 60) {
            // Transition range: 85-100% of calculated velocity
            double scaleFactor = 0.85 + ((distance - 40) / 20.0) * 0.15; // 0.85 at 40", 1.0 at 60"
            return baseTPS * scaleFactor;
        }
        else {
            // Long range: use full calculated velocity
            return baseTPS;
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
            double baseTPS = velocityToTPS(velocity);
            double scaledTPS = applyDistanceScaling(baseTPS, distanceToTarget);
            lastValidSpeed = scaledTPS;
            return scaledTPS;

        } else{
            double distanceToTarget = robotPose.distanceFrom(isRed ? RedGoalPose : BlueGoalPose);
            double velocity = ps.getVelocityInRPM(distanceToTarget + distanceCompensation,shooterDiameter);
            double baseTPS = velocityToTPS(velocity);
            double scaledTPS = applyDistanceScaling(baseTPS, distanceToTarget);
            lastValidSpeed = scaledTPS;
            return scaledTPS;
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
    public int getVelocityCompensation() {
        return velocityCompensation;
    }
    public void setVelocityCompensation(int velocityCompensation) {
        this.velocityCompensation = velocityCompensation;
    }
}
