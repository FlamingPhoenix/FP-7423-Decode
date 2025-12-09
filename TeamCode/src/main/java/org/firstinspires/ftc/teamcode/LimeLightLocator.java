package org.firstinspires.ftc.teamcode;
import com.qualcomm.hardware.limelightvision.LimeLight3A;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;

public class LimeLightLocator {
    LimeLight3A limelight = new LimeLight3A();
    double angle, elevation;
    double goalHeight = 29.5; // height of goal in inches
    public LimeLightLocator(hardwareMap hwmap, double angle, double elevation){ 
        this.angle = angle;
        this.elevation = elevation;

        limelight = hwmap.get(LimeLight3A.class, "limelight");
        limelight.setPollRateHz(100); // This sets how often we ask Limelight for data (100 times per second)
        limelight.start(); 
        limelight.pipelineSwitch(0);
    }
    public double getDistanceToTarget(){
        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            double ty = result.getTy(); // How far up or down the target is (degrees)

            double angleToGoal = angle + ty;
            double angleInRadians = Math.toRadians(angleToGoal);
            double distance = (goalHeight - elevation) / Math.tan(angleInRadians);
            return distance; // distance in inches

        } else {
            return -1; // Indicate that no target is found
        }
    }
    public double getDistanceToTargetStandalone(double ty, double angle, double elevation){
            double angleToGoal = angle + ty;
            double angleInRadians = Math.toRadians(angleToGoal);
            double distance = (goalHeight - elevation) / Math.tan(angleInRadians);
            return distance; // distance in inches
    }
}