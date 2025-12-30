package org.firstinspires.ftc.teamcode.shooter;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class LimeLightLocator {
    Limelight3A limelight;
    double angle, elevation;
    static double goalHeight = 29.5+11.811; // height of goal in inches
    public LimeLightLocator(HardwareMap hwmap, double angle, double elevation){
        this.angle = angle;
        this.elevation = elevation;

        limelight = hwmap.get(Limelight3A.class, "limelight");
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
            return (goalHeight - elevation) / Math.tan(angleInRadians); // distance in inches

        } else {
            return -1; // Indicate that no target is found
        }
    }
    public static double getDistanceToTargetStandalone(double ty, double angle, double elevation){
            double angleToGoal = angle + ty;
            double angleInRadians = Math.toRadians(angleToGoal);
        return (goalHeight - elevation) / Math.tan(angleInRadians); // distance in inches
    }
}