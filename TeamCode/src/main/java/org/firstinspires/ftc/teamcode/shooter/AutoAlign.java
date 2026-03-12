package org.firstinspires.ftc.teamcode.shooter;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.utility.FieldCentricDrive;
import org.firstinspires.ftc.teamcode.utility.FieldCentricDrivePinPoint;

public class AutoAlign{
    FieldCentricDrivePinPoint drive;

    public double kP, kD;
    public double strafeMultiplier;
    public double targetOffset = 0.0;
    private double lastError = 0.0;
    private ElapsedTime timer = new ElapsedTime();

    public AutoAlign(FieldCentricDrivePinPoint drive, double kP){
        this.drive = drive;
        this.kP = kP;
        this.strafeMultiplier = 0.7; // default value
    }

    public AutoAlign(FieldCentricDrivePinPoint drive, double kP, double strafeMultiplier){
        this.drive = drive;
        this.kP = kP;
        this.strafeMultiplier = strafeMultiplier;
    }
    public AutoAlign(FieldCentricDrivePinPoint drive, double kP,double strafeMultiplier, double kD){
        this.drive = drive;
        this.kP = kP;
        this.kD = kD;
        this.strafeMultiplier = strafeMultiplier;
    }

    public void alignToTarget(double tx){
        //tx is angle to target (from limelight camera apriltag detection)
        double rotate = -tx * kP; //negative because positive tx means target is to
        //the right, so we need to turn left
        drive.drive(0, 0, rotate);
    }
    public void alignToTargetWithManualDrive(double tx, double forward, double strafe){
        double dt = timer.seconds();
        timer.reset();
        //tx is angle to target (from limelight camera apriltag detection)
        double alignmentError = tx - targetOffset; // Target the offset instead of zero
        double derivative = 0;
        if (dt > 0 && dt < 1.0) { // Also check dt isn't too large
            derivative = (alignmentError - lastError) / dt;
        }
        lastError = alignmentError;
        // Larger dead zone to prevent oscillation, and limit max rotation speed
        double rotate = 0;
        if (Math.abs(alignmentError) > 1.0) { // Increased from 0.3 to 1.0 degrees
            rotate = Math.max(Math.min(alignmentError * kP + kD*derivative, 0.3), -0.3); // Limited max rotation to 0.3
        }

        drive.drive(strafe, forward, rotate);
    }

    public void setTargetOffset(double offset) {
        this.targetOffset = offset;
    }

}