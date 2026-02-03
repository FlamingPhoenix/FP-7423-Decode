package org.firstinspires.ftc.teamcode.shooter;

import org.firstinspires.ftc.teamcode.utility.FieldCentricDrive;
import org.firstinspires.ftc.teamcode.utility.FieldCentricDrivePinPoint;

public class AutoAlign{
    FieldCentricDrivePinPoint drive;

    public double kP;
    public double strafeMultiplier;
    public double targetOffset = 0.0;

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

    public void alignToTarget(double tx){
        //tx is angle to target (from limelight camera apriltag detection)
        double rotate = -tx * kP; //negative because positive tx means target is to
        //the right, so we need to turn left
        drive.drive(0, 0, rotate);
    }
    public void alignToTargetWithManualDrive(double tx, double forward, double strafe){
        //tx is angle to target (from limelight camera apriltag detection)
        double alignmentError = tx - targetOffset; // Target the offset instead of zero

        // Larger dead zone to prevent oscillation, and limit max rotation speed
        double rotate = 0;
        if (Math.abs(alignmentError) > 1.0) { // Increased from 0.3 to 1.0 degrees
            rotate = Math.max(Math.min(alignmentError * kP, 0.3), -0.3); // Limited max rotation to 0.3
        }

        drive.drive(forward, strafe, rotate);
    }

    public void setTargetOffset(double offset) {
        this.targetOffset = offset;
    }

}