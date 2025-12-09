package org.firstinspires.ftc.teamcode;

import java.lang.reflect.Field;

import org.firstinspires.ftc.teamcode.utility.FieldCentricDrive;

public class AutoAlign{
    FieldCentricDrive drive;
    public AutoAlign(FieldCentricDrive drive){
        this.drive = drive;
    }
    public void alignToTarget(double tx){
        //tx is angle to target (from limelight camera apriltag detection)
        double kP = 0.01; //proportional constant for turning
        double rotate = -tx * kP; //negative because positive tx means target is to
        //the right, so we need to turn left
        drive.drive(0, 0, rotate);
    }
    public void alignToTargetWithManualDrive(double tx, double forward, double strafe){
        //tx is angle to target (from limelight camera apriltag detection)
        double kP = 0.01; //proportional constant for turning
        double rotate = Math.max(Math.min(-tx * kP, 1.0), -1.0); //negative because positive tx means target is to
        //the right, so we need to turn left
        drive.drive(forward, strafe, -rotate); //rotation is inverted in the actual drive method idk why
    }
}