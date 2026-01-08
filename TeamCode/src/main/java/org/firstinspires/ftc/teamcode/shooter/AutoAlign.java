package org.firstinspires.ftc.teamcode.shooter;

import org.firstinspires.ftc.teamcode.utility.FieldCentricDrive;
import org.firstinspires.ftc.teamcode.utility.FieldCentricDrivePinPoint;

public class AutoAlign{
    FieldCentricDrivePinPoint drive;

    public double kP;
    public double strafeMultiplier;

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
        double rotate = Math.abs(tx)>0.4?Math.max(Math.min(tx* kP + Math.copySign(0.2,tx), 1.0), -1.0):0; //negative because positive tx means target is to
        //double rotate = Math.abs(tx)>0.2 ? Math.copySign(kP,-tx):0;
//        double rotate = Math.abs(tx)>0.2 ? Math.tanh(0.5*tx)*kP : 0 ;
        //the right, so we need to turn left
        drive.drive(forward, strafe, rotate); //rotation is inverted in the actual drive method idk why
    }

    public void alignToTargetWithDiagonalApproach(double tx, double forward, double manualStrafe){
        //tx is angle to target (from limelight camera apriltag detection)
        double rotate = Math.abs(tx)>0.2 ? Math.copySign(kP*3,-tx):0;
        //the right, so we need to turn left

        // Calculate diagonal strafe to maintain 45-degree approach angle
        // Strafe in the same direction as rotation to maintain diagonal approach
        double diagonalStrafe = -tx * kP * strafeMultiplier; // configurable strafe multiplier for 45-degree approach
        diagonalStrafe = Math.max(Math.min(diagonalStrafe, 1.0), -1.0); // Clamp to [-1, 1]

        // Combine manual strafe input with diagonal strafe
        double totalStrafe = Math.max(Math.min(manualStrafe + diagonalStrafe, 1.0), -1.0);

        drive.drive(forward, totalStrafe, -rotate); //rotation is inverted in the actual drive method idk why
    }
}