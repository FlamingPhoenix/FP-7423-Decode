package org.firstinspires.ftc.teamcode;


import static java.lang.Math.abs;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.utility.FieldCentricDrive;

@TeleOp
public class TeleOpMain extends OpMode{
    FieldCentricDrive drive;
    DcMotor shooter;
    DcMotor intake;

    Servo front, back, middle, linkage;
    CRServo wheel;
    boolean inShoot = false;
    int shootSequenceState = 0;
    ElapsedTime shootSequenceTimer = new ElapsedTime();
    int shootMode = 0; // 0 = all balls, 1 = back only, 2 = middle only, 3 = front only
    int exp = 1;
    @Override
    public void init() {
        drive = new FieldCentricDrive(hardwareMap);
        shooter = hardwareMap.get(DcMotor.class, "shooter");
        intake = hardwareMap.get(DcMotor.class, "intake");
        front = hardwareMap.servo.get("front");
        back = hardwareMap.servo.get("back");
        middle = hardwareMap.servo.get("middle");
        linkage = hardwareMap.servo.get("linkage");
        wheel = hardwareMap.crservo.get("wheel");
    }
    @Override
    public void loop() {
        if(gamepad1.x){
            drive.resetIMU();
        }
        drive.drive(gamepad1, exp);

        // Add telemetry for debugging drive issues
        telemetry.addData("IMU Heading (deg)", Math.toDegrees(drive.getHeading()));
        telemetry.addData("Left Stick X", gamepad1.left_stick_x);
        telemetry.addData("Left Stick Y", gamepad1.left_stick_y);
        telemetry.addData("Right Stick X", gamepad1.right_stick_x);

        // Individual ball shooting controls
        if(gamepad1.dpad_left && !inShoot) { // Shoot back ball only
            inShoot = true;
            shootMode = 1;
            shootSequenceState = 1;
            shootSequenceTimer.reset();
        } else if(gamepad1.dpad_up && !inShoot) { // Shoot middle ball only
            inShoot = true;
            shootMode = 2;
            shootSequenceState = 3;
            shootSequenceTimer.reset();
        } else if(gamepad1.dpad_right && !inShoot) { // Shoot front ball only
            inShoot = true;
            shootMode = 3;
            shootSequenceState = 5;
            shootSequenceTimer.reset();
        } else if(gamepad1.b && !inShoot) { // Shoot all balls (original sequence)
            inShoot = true;
            shootMode = 0;
            shootSequenceState = 1;
            shootSequenceTimer.reset();
        }

        // Shooter controls (only when not in sequence)
        if(!inShoot) {
            if(gamepad1.right_bumper) {
                shooter.setPower(-0.65);
            } else {
                shooter.setPower(0);
            }
        }

        // Intake controls
        if(gamepad1.left_trigger > 0.1) {
            intake.setPower(gamepad1.left_trigger);
            wheel.setPower(1.0);
        } else if(gamepad1.left_bumper) {
            intake.setPower(-1.0);
            wheel.setPower(-1.0);
        } else {
            intake.setPower(0);
            wheel.setPower(0);
        }

        if(inShoot){
            switch(shootSequenceState) {
                case 1: // Move linkage to back position and lift back ball
                    linkage.setPosition(0.3567);  // Move shooter to back position
                    shooter.setPower(-0.65);
                    if(shootSequenceTimer.milliseconds() > 700) { // Wait for linkage to move
                        back.setPosition(0.6); // Push back ball up
                        shootSequenceTimer.reset();
                        shootSequenceState = 2;
                    }
                    break;

                case 2: // Wait then reset back servo and move to middle
                    if(shootSequenceTimer.milliseconds() > 500) { // Wait for ball to shoot
                        back.setPosition(0.113); // Reset back servo
                        if(shootMode == 1) { // Back ball only
                            shooter.setPower(0);
                            inShoot = false;
                            shootSequenceState = 0;
                        } else { // Continue to middle ball for full sequence
                            linkage.setPosition(0.25); // Move shooter to middle position
                            shootSequenceTimer.reset();
                            shootSequenceState = 3;
                        }
                    }
                    break;

                case 3: // Move linkage to middle position and lift middle ball
                    if(shootMode == 2) { // Middle ball only - need to position linkage first
                        linkage.setPosition(0.25); // Move shooter to middle position
                        shooter.setPower(-0.65);
                    }
                    if(shootSequenceTimer.milliseconds() > 300) { // Wait for linkage to move
                        middle.setPosition(1.0); // Push middle ball up
                        shootSequenceTimer.reset();
                        shootSequenceState = 4;
                    }
                    break;

                case 4: // Wait then reset middle servo and move to front
                    if(shootSequenceTimer.milliseconds() > 500) { // Wait for ball to shoot
                        middle.setPosition(0.5322); // Reset middle servo
                        if(shootMode == 2) { // Middle ball only
                            shooter.setPower(0);
                            inShoot = false;
                            shootSequenceState = 0;
                        } else { // Continue to front ball for full sequence
                            linkage.setPosition(0.0); // Move shooter to front position
                            shootSequenceTimer.reset();
                            shootSequenceState = 5;
                        }
                    }
                    break;

                case 5: // Move linkage to front position and lift front ball
                    if(shootMode == 3) { // Front ball only - need to position linkage first
                        linkage.setPosition(0.0); // Move shooter to front position
                        shooter.setPower(-0.65);
                    }
                    if(shootSequenceTimer.milliseconds() > 300) { // Wait for linkage to move
                        front.setPosition(1.0); // Push front ball up
                        shootSequenceTimer.reset();
                        shootSequenceState = 6;
                    }
                    break;

                case 6: // End sequence
                    if(shootSequenceTimer.milliseconds() > 500) { // Wait for ball to shoot
                        front.setPosition(0.5322); // Reset front servo
                        shooter.setPower(0);
                        inShoot = false;
                        shootSequenceState = 0;
                    }
                    break;
            }
        }
    }
}