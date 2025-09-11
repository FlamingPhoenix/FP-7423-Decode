package org.firstinspires.ftc.teamcode;


import static java.lang.Math.abs;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.utility.FieldCentricDrive;

@TeleOp
public class TeleOpMain extends OpMode{
    FieldCentricDrive drive;
    int exp = 1;
    @Override
    public void init() {
        drive = new FieldCentricDrive(hardwareMap);
    }
    @Override
    public void loop() {
        if(gamepad1.x){
            drive.resetIMU();
        }
        drive.drive(gamepad1, exp);
    }
}