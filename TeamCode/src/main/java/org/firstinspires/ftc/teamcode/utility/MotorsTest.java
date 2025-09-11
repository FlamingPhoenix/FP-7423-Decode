package org.firstinspires.ftc.teamcode.utility;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

public class MotorsTest extends OpMode {
    DcMotor fr,fl,br,bl;
    @Override
    public void init() {
        fr = hardwareMap.dcMotor.get("fr");
        fl = hardwareMap.dcMotor.get("fl");
        br = hardwareMap.dcMotor.get("br");
        bl = hardwareMap.dcMotor.get("bl");
        fl.setDirection(DcMotor.Direction.REVERSE);
        bl.setDirection(DcMotor.Direction.REVERSE);
    }

    @Override
    public void loop() {
        fr.setPower(gamepad1.right_trigger/2);
        fl.setPower(gamepad1.left_trigger/2);
        br.setPower(gamepad1.right_stick_y/2);
        bl.setPower(gamepad1.left_stick_y/2);
    }
}
