package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous
public class BlankAuto extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        DcMotor fl, fr, bl, br;
        fl = hardwareMap.dcMotor.get("fl");
        fr = hardwareMap.dcMotor.get("fr");
        bl = hardwareMap.dcMotor.get("bl");
        br = hardwareMap.dcMotor.get("br");
        ElapsedTime timer = new ElapsedTime();
        fr.setDirection(DcMotor.Direction.REVERSE);
        br.setDirection(DcMotor.Direction.REVERSE);
        waitForStart();
        timer.reset();
        while (opModeIsActive()) {
            if(timer.milliseconds() < 2000) {
                fl.setPower(0.5);
                fr.setPower(0.5);
                bl.setPower(0.5);
                br.setPower(0.5);
            }
            else{
                fr.setPower(0);
                fl.setPower(0);
                bl.setPower(0);
                br.setPower(0);

            }

        }
    }
}
