package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.utility.BallOrder;
import org.firstinspires.ftc.teamcode.utility.ColorHandler;
import org.firstinspires.ftc.teamcode.utility.DualColorHandler;
import org.firstinspires.ftc.teamcode.utility.LEDHandler;

import java.util.Arrays;

@TeleOp
public class ColorsTest extends OpMode {
    ColorHandler cols;
    LEDHandler ledHandler;
    float[][] hsvvalues = new float[6][3];
    @Override
    public void init(){
        cols = new DualColorHandler(hardwareMap);
        ledHandler = new LEDHandler(hardwareMap);
    }

    @Override
    public void loop(){
        BallOrder ballOrder = cols.detectRawBallOrder();
        hsvvalues = cols.getHSVValues();
        ledHandler.ballColors(ballOrder);
        ledHandler.setColorsFromStatic();
        //
        telemetry.addData("back",ballOrder.back);
        telemetry.addData("middle",ballOrder.middle);
        telemetry.addData("front",ballOrder.front);
        //print raw
        telemetry.addData("back1", Arrays.toString(hsvvalues[0]));
        telemetry.addData("back2", Arrays.toString(hsvvalues[1]));
        telemetry.addData("middle1", Arrays.toString(hsvvalues[2]));
        telemetry.addData("middle2", Arrays.toString(hsvvalues[3]));
        telemetry.addData("front1", Arrays.toString(hsvvalues[4]));
        telemetry.addData("front2", Arrays.toString(hsvvalues[5]));

        telemetry.update();
    }
}