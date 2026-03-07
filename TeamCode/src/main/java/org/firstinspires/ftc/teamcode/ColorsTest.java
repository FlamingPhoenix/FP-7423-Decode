package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.utility.BallOrder;
import org.firstinspires.ftc.teamcode.utility.ColorHandler;
import org.firstinspires.ftc.teamcode.utility.DualColorHandler;
import org.firstinspires.ftc.teamcode.utility.LEDHandler;

@TeleOp
public class ColorsTest extends OpMode {
    ColorHandler cols;
    LEDHandler ledHandler;
    @Override
    public void init(){
        cols = new DualColorHandler(hardwareMap);
        ledHandler = new LEDHandler(hardwareMap);
    }

    @Override
    public void loop(){
        BallOrder ballOrder = cols.detectRawBallOrder();
        ledHandler.ballColors(ballOrder);
        ledHandler.setColorsFromStatic();
        telemetry.addData("back",ballOrder.back);
        telemetry.addData("middle",ballOrder.middle);
        telemetry.addData("front",ballOrder.front);
    }
}
