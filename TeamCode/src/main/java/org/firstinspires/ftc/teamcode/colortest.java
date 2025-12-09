package org.firstinspires.ftc.teamcode;

import static java.lang.Math.abs;
import static java.lang.Math.min;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

@TeleOp
public class colortest extends OpMode
{

    int minidx;
    NormalizedColorSensor col1, col2;
    private float[][] hsvValues = new float[2][3];
    final private int[] PGN = {240,160,34};
    final private String[] PGN_NAMES = {"Purple","Green","OTHER"};
    @Override
    public void init() {
        col1 = hardwareMap.get(NormalizedColorSensor.class, "col2");
//        col2 = hardwareMap.get(NormalizedColorSensor.class, "col2");
        col1.setGain(2.4f);
    }
    @Override
    public void loop() {

        NormalizedRGBA color1 = col1.getNormalizedColors();
//        NormalizedRGBA color2 = col2.getNormalizedColors();

        Color.colorToHSV(color1.toColor(),hsvValues[0]);
//        Color.colorToHSV(color2.toColor(),hsvValues[1]);
        double[] distances = {
                circularDistance(hsvValues[0][0], PGN[0]),
                circularDistance(hsvValues[0][0], PGN[1]),
                circularDistance(hsvValues[0][0], PGN[2])
        };

        minidx = 2;
        for (int j = 0; j < 3; j++) {
            if (distances[j] < distances[minidx]) {
                minidx = j;
            }
        }
        telemetry.addData("Hue", hsvValues[0][0]);
        telemetry.addData("sensed color", PGN_NAMES[minidx]);
        telemetry.update();

    }
    private double circularDistance(double a, double b){
        double diff = abs(a-b);
        return min(diff,360-diff);
    }
}
