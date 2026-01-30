package org.firstinspires.ftc.teamcode.utility;

import static java.lang.Math.abs;
import static java.lang.Math.min;

import android.graphics.Color;

import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

public class ColorIdentification {
    NormalizedColorSensor colorSensor;
    public ColorIdentification(NormalizedColorSensor col){
        colorSensor = col;
        colorSensor.setGain(2.4f);
    }
    int minidx;
    private float[] hsvValues = new float[3];
    final private int[] PGN = {240,160,34};
    final private String[] PGN_NAMES = {"Purple","Green","OTHER"};

    public ARTIFACT_COLOR getCurrentColor(){
        NormalizedRGBA color = colorSensor.getNormalizedColors();
        Color.colorToHSV(color.toColor(),hsvValues);
        double[] distances = {
                circularDistance(hsvValues[0], PGN[0]),
                circularDistance(hsvValues[0], PGN[1]),
                circularDistance(hsvValues[0], PGN[2])
        };

        minidx = 2;
        for (int j = 0; j < 3; j++) {
            if (distances[j] < distances[minidx]) {
                minidx = j;
            }
        }

        return minidx == 0 ? ARTIFACT_COLOR.PURPLE : (minidx == 1 ? ARTIFACT_COLOR.GREEN : ARTIFACT_COLOR.OTHER);

    }
    public boolean artifactPresent(){
        ARTIFACT_COLOR currentColor = getCurrentColor();
        return !(currentColor == ARTIFACT_COLOR.OTHER);
    }


    private double circularDistance(double a, double b){
        double diff = abs(a-b);
        return min(diff,360-diff);
    }


}
