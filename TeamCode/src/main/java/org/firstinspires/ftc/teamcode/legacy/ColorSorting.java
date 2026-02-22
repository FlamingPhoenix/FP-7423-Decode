package org.firstinspires.ftc.teamcode.legacy;

import static java.lang.Math.abs;
import static java.lang.Math.min;

import android.graphics.Color;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

public class ColorSorting {
    HardwareMap hwmap;
    NormalizedColorSensor col1,col2,col3;
    private float[][] hsvValues = new float[3][3];
    private int[] colorIDs = new int[3];

    final private int[] PGN = {260,168,34};
    public ColorSorting(HardwareMap hwmap) {
        this.hwmap = hwmap;
        col1 = hwmap.get(NormalizedColorSensor.class, "col1");
        col2 = hwmap.get(NormalizedColorSensor.class, "col2");
        col3 = hwmap.get(NormalizedColorSensor.class, "col3");
    }
    public void updateColors(){
        NormalizedRGBA color1 = col1.getNormalizedColors();
        NormalizedRGBA color2 = col2.getNormalizedColors();
        NormalizedRGBA color3 = col3.getNormalizedColors();
        Color.colorToHSV(color1.toColor(),hsvValues[0]);
        Color.colorToHSV(color2.toColor(),hsvValues[1]);
        Color.colorToHSV(color3.toColor(),hsvValues[2]);
        for (int i = 0; i < 3; i++) {
            double[] distances = {
                    circularDistance(hsvValues[i][0], PGN[0]),
                    circularDistance(hsvValues[i][0], PGN[1]),
                    circularDistance(hsvValues[i][0], PGN[2])
            };
            int minIdx = 0;
            for (int j = 1; j < 3; j++) {
                if (distances[j] < distances[minIdx]) {
                    minIdx = j;
                }
            }
            colorIDs[i] = minIdx;
        }
    }
    public int whereIs(int colorID){
        for (int i = 0; i < 3; i++) {
            if(colorIDs[i] == colorID) return i;
        }
        return -1;
    }
    public int whereIsClosestTo(int ColorID, int pos){
        int closest = -1;
        double closestDist = Double.MAX_VALUE;
        for (int i = 0; i < 3; i++) {
            if(colorIDs[i] == ColorID){
                double dist = abs(i - pos);
                if(dist < closestDist){
                    closestDist = dist;
                    closest = i;
                }
            }
        }
        return closest;
    }
    private double circularDistance(double a, double b){
        double diff = abs(a-b);
        return min(diff,360-diff);
    }
}
