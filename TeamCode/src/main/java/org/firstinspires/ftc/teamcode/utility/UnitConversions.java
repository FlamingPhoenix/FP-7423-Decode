package org.firstinspires.ftc.teamcode.utility;

public class UnitConversions {
    public static final double MM_TO_INCH = 0.0393701;
    public static final double INCH_TO_MM = 25.4;
    public static final double MM_TO_M = 0.001;
    public static final double M_TO_MM = 1000;
    public static final double M_TO_INCH = 39.3701;
    public static final double INCH_TO_M = 0.0254;
    public static final double M_TO_FEET = 3.28084;
    public static final double FEET_TO_M = 0.3048;
    public static final double MM_TO_FEET = 0.00328084;
    public static final double FEET_TO_MM = 304.8;
    public static final double INCH_TO_FEET = 0.0833333;
    public static final double FEET_TO_INCH = 12;
    public static final double RAD_TO_DEG = 57.2958;
    public static final double DEG_TO_RAD = 0.0174533;
    public static double inToMM(double in){
        return in*INCH_TO_MM;
    }
    public static double mmToIn(double mm){
        return mm*MM_TO_INCH;
    }
}
