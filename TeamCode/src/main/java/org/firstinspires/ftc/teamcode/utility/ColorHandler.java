package org.firstinspires.ftc.teamcode.utility;

public interface ColorHandler {
    BallOrder detectBallOrder();
    BallOrder detectRawBallOrder(); //returns the ball order based on the first sensor that detects a color, without trying to resolve conflicts. Useful for debugging sensor issues.
    float[][] getHSVValues(); //returns the raw HSV values from the sensors, useful for debugging and tuning
}