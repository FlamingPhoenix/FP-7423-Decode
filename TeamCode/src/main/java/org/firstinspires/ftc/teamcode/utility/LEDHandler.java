package org.firstinspires.ftc.teamcode.utility;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PWMOutput;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.Queue;

public class LEDHandler {
    Servo led1, led2, led3;
    public static final double LED_PURPLE = 0.721;
    public static final double LED_GREEN = 0.42;
    public static final double LED_ERROR = 0.278;
    public static final double LED_OFF = 0.0;
    public static final double LED_WHITE = 1.0;
    public static final double LED_BLUE = 0.666;


    public static double led1Color;
    public static double led2Color;
    public static double led3Color;
    public LEDHandler(HardwareMap hardwareMap){
        led1 = hardwareMap.get(Servo.class, "led1");
        led2 = hardwareMap.get(Servo.class, "led2");
        led3 = hardwareMap.get(Servo.class, "led3");
        setALLColor(LED_WHITE);
        setColorsFromStatic();
    }
    public void setALLColor(double color) {
        led1Color = color;
        led2Color = color;
        led3Color = color;
    }
    public void ballColors(BallOrder ballorder){
        switch (ballorder.front){
            case GREEN:
                led1Color = LED_GREEN;
                break;
            case PURPLE:
                led1Color = LED_PURPLE;
                break;
            case UNKNOWN:
                led1Color = LED_WHITE;
                break;
        }
        switch (ballorder.middle){
            case GREEN:
                led2Color = LED_GREEN;
                break;
            case PURPLE:
                led2Color = LED_PURPLE;
                break;
            case UNKNOWN:
                led2Color = LED_WHITE;
                break;
        }
        switch (ballorder.back) {
            case GREEN:
                led3Color = LED_GREEN;
                break;
            case PURPLE:
                led3Color = LED_PURPLE;
                break;
            case UNKNOWN:
                led3Color = LED_WHITE;
                break;
        }
    }

    /**
     * CALL THIS EVERY LOOP
     */
    public void setColorsFromStatic() {
        setLed1(led1Color);
        setLed2(led2Color);
        setLed3(led3Color);
    }
    public void updateStaticColors() {
        led1Color = led1.getPosition();
        led2Color = led2.getPosition();
        led3Color = led3.getPosition();
    }




    public void setLed1(double color){
        led1.setPosition(color);
    }
    public void setLed2(double color) {
        led2.setPosition(color);
    }
    public void setLed3(double color) {
        led3.setPosition(color);
    }


}
