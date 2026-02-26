package org.firstinspires.ftc.teamcode.utility;
import javax.swing.colorchooser.ColorSelectionModel;

import com.qualcomm.robotcore.hardware.ColorSensor;

public class DualColorHandler implements ColorHandler {
    ColorSensor backc1, backc2, middlec1, middlec2, frontc1, frontc2;
    
    public DualColorHandler(HardwareMap hardwareMap) {
        ColorSensor backc1 = hardwareMap.get(ColorSensor.class, "backc1");
        ColorSensor backc2 = hardwareMap.get(ColorSensor.class, "backc2");
        ColorSensor middlec1 = hardwareMap.get(ColorSensor.class, "middlec1");
        ColorSensor middlec2 = hardwareMap.get(ColorSensor.class, "middlec2");
        ColorSensor frontc1 = hardwareMap.get(ColorSensor.class, "frontc1");
        ColorSensor frontc2 = hardwareMap.get(ColorSensor.class, "frontc2");

    }

    @Override
    public BallOrder detectBallOrder() {
        BallColor backColor1 = detectBallColor(backc1);
        BallColor backColor2 = detectBallColor(backc2);
        BallColor middleColor1 = detectBallColor(middlec1);
        BallColor middleColor2 = detectBallColor(middlec2);
        BallColor frontColor1 = detectBallColor(frontc1);
        BallColor frontColor2 = detectBallColor(frontc2);
        // take whichever color is not UNKNOWN for each position
        BallColor backColor = (backColor1 != BallColor.UNKNOWN) ? backColor1 : backColor2;
        BallColor middleColor = (middleColor1 != BallColor.UNKNOWN) ? middleColor1 : middleColor2;
        BallColor frontColor = (frontColor1 != BallColor.UNKNOWN) ? frontColor1 : frontColor2;

        return new BallOrder(backColor, middleColor, frontColor);
    }

    private BallColor detectBallColor(ColorSensor sensor) {
        int red = sensor.red();
        int green = sensor.green();
        int blue = sensor.blue();

        // Green ball detection: high green, lower red and blue
        if (green > red && green > blue && green > 100) {
            return BallColor.GREEN;
        }
        // Purple ball detection: high red and blue, lower green
        else if ((red + blue) > (2 * green) && red > 80 && blue > 80) {
            return BallColor.PURPLE;
        }

        return BallColor.UNKNOWN;
    }
}