package org.firstinspires.ftc.teamcode.utility;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class SingleColorHandler implements ColorHandler {
    ColorSensor backc, middlec, frontc;
    public SingleColorHandler(HardwareMap hardwareMap) {
        backc = hardwareMap.get(ColorSensor.class, "backc1");
        middlec = hardwareMap.get(ColorSensor.class, "middlec1");
        frontc = hardwareMap.get(ColorSensor.class, "frontc1");
    }
    @Override
    public BallOrder detectBallOrder() {
        BallColor backColor = detectBallColor(backc);
        BallColor middleColor = detectBallColor(middlec);
        BallColor frontColor = detectBallColor(frontc);

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
