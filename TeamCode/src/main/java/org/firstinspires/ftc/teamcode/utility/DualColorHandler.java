package org.firstinspires.ftc.teamcode.utility;
//import javax.swing.colorchooser.ColorSelectionModel;

import static java.lang.Math.abs;
import static java.lang.Math.min;

import android.graphics.Color;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.ArrayDeque;
import java.util.Deque;
@Configurable
public class DualColorHandler implements ColorHandler {
    public static double satThreshold = 0.2;
    public static double gain = 1.0;
    private static final long CONFIDENCE_WINDOW_MS = 200;

    private final ColorSensor backc1;
    private final ColorSensor backc2;
    private final ColorSensor middlec1;
    private final ColorSensor middlec2;
    private final ColorSensor frontc1;
    private final ColorSensor frontc2;
    private final MajorityWindow majorityWindow;

    final private int[] PGN = {260,168,34}; //hue values for purple, green, and orange (furthest hue from either)

    public DualColorHandler(HardwareMap hardwareMap) {
        this.backc1 = hardwareMap.get(ColorSensor.class, "backc1");
        this.backc2 = hardwareMap.get(ColorSensor.class, "backc2");
        this.middlec1 = hardwareMap.get(ColorSensor.class, "middlec1");
        this.middlec2 = hardwareMap.get(ColorSensor.class, "middlec2");
        this.frontc1 = hardwareMap.get(ColorSensor.class, "frontc1");
        this.frontc2 = hardwareMap.get(ColorSensor.class, "frontc2");
        this.backc1.setGain(gain);
        this.backc2.setGain(gain);
        this.middlec1.setGain(gain);
        this.middlec2.setGain(gain);
        this.frontc1.setGain(gain);
        this.frontc2.setGain(gain);
        
        majorityWindow = new MajorityWindow(CONFIDENCE_WINDOW_MS);
    }

    @Override
    public BallOrder detectBallOrder() {
        BallOrder rawOrder = detectRawBallOrder();
        return majorityWindow.update(rawOrder);
    }

    @Override
    public BallOrder detectRawBallOrder() {
//        BallColor backColor1 = detectBallColor(backc1);
//        BallColor backColor2 = detectBallColor(backc2);
//        BallColor middleColor1 = detectBallColor(middlec1);
//        BallColor middleColor2 = detectBallColor(middlec2);
//        BallColor frontColor1 = detectBallColor(frontc1);
//        BallColor frontColor2 = detectBallColor(frontc2);
        BallColor backColor1 = detectBallColor2(backc1);
        BallColor backColor2 = detectBallColor2(backc2);
        BallColor middleColor1 = detectBallColor2(middlec1);
        BallColor middleColor2 = detectBallColor2(middlec2);
        BallColor frontColor1 = detectBallColor2(frontc1);
        BallColor frontColor2 = detectBallColor2(frontc2);

        // Pick the first sensor that produces a known color for each position.
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

    private BallColor detectBallColor2(ColorSensor sensor){
        NormalizedRGBA rgba = ((NormalizedColorSensor) sensor).getNormalizedColors();
        float[] hsvvalues =  new float[3];
        Color.colorToHSV(rgba.toColor(),hsvvalues);
        if(hsvvalues[2] < 0.1){ //if value is very low, it's probably just a dark reading rather than a purple or green ball
            return BallColor.UNKNOWN;
        }
        if(hsvvalues[1] < satThreshold){ //if saturation is very low, it's probably just a white reading rather than a purple or green ball
            return BallColor.UNKNOWN;
        }
        double[] distances = {
                circularDistance(hsvvalues[0], PGN[0]),
                circularDistance(hsvvalues[0], PGN[1]),
                circularDistance(hsvvalues[0], PGN[2])
        };
        int minidx = 2;
        for (int j = 0; j < 3; j++) {
            if (distances[j] < distances[minidx]) {
                minidx = j;
            }
        }
        switch (minidx){
            case 0:
                return BallColor.PURPLE;
            case 1:
                return BallColor.GREEN;
            default:
                return BallColor.UNKNOWN;
        }
    }
    public float[][] getRawHSV(){
        NormalizedRGBA back1 = ((NormalizedColorSensor) backc1).getNormalizedColors();
        NormalizedRGBA back2 = ((NormalizedColorSensor) backc2).getNormalizedColors();
        NormalizedRGBA middle1 = ((NormalizedColorSensor) middlec1).getNormalizedColors();
        NormalizedRGBA middle2 = ((NormalizedColorSensor) middlec2).getNormalizedColors();
        NormalizedRGBA front1 = ((NormalizedColorSensor) frontc1).getNormalizedColors();
        NormalizedRGBA front2 = ((NormalizedColorSensor) frontc2).getNormalizedColors();
        float[][] hsvvalues = new float[6][3];
        Color.colorToHSV(back1.toColor(),hsvvalues[0]);
        Color.colorToHSV(back2.toColor(),hsvvalues[1]);
        Color.colorToHSV(middle1.toColor(),hsvvalues[2]);
        Color.colorToHSV(middle2.toColor(),hsvvalues[3]);
        Color.colorToHSV(front1.toColor(),hsvvalues[4]);
        Color.colorToHSV(front2.toColor(),hsvvalues[5]);
        return hsvvalues;
    }
    private double circularDistance(double a, double b){
        double diff = abs(a-b);
        return min(diff,360-diff);
    }


    private static class MajorityWindow {
        private final ElapsedTime elapsedTime = new ElapsedTime();
        private final long windowMs;
        private final Deque<StampedBallOrder> samples = new ArrayDeque<>();

        private MajorityWindow(long windowMs) {
            this.windowMs = Math.max(1, windowMs);
            elapsedTime.reset();
        }

        private BallOrder update(BallOrder rawOrder) {
            if (rawOrder == null) {
                return new BallOrder(BallColor.UNKNOWN, BallColor.UNKNOWN, BallColor.UNKNOWN);
            }

            long nowMs = (long) elapsedTime.milliseconds();
            samples.addLast(new StampedBallOrder(nowMs, rawOrder));
            pruneOldSamples(nowMs);

            return new BallOrder(
                    resolveMajority(Position.FRONT),
                    resolveMajority(Position.MIDDLE),
                    resolveMajority(Position.BACK)
            );
        }

        private void pruneOldSamples(long nowMs) {
            long minTimestamp = nowMs - windowMs;
            while (!samples.isEmpty() && samples.peekFirst().timestampMs < minTimestamp) {
                samples.removeFirst();
            }
        }

        private BallColor resolveMajority(Position position) {
            int greenCount = 0;
            int purpleCount = 0;
            int unknownCount = 0;

            for (StampedBallOrder sample : samples) {
                BallColor color;
                switch (position) {
                    case FRONT:
                        color = sample.order.front;
                        break;
                    case MIDDLE:
                        color = sample.order.middle;
                        break;
                    case BACK:
                        color = sample.order.back;
                        break;
                    default:
                        color = BallColor.UNKNOWN;
                        break;
                }

                if (color == BallColor.GREEN) {
                    greenCount++;
                } else if (color == BallColor.PURPLE) {
                    purpleCount++;
                } else if (color == BallColor.UNKNOWN) {
                    unknownCount++;
                }
            }

            if (greenCount > purpleCount && greenCount > unknownCount) {
                return BallColor.GREEN;
            }

            if (purpleCount > greenCount && purpleCount > unknownCount) {
                return BallColor.PURPLE;
            }
            if(unknownCount > greenCount && unknownCount > purpleCount){
                return BallColor.UNKNOWN;
            }
            // In case of a tie, prefer the most recent sample's color if it's not UNKNOWN
            if (!samples.isEmpty()) {
                BallColor mostRecentColor;
                switch (position) {
                    case FRONT:
                        mostRecentColor = samples.peekLast().order.front;
                        break;
                    case MIDDLE:
                        mostRecentColor = samples.peekLast().order.middle;
                        break;
                    case BACK:
                        mostRecentColor = samples.peekLast().order.back;
                        break;
                    default:
                        mostRecentColor = BallColor.UNKNOWN;
                        break;
                }
                if (mostRecentColor != BallColor.UNKNOWN) {
                    return mostRecentColor;
                }
            }
            // If there's a tie and the most recent color is UNKNOWN, default to UNKNOWN
            return BallColor.UNKNOWN;
        }
    }

    private enum Position {
        FRONT,
        MIDDLE,
        BACK
    }

    private static class StampedBallOrder {
        private final long timestampMs;
        private final BallOrder order;

        private StampedBallOrder(long timestampMs, BallOrder order) {
            this.timestampMs = timestampMs;
            this.order = order;
        }
    }
}