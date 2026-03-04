package org.firstinspires.ftc.teamcode.utility;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.ArrayDeque;
import java.util.Deque;

public class SingleColorHandler implements ColorHandler {
    private static final long CONFIDENCE_WINDOW_MS = 500;

    private final ColorSensor backc;
    private final ColorSensor middlec;
    private final ColorSensor frontc;
    private final MajorityWindow majorityWindow;

    public SingleColorHandler(HardwareMap hardwareMap) {
        backc = hardwareMap.get(ColorSensor.class, "backc1");
        middlec = hardwareMap.get(ColorSensor.class, "middlec1");
        frontc = hardwareMap.get(ColorSensor.class, "frontc1");
        majorityWindow = new MajorityWindow(CONFIDENCE_WINDOW_MS);
    }

    @Override
    public BallOrder detectBallOrder() {
        BallOrder rawOrder = detectRawBallOrder();
        return majorityWindow.update(rawOrder);
    }

    @Override
    public BallOrder detectRawBallOrder() {
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
                }
            }

            if (greenCount > purpleCount) {
                return BallColor.GREEN;
            }

            if (purpleCount > greenCount) {
                return BallColor.PURPLE;
            }

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
