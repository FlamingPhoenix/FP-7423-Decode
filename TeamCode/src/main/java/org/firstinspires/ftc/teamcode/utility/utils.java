package org.firstinspires.ftc.teamcode.utility;

public class utils {
    /**
     * Run a function after a delay asynchronously
     * @param runnable The function to run - can be a lambda
     * @param delay milliseconds to wait before running the function
     */
    public static void setTimeout(Runnable runnable, int delay){
        new Thread(() -> {
            try {
                Thread.sleep(delay);
                runnable.run();
            }
            catch (Exception e){
                System.err.println(e);
            }
        }).start();
    }

    /**
     * Clamp a value between a minimum and maximum
     * @param val value to clamp
     * @param min Minimum value
     * @param max Maximum value
     * @return clamped value
     */
    public static double clamp(double val, double min, double max){
        return Math.max(min, Math.min(max, val));
    }
    /**
     * Clamp a value between 0 and 1
     * @param val value to clamp
     * @return clamped value
     */
    public static double clamp(double val){
        return clamp(val, 0.0, 1.0);
    }

    /**
     * Map a value from one range to another
     * @param val value to map
     * @param inMin minimum value of input range
     * @param inMax maximum value of input range
     * @param outMin minimum value of output range
     * @param outMax maximum value of output range
     * @return mapped value
     */
    public static double mapRange(double val, double inMin, double inMax, double outMin, double outMax){
        return (val - inMin) * (outMax - outMin) / (inMax - inMin) + outMin;
    }

}
