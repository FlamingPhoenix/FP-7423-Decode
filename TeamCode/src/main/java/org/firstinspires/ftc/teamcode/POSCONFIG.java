package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;

@Configurable
public class POSCONFIG {
    public static  double FRONT = 0.66;
    public static  double BACK = 1;
    public static  double MIDDLE = 0.85;
    public static double singlewait = 300; //ms to wait for linkage to move over a single ball
    public static double doublewait = 700; //ms to wait for linkage to move over two balls (over middle)
    public static double ballwait = 400; //ms to wait for ball to be pushed out after linkage is in position


    //TODO: Configure values
    public static double LOCKENGAGED;
    public static double LOCKDISENGAGED;
    public static double LIFTERUP;
    public static double LIFTERDOWN;
    public static double LIFTERBLOCKING;
    //
    public static double INTAKEPOWER = 0.9;
}
