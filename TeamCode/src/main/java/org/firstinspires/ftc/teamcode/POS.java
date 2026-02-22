package org.firstinspires.ftc.teamcode;

public enum POS {
    FRONT(POSCONFIG.FRONT,2),
    BACK(POSCONFIG.BACK,0),
    MIDDLE(POSCONFIG.MIDDLE,1),
    RESET(POSCONFIG.BACK,-1);

    public final double linkagePos;
    public final int index;
    POS(double linkagePos, int index) {
        this.linkagePos = linkagePos;
        this.index = index;
    }
}
