package org.firstinspires.ftc.teamcode.utility;

import android.content.Context;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class PersistentConstants {
    PersistentStorage ps;
    boolean lastleft, lastright, lastup, lastdown;
    Context context;
    int b;
    double m;
    public PersistentConstants(Context context){
        this.context = context;
        b = getb();
        m = getm();
    }


    public int getb(){
        return (int) PersistentStorage.loadDouble(context,"b",280);
    }
    public double getm(){
        return PersistentStorage.loadDouble(context,"m",2.14);
    }
    public double setb(int b){
        PersistentStorage.saveDouble(context,"b",b);
        return b;
    }
    public double setm(double m){
        PersistentStorage.saveDouble(context,"m",m);
        return m;
    }
    public void update(Gamepad gamepad2){
        boolean left = gamepad2.dpad_left;
        boolean right = gamepad2.dpad_right;
        boolean up = gamepad2.dpad_up;
        boolean down = gamepad2.dpad_down;
        if (left && !lastleft) {
            m-=0.01;
            setm(m);
            lastleft=true;
        }
        if (right && !lastright) {
            m+=0.01;
            setm(m);
            lastright=true;
        }
        if (up && !lastup) {
            b+=10;
            setb(b);
            lastup=true;
        }
        if (down && !lastdown) {
            b-=10;
            setb(b);
            lastdown=true;
        }
        if (!left) lastleft=false;
        if (!right) lastright=false;
        if (!up) lastup=false;
        if (!down) lastdown=false;

    }
}
