package org.firstinspires.ftc.teamcode.utility;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.HashMap;

public class MotorContainer {
    public DcMotor fl, fr, bl, br;
    HardwareMap hardwareMap;
    HashMap<String, DcMotor> motors = new HashMap<>();
    public MotorContainer(String[] names, DcMotor[] motors, HardwareMap hardwareMap){
        this.hardwareMap = hardwareMap;
        if(names.length != motors.length){
            throw new IllegalArgumentException("names and motors must be the same length");
        }
        for(int i = 0; i < names.length; i++){
            this.motors.put(names[i], motors[i]);
        }
    }
    public MotorContainer(DcMotor fl, DcMotor fr, DcMotor bl, DcMotor br, HardwareMap hardwareMap){
        this.hardwareMap = hardwareMap;
        motors.put("fl", fl);
        motors.put("fr", fr);
        motors.put("bl", bl);
        motors.put("br", br);
    }
    public void initMotors(DcMotor[] motors, String[] names, HardwareMap hardwareMap){
        if(names.length != motors.length){
            throw new IllegalArgumentException(String.format("names and motors must be the same length. Got %d motors, %d names",motors.length, names.length));
        }
        for(int i = 0; i < names.length; i++){
            motors[i] = hardwareMap.dcMotor.get(names[i]);
        }
    }
    public void initMotors(DcMotor[] motors, String[] names){
        if(names.length != motors.length){
            throw new IllegalArgumentException(String.format("names and motors must be the same length. Got %d motors, %d names",motors.length, names.length));
        }
        for(int i = 0; i < names.length; i++){
            motors[i] = hardwareMap.dcMotor.get(names[i]);
        }
    }
    public void initMotors(String[] names){
        for(String name : names){
            motors.put(name, hardwareMap.dcMotor.get(name));
        }
    }
    public DcMotor getMotor(String name){
        return motors.get(name);
    }
    public void setMotorPower(String name, double power){
        motors.get(name).setPower(power);
    }
    public void setMotorsPowers(String[] names, double[] powers){
        if(names.length != powers.length){
            throw new IllegalArgumentException(String.format("names and powers must be the same length. Got %d names, %d powers",names.length, powers.length));
        }
        for(int i = 0; i < names.length; i++){
            motors.get(names[i]).setPower(powers[i]);
        }
    }

}
