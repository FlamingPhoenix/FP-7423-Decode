package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

/*
 * This is a utility program to find the position of a servo
 * add the name of the servo to the servoNames array
 * then run the program
 * use dpad left & right to select the servo
 * use dpad up & down for fine control
 * use left stick up & down for coarse control
 * use left & right bumper to change the increment multiplier of dpad
 */
@TeleOp
public class FindServoPosition extends OpMode {
    String[] servoNames ={"lock"};
    Servo[] servos = {};
    int selectedServo = 0;
    int incrementMultiplier = 1;
    boolean didchange = false;
    @Override
    public void init() {
        servos = new Servo[servoNames.length];
        for(int i = 0; i < servoNames.length; i++){
            servos[i] = hardwareMap.servo.get(servoNames[i]);
        }
    }
    @Override
    public void loop() {
        if(gamepad1.right_bumper){
            incrementMultiplier = 10;
        }
        if(gamepad1.left_bumper){
            incrementMultiplier = 1;
        }
        if(gamepad1.dpad_right && !didchange){
            selectedServo = (selectedServo + 1) % (servos.length);
            didchange = true;
        }else if(gamepad1.dpad_left && !didchange){
            selectedServo = (selectedServo - 1) % (servos.length);
            didchange = true;
        }else if(gamepad1.dpad_up && !didchange){
            servos[selectedServo].setPosition(servos[selectedServo].getPosition() + 0.001*incrementMultiplier);
            didchange = true;
        }else if(gamepad1.dpad_down && !didchange){
            servos[selectedServo].setPosition(servos[selectedServo].getPosition() - 0.001*incrementMultiplier);
            didchange = true;
        }else if(gamepad1.left_stick_y > 0.1 && !didchange){
            servos[selectedServo].setPosition(servos[selectedServo].getPosition() + 0.1);
            didchange = true;
        }else if(gamepad1.left_stick_y < -0.1 && !didchange) {
            servos[selectedServo].setPosition(servos[selectedServo].getPosition() - 0.1);
            didchange = true;
        } else{
            didchange = false;
        }
        for(int i = 0; i < servos.length; i++){
            telemetry.addData(servoNames[i], servos[i].getPosition());
        }
        telemetry.addData("selected servo", servoNames[selectedServo]);
        telemetry.addData("increment multiplier", incrementMultiplier);
        telemetry.update();
    }
}