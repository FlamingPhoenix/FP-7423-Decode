package org.firstinspires.ftc.teamcode.legacy;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

public class RailController {
    DcMotorEx railMotor;
    final double maThreshold = 100;
    public RailController(HardwareMap hardwareMap) {
        railMotor = hardwareMap.get(DcMotorEx.class, "railMotor");
        railMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
    public void calibrate(){
        while(railMotor.getCurrent(CurrentUnit.MILLIAMPS) < maThreshold){
            railMotor.setPower(0.5);
        }
        railMotor.setPower(0);
        railMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
}
