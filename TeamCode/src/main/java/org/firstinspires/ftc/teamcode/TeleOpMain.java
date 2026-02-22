package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.shooter.AutoAlign;
import org.firstinspires.ftc.teamcode.shooter.ShootCalculator;
import org.firstinspires.ftc.teamcode.utility.FieldCentricDrivePinPoint;

@Configurable
@TeleOp
public class TeleOpMain extends OpMode {

    //CONFIGURABLES
    public static double velocityMultiplier = 2.14;
    public static int velocityCompensation = 280; //tps
    public static double KP = 60;
    public static double KI = 0;
    public static double KD = 0.2;
    public static double KF = 15;

    //drive initialization
    FieldCentricDrivePinPoint drive;


    //hardware
    Servo back, middle, front, lock, linkage;
    DcMotor intake;
    DcMotorEx shooter;
    Limelight3A limelight;
    ColorSensor middlec, backc, frontc;




    //modules
    ShootCalculator shootCalculator;
    AutoAlign autoAligner;


    //variables
    double multiplier = 1.0;

    @Override
    public void init() {
        drive = new FieldCentricDrivePinPoint(hardwareMap, true);
    }

    @Override
    public void loop() {


        drive.drive(gamepad1, multiplier);
    }
}
