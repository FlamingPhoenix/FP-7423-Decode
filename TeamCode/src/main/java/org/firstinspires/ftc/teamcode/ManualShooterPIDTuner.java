package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

@TeleOp
@Configurable
public class ManualShooterPIDTuner extends OpMode {
    private TelemetryManager panelstelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
    private DcMotorEx shooterMotor;
    public static double TARGET_SPEED = 1200; // tps
    public static double KP = 0.1;
    public static double KI = 0.0;
    public static double KD = 0.0;
    public static double KF = 0.0;
    private PIDFCoefficients pidfCoefficients = new PIDFCoefficients(KP, KI, KD, KF);
    @Override
    public void init() {
        shooterMotor = hardwareMap.get(DcMotorEx.class, "shooter");
        panelstelemetry.addData("Manual Shooter PID Tuner", "Initialized");
        shooterMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
    }
    @Override
    public void loop() {
        shooterMotor.setVelocityPIDFCoefficients(KP, KI, KD, KF);
        shooterMotor.setVelocity(TARGET_SPEED);
        double currentVelocity = shooterMotor.getVelocity();
        panelstelemetry.addData("Target Speed (tps)", TARGET_SPEED);
        panelstelemetry.addData("Current Speed (tps)", currentVelocity);
        panelstelemetry.addData("Error", TARGET_SPEED - currentVelocity);
        panelstelemetry.addData("KP", KP);
        panelstelemetry.addData("KI", KI);
        panelstelemetry.addData("KD", KD);
        panelstelemetry.addData("KF", KF);
        panelstelemetry.addData("Motor Power", shooterMotor.getPower());
        panelstelemetry.update(telemetry);

    }

}
