package org.firstinspires.ftc.teamcode;


import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.shooter.AutoAlign;
import org.firstinspires.ftc.teamcode.shooter.LimeLightLocator;
import org.firstinspires.ftc.teamcode.shooter.PerfectShooting;
import org.firstinspires.ftc.teamcode.shooter.ShootCalculator;
import org.firstinspires.ftc.teamcode.utility.FieldCentricDrive;
import org.firstinspires.ftc.teamcode.utility.FieldCentricDrivePinPoint;
import org.firstinspires.ftc.teamcode.utility.PersistentStorage;

@TeleOp
@Configurable
public class TeleOpMainLimeLight extends OpMode{
    /*
    TODO 12-9-25
    FIND IF CONFLICT BETWEEN setVelocity and setPower
    IMPLEMENT LIMELIGHT
    ADD BUTTON FOR AUTOALIGN
     */
    //CONFIGURABLES
    public static double velocityMultiplier = 2.14;
    public static double velocityCompensation = 280; //tps
    public static double KP = 60;
    public static double KI = 0;
    public static double KD = 0.2;
    public static double KF = 17.2;
    public static double alignkp = -0.06;
    public static double strafeMultiplier = 0.7; // multiplier for diagonal strafe during auto-align
    public static double ball1mult = 1;
    public static double ball2mult = 1;
    public static double ball3mult = 1;
    //THE REST
    RevBlinkinLedDriver blinkin;
    private int maxTPS = 28 * 4000; // 4000 RPM with 28 ticks per revolution (for 6000 rpm motor)
    FieldCentricDrivePinPoint drive;
    DcMotorEx shooter;
    DcMotor intake;


    Servo front, back, middle, linkage;
    CRServo wheel;
    boolean inShoot = false;
    int shootSequenceState = 0;
    ElapsedTime shootSequenceTimer = new ElapsedTime();
    int shootMode = 0; // 0 = all balls, 1 = back only, 2 = middle only, 3 = front only
    int exp = 1;
    boolean autoAlignActive = false;
    Limelight3A limelight;
    AutoAlign autoAligner;
    LimeLightLocator locator;
    ShootCalculator shooterCalculator;
    double ta, tx, ty,distanceToTarget;
    double shooterSpeed = -1050; //default shooter velocity in ticks per second
    boolean limeLightWorking = true;
    double positionCompensation;
    double multiplierCompensation = 1.0;



    boolean lastUp, lastDown = false;
    @Override
    public void init() {
        velocityCompensation = PersistentStorage.loadX(this.hardwareMap.appContext,280);
        drive = new FieldCentricDrivePinPoint(hardwareMap);
        shooter = hardwareMap.get(DcMotorEx.class, "shooter");
        intake = hardwareMap.get(DcMotor.class, "intake");
//        blinkin = hardwareMap.get(RevBlinkinLedDriver.class, "blinkin");
        front = hardwareMap.servo.get("front");
        back = hardwareMap.servo.get("back");
        middle = hardwareMap.servo.get("middle");
        linkage = hardwareMap.servo.get("linkage");
        wheel = hardwareMap.crservo.get("wheel");
        wheel = hardwareMap.crservo.get("wheel");
        try {
            limelight = hardwareMap.get(Limelight3A.class, "limelight");
            limelight.setPollRateHz(100);
            limelight.start();
            limelight.pipelineSwitch(0);
        }
        catch (IllegalArgumentException e){//change to llworking = false on default and set to true when llworking
            limeLightWorking = false;
        }
        autoAligner = new AutoAlign(drive,alignkp,strafeMultiplier);
        shooterCalculator = new ShootCalculator(); // height of shooter from ground in inches

        shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,new PIDFCoefficients(KP, KI, KD, KF));
    }
    @Override
    public void loop() {
        telemetry.addData("Limelight",limeLightWorking ? "Active" : "Inactive");
        if(gamepad1.x){
            drive.resetIMU();
        }
        if(gamepad2.dpad_up){
            //increase shootcalculator.velocityCompensation
            if(!lastUp){
                shooterCalculator.setVelocityCompensation(shooterCalculator.getVelocityCompensation()+10);
                lastUp = true;
                PersistentStorage.saveX(hardwareMap.appContext, shooterCalculator.getVelocityCompensation());

            }
        } else if (gamepad2.dpad_down){
            //decrease shootcalculator.velocityCompensation
            if(!lastDown){
                shooterCalculator.setVelocityCompensation(shooterCalculator.getVelocityCompensation()-10);
                lastDown = true;
                PersistentStorage.saveX(hardwareMap.appContext, shooterCalculator.getVelocityCompensation());

            }
        } else{
            lastUp = false;
            lastDown = false;
        }
        telemetry.addData("heading",drive.getHeading());
        // Check for auto-align activation
        if(limeLightWorking) {
            autoAlignActive = gamepad1.right_trigger > 0.2;

            //Limelight logic. GAMEPAD BUTTON FOR ENABLE AUTOALIGN MUST PRECEDE THIS
            LLResult result = limelight.getLatestResult();
            shooterSpeed = Math.min(-1050, shooterCalculator.calculateRPMWForTELE(result, positionCompensation) * multiplierCompensation);
            if (result != null && result.isValid()) {
                ty = result.getTy();
                tx = result.getTx();
                ta = result.getTa();
                distanceToTarget = shooterCalculator.distanceToTarget; // Get calculated distance
                telemetry.addData("Limelight TX", tx);
                telemetry.addData("Limelight TY", ty);
                telemetry.addData("Limelight TA", ta);
                telemetry.addData("Distance to Target (in)", String.format("%.1f", distanceToTarget));
            }
            else{
                tx=0;
            }

        } else{
            autoAlignActive = false;
            shooterSpeed =-1050;
            tx=0;

        }

        telemetry.addData("Shooter Velocity (RPM)", shooterSpeed);

        // Add telemetry for debugging drive issues
        telemetry.addData("IMU Heading (deg)", Math.toDegrees(drive.getHeading()));
        telemetry.addData("Left Stick X", gamepad1.left_stick_x);
        telemetry.addData("Left Stick Y", gamepad1.left_stick_y);
        telemetry.addData("Right Stick X", gamepad1.right_stick_x);

        // Individual ball shooting controls
        if(gamepad1.dpad_left && !inShoot) { // Shoot front ball only
            inShoot = true;
            shootMode = 1;
            shootSequenceState = 1;
            shootSequenceTimer.reset();
            positionCompensation = 5;
            multiplierCompensation = ball1mult;
        } else if(gamepad1.dpad_up && !inShoot) { // Shoot middle ball only
            inShoot = true;
            shootMode = 2;
            shootSequenceState = 3;
            shootSequenceTimer.reset();
            positionCompensation = 11;
            multiplierCompensation = ball2mult;
        } else if(gamepad1.dpad_right && !inShoot) { // Shoot back ball only
            inShoot = true;
            shootMode = 3;
            shootSequenceState = 5;
            shootSequenceTimer.reset();
            positionCompensation = 15.5;
            multiplierCompensation = ball3mult;
        } else if(gamepad1.b && !inShoot) { // Shoot all balls (original sequence)
            inShoot = true;
            shootMode = 0;
            shootSequenceState = 1;
            shootSequenceTimer.reset();
        }

        // Shooter controls (only when not in sequence)
        if(!inShoot) {
            if(gamepad1.right_bumper) {
                shooter.setVelocity(shooterSpeed);
            } else {
                shooter.setPower(0);
            }
        }

        // Intake controls
        if(gamepad1.left_trigger > 0.1) {
            intake.setPower(gamepad1.left_trigger*0.7);
        } else if(gamepad1.left_bumper) {
            intake.setPower(-0.9);
            wheel.setPower(1);

        } else {
            intake.setPower(0);
            wheel.setPower(0);
        }

        if(inShoot){
            switch(shootSequenceState) {
                case 1: // Move linkage to back position and lift back ball
                    linkage.setPosition(0.3567);  // Move shooter to back position
                    shooter.setVelocity(shooterSpeed);
                    if(shootSequenceTimer.milliseconds() > 700) { // Wait for linkage to move
                        back.setPosition(0.6); // Push back ball up
                        shootSequenceTimer.reset();
                        shootSequenceState = 2;
                    }
                    break;

                case 2: // Wait then reset back servo and move to middle
                    if(shootSequenceTimer.milliseconds() > 500) { // Wait for ball to shoot
                        back.setPosition(0); // Reset back servo
                        if(shootMode == 1) { // Back ball only
                            shooter.setPower(0);
                            inShoot = false;
                            shootSequenceState = 0;
                        } else { // Continue to middle ball for full sequence
                            linkage.setPosition(0.25); // Move shooter to middle position
                            shootSequenceTimer.reset();
                            shootSequenceState = 3;
                        }
                    }
                    break;

                case 3: // Move linkage to middle position and lift middle ball
                    if(shootMode == 2) { // Middle ball only - need to position linkage first
                        linkage.setPosition(0.25); // Move shooter to middle position
                        shooter.setVelocity(shooterSpeed);
                    }
                    if(shootSequenceTimer.milliseconds() > 300) { // Wait for linkage to move
                        middle.setPosition(0.6); // Push middle ball up
                        shootSequenceTimer.reset();
                        shootSequenceState = 4;
                    }
                    break;

                case 4: // Wait then reset middle servo and move to front
                    if(shootSequenceTimer.milliseconds() > 500) { // Wait for ball to shoot
                        middle.setPosition(0); // Reset middle servo
                        if(shootMode == 2) { // Middle ball only
                            shooter.setPower(0);
                            inShoot = false;
                            shootSequenceState = 0;
                        } else { // Continue to front ball for full sequence
                            linkage.setPosition(0.0); // Move shooter to front position
                            shootSequenceTimer.reset();
                            shootSequenceState = 5;
                        }
                    }
                    break;

                case 5: // Move linkage to front position and lift front ball
                    if(shootMode == 3) { // Front ball only - need to position linkage first
                        linkage.setPosition(0.0); // Move shooter to front position
                        shooter.setVelocity(shooterSpeed);
                    }
                    if(shootSequenceTimer.milliseconds() > 300) { // Wait for linkage to move
                        front.setPosition(0.6); // Push front ball up
                        shootSequenceTimer.reset();
                        shootSequenceState = 6;
                    }
                    break;

                case 6: // End sequence
                    if(shootSequenceTimer.milliseconds() > 500) { // Wait for ball to shoot
                        front.setPosition(0); // Reset front servo
                        shooter.setPower(0);
                        inShoot = false;
                        shootSequenceState = 0;
                    }
                    break;
            }
        }

        telemetry.addData("shooter velocity", shooter.getVelocity());
        telemetry.update();


        //drive control
        if(!autoAlignActive){
            drive.drive(gamepad1, exp);
        }
        else{
            autoAligner.alignToTargetWithManualDrive(tx,gamepad1.left_stick_y,-gamepad1.left_stick_x*1.1); // Uses diagonal approach for 45-degree alignment
        }
    }
}