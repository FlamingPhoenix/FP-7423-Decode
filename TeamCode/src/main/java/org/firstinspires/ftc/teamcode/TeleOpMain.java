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
import org.firstinspires.ftc.teamcode.utility.ColorHandler;
import org.firstinspires.ftc.teamcode.utility.Debounce;
import org.firstinspires.ftc.teamcode.utility.FieldCentricDrivePinPoint;
import org.firstinspires.ftc.teamcode.utility.LEDHandler;
import org.firstinspires.ftc.teamcode.utility.PersistentConstants;
import org.firstinspires.ftc.teamcode.utility.STATE;

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
    ColorHandler colorHandler;
    ShootQueue shootQueue;
    PersistentConstants pc;
    LEDHandler ledHandler;
    Debounce debouncer = new Debounce(300); //300ms debounce time for button presses


    //variables
    double multiplier = 1.0;
    STATE currentState = STATE.IDLE;

    @Override
    public void init() {
        //hardware init
        shooter = hardwareMap.get(DcMotorEx.class, "shooter");
        intake = hardwareMap.get(DcMotor.class, "intake");

        /*
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
            limelight.setPollRateHz(100);
            limelight.start();
            limelight.pipelineSwitch(0);
         */

        //modules init
        drive = new FieldCentricDrivePinPoint(hardwareMap, true);
        pc = new PersistentConstants(this.hardwareMap.appContext);
        shootCalculator = new ShootCalculator(30,4,72); // TODO: set angle + elevation
            velocityCompensation = pc.getb();
            velocityMultiplier = pc.getm();
            shootCalculator.setVelocityMultiplier(velocityMultiplier);
            shootCalculator.setVelocityCompensation(velocityCompensation);
        /*
        shootQueue = new ShootQueue(hardwareMap);
        colorHandler = new ColorHandler(hardwareMap);
        ledHandler = new LEDHandler(hardwareMap);
            ledHandler.setALLColor(LEDHandler.LED_BLUE);
            ledHandler.setColorsFromStatic();
         */
    }

    @Override
    public void loop() {
        //CONTROL SCHEME:
        //gamepad 1: drive + intake control + currentState override
        //gamepad 2: shooter control - colors/positions;


        //pc.update(gamepad2);
        // handle user input
            //shooter adjustment control
            //multiplier control
            //intake control
            //auto align control
            //lock override control

        //handle limelight
        //handle shooter speed

        switch (currentState) {
            case INTAKING:

                //run intake until 3 balls are detected
                //auto intake align????
                //once all 3 detected (or manual override) clear all shoot queue and move to READY


                //intake.setPower(INTAKEPOWER);
                //BallOrder ballOrder = colorHandler.detectBallOrder();
                //ledHandler.ballColors(ballOrder);
                //if(ballOrder.isFull() || gamepadoverride){
                //shootQueue.clearQueue();
                //currentState = STATE.READY;
                //intake.setPower(0.1); //???keep balls from falling out????
                //}
                break;
            case READY:
                //engage lock
                //set LED colors to match ball order
                //controller inputs shooting order; either manual position or color order based.
                //waits until controller selects shoot and then moves to SHOOTING
                //start revving up the shooter?
                
                //debounced buttons!
                //if(debouncer.update("gamepad2.dpad_down", gamepad2.dpad_down)){
                //    shootQueue.addOne(POS.BACK);
                //}
                //if(debouncer.update("gamepad2.dpad_up", gamepad2.dpad_up)){
                //    shootQueue.addOne(POS.MIDDLE);
                //}
                //if(debouncer.update("gamepad2.dpad_right", gamepad2.dpad_right)){
                //    shootQueue.addOne(POS.FRONT);
                //}

                //if(gamepad1.right_bumper){
                //    currentState = STATE.CALCULATING;
                //}

                break;
            case CALCULATING:
                // finds optimal shooting order; immediately moves to SHOOTING
                // LEDS set to reflect shooting (automatically done in findOptimalOrder())
                //Queue<POS> shootSequence = ballOrder.findOptimalOrder(shootCalculator.calculateTargetOrder());
                //shootQueue.setQueue(shootSequence);
                currentState = STATE.SHOOTING;
                break;
            case SHOOTING:
                //handle shoot queue
                //shootQueue.update();
                //once shootqueue.ishooting() is false, move to IDLE
//                if(!shootQueue.isShooting()){
//                    currentState = STATE.IDLE;
//                }
                break;
            case IDLE:
                //disengage lock
                //do nothing - before teleop starts or just driving around
                //wait until controller starts intake and then move to INTAKING
                if(gamepad1.right_trigger > 0.4){
                    //intake.setPower(INTAKEPOWER);
                    currentState = STATE.INTAKING;
                }
                break;
            case RECOVER:
                //if the state is messed up, reset as if restarting the entire teleop
                //shootQueue.clearAndReset();
                currentState = STATE.IDLE;
                break;
        }






        //update LEDs
        //ledHandler.setColorsFromStatic();

        //update drive
        drive.drive(gamepad1, multiplier);

        //telemetry
        telemetry.addData("heading", drive.getHeading());
    }
}
