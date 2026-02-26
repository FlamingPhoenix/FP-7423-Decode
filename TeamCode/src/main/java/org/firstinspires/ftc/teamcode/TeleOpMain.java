package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.Queue;

import org.firstinspires.ftc.teamcode.shooter.AutoAlign;
import org.firstinspires.ftc.teamcode.shooter.ShootCalculator;
import org.firstinspires.ftc.teamcode.utility.BallColor;
import org.firstinspires.ftc.teamcode.utility.BallOrder;
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
        Queue<BallColor> colorOrder;
        Boolean colorMode = null; //false = position based shooting, true = color based shooting
    PersistentConstants pc;
    LEDHandler ledHandler;
    Debounce debouncer = new Debounce(200); //quick debounce time for button presses
    Debounce longDebouncer = new Debounce(600); //slow debounce time for state changes


    //variables
    double multiplier = 1.0;
    STATE currentState = STATE.IDLE;
    boolean limeLightWorking = false;
    double shooterTPS = -1050;
    double tx;

    @Override
    public void init() {
        //hardware init
        //shooter = hardwareMap.get(DcMotorEx.class, "shooter");
        intake = hardwareMap.get(DcMotor.class, "intake");

        try {
            limelight = hardwareMap.get(Limelight3A.class, "limelight");
            limelight.setPollRateHz(100);
            limelight.start();
            limelight.pipelineSwitch(0);
        }
        catch (IllegalArgumentException e){//change to llworking = false on default and set to true when llworking
            limeLightWorking = false;
        }

        //modules init
        drive = new FieldCentricDrivePinPoint(hardwareMap, true);
        pc = new PersistentConstants(this.hardwareMap.appContext);
        shootCalculator = new ShootCalculator(30,4,72); // TODO: set angle + elevation
            velocityCompensation = pc.getb();
            velocityMultiplier = pc.getm();
            shootCalculator.setVelocityMultiplier(velocityMultiplier);
            shootCalculator.setVelocityCompensation(velocityCompensation);
        /*
        ColorHandler colorHandler = new SingleColorHandler(hardwareMap);
        shootQueue = new ShootQueue(hardwareMap);
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


        // ===================================
        // LIMELIGHT + SHOOT CALCULATOR
        if(limeLightWorking) {
            autoAlignActive = false; // TODO: SET CONTROL
            LLResult result = limelight.getLatestResult();
            shooterTPS = shooterCalculator.calculateRPMForTele(result, positionCompensation) * multiplierCompensation;
            if (result != null && result.isValid()) { 
                tx = result.getTx();
                distanceToTarget = shooterCalculator.distanceToTarget; // Get calculated distance
//                telemetry.addData("Limelight TX", tx);
                telemetry.addData("Distance to Target (in)", String.format("%.1f", distanceToTarget));
            } else{ tx=0; autoAlignActive = false; }
        } else{ autoAlignActive = false; shooterTPS =-1050; tx=0; }


        // ===================================
        // SMART STATES
        switch (currentState) {
            case INTAKING:

                //run intake until 3 balls are detected
                //auto intake align????
                //once all 3 detected (or manual override) clear all shoot queue and move to READY

                //if(gamepad1.right_trigger > 0.2){
                //intake.setPower(INTAKEPOWER);
                //} else { intake.setPower(0); }

                //BallOrder ballOrder = colorHandler.detectBallOrder();
                //ledHandler.ballColors(ballOrder);
                //if(ballOrder.isFull() || longDebouncer.update("gamepad1.right_bumper", gamepad1.right_bumper)){
                    //shootQueue.clearAndReset();
                    //colorOrder.clear();
                    //currentState = STATE.READY;
                    //intake.setPower(0.1); //???keep balls from falling out????
                //}
                break;
            case READY:
                //engage lock
                //lock.setPosition(POSCONFIG.LOCKENGAGED);

                //controller inputs shooting order; either manual position or color order based.
                //waits until controller selects shoot and then moves to SHOOTING
                //start revving up the shooter?
                
                //debounced buttons!

                //only one mode active at a time
                if(colorMode == null || colorMode == false) {
                    //manual shoote order control. Shows selected positions on LEDs
                    colorMode = false;
                    //if(debouncer.update("gamepad2.dpad_down", gamepad2.dpad_down)){
                    //    shootQueue.addOne(POS.BACK);
                    //    ledHandler.led3Color = LEDHandler.LED_WHITE;
                    //}
                    //if(debouncer.update("gamepad2.dpad_up", gamepad2.dpad_up)){
                    //    shootQueue.addOne(POS.MIDDLE);
                    //    ledHandler.led2Color = LEDHandler.LED_WHITE;
                    //}
                    //if(debouncer.update("gamepad2.dpad_right", gamepad2.dpad_right)){
                    //    shootQueue.addOne(POS.FRONT);
                    //    ledHandler.led1Color = LEDHandler.LED_WHITE;
                    //}
                    //if(debouncer.update("gamepad2.dpad_down", gamepad2.dpad_down)){//clear and reset shoot order
                    //    shootQueue.clearQueue();
                    //    BallOrder ballOrder = colorHandler.detectBallOrder();
                    //    ledHandler.ballColors(ballOrder);
                    //}
                }
                if(colorMode == null || colorMode == true){
                    //color order based shooting control. Shows selected colors on LEDs
                    colorMode = true;
                    //if(debouncer.update("gamepad2.a", gamepad2.a)){ //color order based shooting
                    //    colorOrder.add(BallColor.PURPLE);
                    //}
                    //if(debouncer.update("gamepad2.b", gamepad2.b)){
                    //    colorOrder.add(BallColor.GREEN);
                    //}
                }
                



                //if(longDebouncer.update("gamepad1.right_bumper", gamepad1.right_bumper)){
                //    currentState = STATE.CALCULATING;
                //}

                break;
            case CALCULATING:
                // finds optimal shooting order; immediately moves to SHOOTING
                // LEDS set to reflect shooting (automatically done in findOptimalOrder())

                if(colorMode == false){
                    //do nothing; shoot order is already set; move to reset and shooting
                } else{
                    //BallOrder targetOrder = BallOrder.queueToBallOrder(colorOrder); // convert queue of colors to ball order object
                    //BallOrder ballOrder = colorHandler.detectBallOrder(); // detect current ball order
                    //Queue<POS> shootSequence = ballOrder.findOptimalOrder(targetOrder, shootQueue.getCurrentPos()); // find optimal shoot sequence
                    //shootQueue.setQueue(shootSequence); // send to shoot queue
                }
                colorOrder.clear(); //clear color order for next time
                colorMode = null; //reset color mode for next time
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
                if(gamepad1.right_trigger > 0.6){
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
