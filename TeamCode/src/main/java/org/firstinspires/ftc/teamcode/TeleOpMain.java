package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.Queue;
import java.util.ArrayDeque;
import org.firstinspires.ftc.teamcode.shooter.AutoAlign;
import org.firstinspires.ftc.teamcode.shooter.ShootCalculator;
// import org.firstinspires.ftc.teamcode.utility.BallColor;
// import org.firstinspires.ftc.teamcode.utility.BallOrder;
// import org.firstinspires.ftc.teamcode.utility.ColorHandler;
// import org.firstinspires.ftc.teamcode.utility.ConfidenceFilter;
// import org.firstinspires.ftc.teamcode.utility.Debounce;
// import org.firstinspires.ftc.teamcode.utility.FieldCentricDrivePinPoint;
// import org.firstinspires.ftc.teamcode.utility.LEDHandler;
// import org.firstinspires.ftc.teamcode.utility.POS;
// import org.firstinspires.ftc.teamcode.utility.PersistentConstants;
// import org.firstinspires.ftc.teamcode.utility.STATE;
import org.firstinspires.ftc.teamcode.utility.*;

@Configurable
@TeleOp
public class TeleOpMain extends OpMode {

    //CONFIGURABLES
    public static double velocityMultiplier = 2.14;
    public static int velocityCompensation = 280; //tps
    public static double KP = 40;
    public static double KI = 0;
    public static double KD = 0.2;
    public static double KF = 15;
    public static double alignkp = -0.04;


    //drive initialization
    FieldCentricDrivePinPoint drive;


    //hardware
    Servo back, middle, front, lock, linkage;
    DcMotor intake;
    DcMotorEx shooter;
    boolean autoAlignActive = false;

    Limelight3A limelight;
    ColorSensor middlec, backc, frontc;




    //modules
    AutoAlign autoAligner;
    ColorHandler colorHandler;
    ShootQueue shootQueue;
        Queue<BallColor> colorOrder = new ArrayDeque<>(); // queue to hold detected ball colors for shoot order selection
        Boolean colorMode = null; //false = position based shooting, true = color based shooting
    PersistentConstants pc;
    LEDHandler ledHandler;
    Debounce debouncer = new Debounce(200); //quick debounce time for button presses
    Debounce longDebouncer = new Debounce(600); //slow debounce time for state changes
    ConfidenceFilter confidenceFilter = new ConfidenceFilter(100, 0.8); // filter for color detection confidence;

    ShootCalculator shooterCalculator;
    //variables
    double multiplier = 1.0;
    STATE currentState = STATE.IDLE;
    boolean limeLightWorking = true;
    double positionCompensation;
    double multiplierCompensation = 1.0;
    double shooterTPS = -1050;
    double ta, tx, ty,distanceToTarget;

    int pickedOrders = 0;
    BallOrder ballOrder;

    @Override
    public void init() {
        //hardware init
        shooter = hardwareMap.get(DcMotorEx.class, "shooter");
        shooter.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(KP,KI,KD,KF));
        intake = hardwareMap.get(DcMotor.class, "intake");
        lock = hardwareMap.get(Servo.class, "lock");
        

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
        shooterCalculator = new ShootCalculator(0,14,72); // TODO: set angle + elevation
            velocityCompensation = pc.getb();
            velocityMultiplier = pc.getm();
            shooterCalculator.setVelocityMultiplier(velocityMultiplier);
            shooterCalculator.setVelocityCompensation(velocityCompensation);
        autoAligner = new AutoAlign(drive, alignkp, 0.7);
        colorHandler = new SingleColorHandler(hardwareMap);
        shootQueue = new ShootQueue(hardwareMap);
        ledHandler = new LEDHandler(hardwareMap);
            ledHandler.setALLColor(LEDHandler.LED_BLUE);
            ledHandler.setColorsFromStatic();
        
    }

    @Override
    public void loop() {
        //CONTROL SCHEME:
        //gamepad 1: drive + intake control + currentState override
        // left trigger - intake
        // right trigger - auto align active
        // b button - advance state
        // y button - recover state (triangle)
        // a button - manual rapid shooting select


        //gamepad 2: shooter control - colors/positions;
        // dpad left/up/right - manual shoot position select
        // dpad down - cancel/restart selection
        // a/b(x o) buttons - color based shoot select
        // left trigger/bumper - lock override
        // right trigger/bumper - set automatic rapid shooting mode
        


        //pc.update(gamepad2);
        
        // handle user input
            //shooter adjustment control
            //lock override control


        // ===================================
        // LIMELIGHT + SHOOT CALCULATOR
        if(limeLightWorking) {
            autoAlignActive = gamepad1.right_trigger > 0.2;
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
                lock.setPosition(POSCONFIG.LOCKDISENGAGED); //disengage lock to allow balls to move in
                //run intake until 3 balls are detected
                //auto intake align????
                //once all 3 detected (or manual override) clear all shoot queue and move to READY

                if(gamepad1.right_bumper){ //power intake, could be constantly on
                intake.setPower(-POSCONFIG.INTAKEPOWER);
                } else { intake.setPower(0); }

                ballOrder = colorHandler.detectBallOrder(); //ball leds
                ledHandler.ballColors(ballOrder);


                //move on case
                if(longDebouncer.update("gamepad1.b", gamepad1.b)){
                    // clean shooter
                    shootQueue.clearAndReset();
                    
                    //otherwise, move to ready state to allow for shoot order selection
                    currentState = STATE.READY;
                    //clean color order and stop intake
                    colorOrder.clear();
                    intake.setPower(0);
                    pickedOrders = 0;
                }
                break;
            case READY:
                lock.setPosition(POSCONFIG.LOCKENGAGED); //engage lock
                intake.setPower(0); //ensure intake is off
                shooter.setVelocity(shooterTPS); //start revving up shooter

                //controller inputs shooting order; either manual position or color order based.
                //waits until controller selects shoot and then moves to SHOOTING
                
                //debounced buttons!

                //only one mode active at a time
                if(colorMode == null || colorMode == false) {
                    //manual shoote order control. Shows selected positions on LEDs
                    if(debouncer.update("gamepad2.dpad_left", gamepad2.dpad_left)){
                       shootQueue.addOne(POS.BACK);
                       LEDHandler.led1Color = LEDHandler.LED_WHITE;
                       pickedOrders++;
                       colorMode = false;
                    }
                    if(debouncer.update("gamepad2.dpad_up", gamepad2.dpad_up)){
                       shootQueue.addOne(POS.MIDDLE);
                       LEDHandler.led2Color = LEDHandler.LED_WHITE;
                       pickedOrders++;
                       colorMode = false;
                       
                    }
                    if(debouncer.update("gamepad2.dpad_right", gamepad2.dpad_right)){
                       shootQueue.addOne(POS.FRONT);
                       LEDHandler.led3Color = LEDHandler.LED_WHITE;
                       pickedOrders++;
                       colorMode = false;
                    }
                }
                if(colorMode == null || colorMode == true){
                    //color order based shooting control. Shows selected colors on LEDs
                    
                    if(debouncer.update("gamepad2.a", gamepad2.a)){ //color order based shooting
                        colorOrder.add(BallColor.PURPLE);
                        pickedOrders++;
                        colorMode = true;
                        switch (pickedOrders) {
                            case 1:
                                LEDHandler.led1Color = LEDHandler.LED_PURPLE;
                                break;
                            case 2:
                                LEDHandler.led2Color = LEDHandler.LED_PURPLE;
                                break;
                            case 3:
                                LEDHandler.led3Color = LEDHandler.LED_PURPLE;
                                break;
                            default:
                                    break;
                        }
                    }
                    if(debouncer.update("gamepad2.b", gamepad2.b)){
                        colorOrder.add(BallColor.GREEN);
                        pickedOrders++;
                        colorMode = true;
                        switch (pickedOrders) {
                            case 1:
                                LEDHandler.led1Color = LEDHandler.LED_GREEN;
                                break;
                            case 2:
                                LEDHandler.led2Color = LEDHandler.LED_GREEN;
                                break;
                            case 3:
                                LEDHandler.led3Color = LEDHandler.LED_GREEN;
                                break;
                            default:
                                    break;
                        }
                    }
                }
                //dpad down univeral reset for shoot order
                if(debouncer.update("gamepad2.dpad_down", gamepad2.dpad_down)){//clear and reset shoot order
                   shootQueue.clearQueue();
                   ballOrder = colorHandler.detectBallOrder();
                   ledHandler.ballColors(ballOrder);
                   pickedOrders = 0;
                   colorOrder.clear();
                   colorMode = null;
                }



                if(longDebouncer.update("gamepad1.b", gamepad1.b) || debouncer.update("gamepad2.y", gamepad2.y)){ // advance on gp1 override or gp2 progress
                    if(pickedOrders == 0){
                        //if no orders picked, do rapid
                        shootQueue.addRapid();
                        currentState = STATE.SHOOTING;
                        lock.setPosition(POSCONFIG.LOCKENGAGED); //engage lock to hold balls
                    } else {
                        //otherwise, calculate order
                        currentState = STATE.CALCULATING;
                    }
                }

                break;
            case CALCULATING:
                // finds optimal shooting order; immediately moves to SHOOTING
                // LEDS set to reflect shooting (automatically done in findOptimalOrder())

                //detect current ball order
                ballOrder = colorHandler.detectBallOrder(); // detect current ball order
                if(colorMode == false){
                    //do nothing; shoot order is already set; move to reset and shooting
                } else{
                    BallOrder targetOrder = BallOrder.queueToBallOrder(colorOrder); // convert queue of colors to ball order object
                    Queue<POS> shootSequence = ballOrder.findOptimalOrder(targetOrder, shootQueue.getCurrentPos()); // find optimal shoot sequence
                    shootQueue.setQueue(shootSequence); // send to shoot queue
                }
                colorOrder.clear(); //clear color order for next time
                colorMode = null; //reset color mode for next time
                currentState = STATE.SHOOTING; // start shooting

                ledHandler.ballColors(ballOrder); // update LEDs to show detected order before shooting

                break;
            case SHOOTING:
                shooter.setVelocity(shooterTPS); //ensure shooter is at correct velocity
                //handle shoot queue
                shootQueue.update();
                if(!shootQueue.isShooting()){ // once shootQueue isn't shooting anymore, go to idle.
                    currentState = STATE.IDLE;
                }
                break;
            case IDLE:
                //do nothing - before teleop starts or just driving around
                //wait until controller starts intake and then move to INTAKING
                //disengage lock
                ledHandler.setALLColor(LEDHandler.LED_OFF); // turn off LEDs when idle
                shooter.setVelocity(0); //stop shooter from revving
                if(gamepad1.right_bumper){ //higher threshold to prevent accidents
                    //move on to intaking
                    intake.setPower(POSCONFIG.INTAKEPOWER);
                    shootQueue.linkageBack(); // go back to give space for intake
                    currentState = STATE.INTAKING;
                }
                break;
            case RECOVER:
                //if the state is messed up, reset as if restarting the entire teleop
                //shootQueue.clearAndReset();
                currentState = STATE.IDLE;
                //reset colors
                colorOrder.clear();
                colorMode = null;
                shootQueue.clearAndReset();
                shootQueue.update(); // ensure reset is processed immediately
                //reset LEDs
                
                break;
        }
        // ================ END SMART STATES ===================


        // GLOBAL OVERRIDE CONTROLS (can be used in any state)
        //rapid shooting
        if(currentState == STATE.INTAKING || currentState == STATE.READY){
            if(debouncer.update("gamepad1.a", gamepad1.a)){ // move to RAPID SHOOTING mode, skipping calculation and shoot order selection
                colorOrder.clear();
                colorMode = null;
                shootQueue.clearAndReset();
                shootQueue.addRapid();
                currentState = STATE.SHOOTING;
            }
            
        }
        //recovery mode
        if(debouncer.update("gamepad1.y", gamepad1.y)){ //recover triangle
            currentState = STATE.RECOVER; 
        }
        
        //drive slow mode
        if(gamepad1.left_trigger>0.2){ multiplier = 0.5; } else { multiplier = 1.0; }

        //lock override
        if(gamepad2.left_bumper) { lock.setPosition(POSCONFIG.LOCKDISENGAGED); } else
        if(gamepad2.left_trigger > 0.4){ lock.setPosition(POSCONFIG.LOCKENGAGED); }




        //update LEDs
        ledHandler.setColorsFromStatic();
        
        colorHandler.detectBallOrder(); // keep updating the confidence filter and ball order

        //update drive
        if(autoAlignActive){
            //auto align
            autoAligner.alignToTargetWithManualDrive(tx, gamepad1.left_stick_y * multiplier, -gamepad1.left_stick_x * multiplier);
        } else {
            //normal driver control
            drive.drive(gamepad1, multiplier);
        }

        //telemetry
        telemetry.addData("heading", drive.getHeading());
        telemetry.addData("Current state", currentState);
    }
}
