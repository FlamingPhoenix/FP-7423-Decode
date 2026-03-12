package org.firstinspires.ftc.teamcode.autos;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.PathChain;
import com.pedropathing.paths.PathConstraints;
import com.pedropathing.util.Timer;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import org.firstinspires.ftc.teamcode.legacy.POSCONFIG_OLD;
import org.firstinspires.ftc.teamcode.legacy.pedroPathingLegacy.ConstantsOLD;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "Red Auto Close NEW", group = "Autonomous")
@Configurable // Panels
public class redCloseAuto extends OpMode {

    private TelemetryManager panelsTelemetry; // Panels Telemetry instance
    public Follower follower; // Pedro Pathing follower instance
    private Timer pathTimer; // Timer for path state transitions
    private int pathState; // Current autonomous path state (state machine)
    private Paths paths; // Paths defined in the Paths class

    // Limelight hardware
    Limelight3A limelight;
    boolean limeLightWorking = true;
    int aprilTagId = -1;
    boolean aprilTagDetected = false;
    ElapsedTime aprilTagScanTimer = new ElapsedTime();

    // Shooting hardware
    DcMotorEx shooter;
    Servo front, back, middle, linkage, gate;

    // Intake hardware
    DcMotor intake;
    boolean inShoot = false;
    int shootSequenceState = 0;
    ElapsedTime shootSequenceTimer = new ElapsedTime();
    double shooterSpeed = -1620;
    double firstBallSpeed = -1620; // Slower speed for first ball
    int shootingOrder = 0; // 0 = normal (back->middle->front), 1 = reverse (front->middle->back), 2 = middle->back->front
    int storedShootingOrder = 0; // Store AprilTag order for second shooting sequence
    boolean isFirstShoot = true; // Track if this is the first shooting sequence
    boolean keepShooterWarmed = false; // Keep shooter running at -1000 when not shooting

    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
        pathTimer = new Timer();
        shootSequenceTimer = new ElapsedTime();
        Constants.driveConstants.setUseBrakeModeInTeleOp(true);
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(121.872, 122.018, Math.toRadians(45)));

        // Initialize Limelight
        try {
            limelight = hardwareMap.get(Limelight3A.class, "limelight");
            limelight.setPollRateHz(100);
            limelight.start();
            limelight.pipelineSwitch(0); // Start with shooting pipeline
        }
        catch (IllegalArgumentException e){
            limeLightWorking = false;
        }

        // Initialize shooting hardware
        shooter = hardwareMap.get(DcMotorEx.class, "shooter");
        front = hardwareMap.servo.get("front");
        back = hardwareMap.servo.get("back");
        middle = hardwareMap.servo.get("middle");
        linkage = hardwareMap.servo.get("linkage");
        gate = hardwareMap.servo.get("lock");

        // Initialize intake hardware
        intake = hardwareMap.get(DcMotor.class, "intake");

        // Setup shooter
        shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(60, 0, 0.2, 17.2));

        paths = new Paths(follower); // Build paths

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.debug("Limelight", limeLightWorking ? "Active" : "Inactive");
        panelsTelemetry.update(telemetry);

    }

    @Override
    public void loop() {
        follower.update(); // Update Pedro Pathing
        pathState = autonomousPathUpdate(); // Update autonomous state machine
        updateShootingSequence(); // Update shooting sequence

        // Log values to Panels and Driver Station
        panelsTelemetry.debug("Path State", pathState);
        panelsTelemetry.debug("X", follower.getPose().getX());
        panelsTelemetry.debug("Y", follower.getPose().getY());
        panelsTelemetry.debug("Heading", follower.getPose().getHeading());
        panelsTelemetry.debug("Shooting", inShoot ? "Active" : "Inactive");
        panelsTelemetry.debug("Shoot State", shootSequenceState);
        panelsTelemetry.debug("AprilTag ID", aprilTagId);
        panelsTelemetry.debug("Shooting Order", shootingOrder);
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void start() {
        // Initialize all motors to safe states
        intake.setPower(0);  // Stop intake initially
        shooter.setVelocity(firstBallSpeed); // Start shooter with -1100 speed for first ball

        // Initialize servo positions to safe states
        linkage.setPosition(0.66); // Move linkage to first ball (front) position immediately
        front.setPosition(0);
        back.setPosition(0.35);
        middle.setPosition(0.04);
        gate.setPosition(0.69); // Start with gate open

        setPathState(0);
    }

    public static class Paths {
        public PathChain Path1; // go back to shoot first
        public PathChain Path2; // move to balls
        public PathChain Path3; // intake balls
        public PathChain Path4; // go shoot
        public PathChain Path5; // open gate
        public PathChain Path6; // intake gate
        public PathChain Path7; // wiggle gate
        public PathChain Path8; // go shoot
        public PathChain Path9; // intake path
        public PathChain Path10; // shoot path

        public Paths(Follower follower) {
            Path1 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(121.872, 122.018),

                                    new Pose(97.465, 96.556)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(48))

                    .build();

            Path2 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(97.465, 96.556),
                                    new Pose(87.004, 62.119),
                                    new Pose(97.153, 61.186)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(48), Math.toRadians(0))

                    .build();

            Path3 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(97.153, 61.186),

                                    new Pose(131.580, 60.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))

                    .build();

            Path4 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(131.580, 60.000),
                                    new Pose(84.790, 60.526),
                                    new Pose(88.854, 86.565)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(48))

                    .build();

            Path5 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(88.854, 86.565),
                                    new Pose(108.242, 51.563),
                                    new Pose(127.394, 60.257)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(48), Math.toRadians(0))

                    .build();

            Path6 = follower.pathBuilder()
          .addPath(
            new BezierCurve(
              new Pose(127.394, 60.257),
            new Pose(124.072, 58.035),
            new Pose(131.719, 53.859)
            )
          )
          .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(60))
          .build();

            Path7 = follower.pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(131.719, 53.859),
                                    new Pose(127.906, 48.269),
                                    new Pose(136.122, 59.488),
                                    new Pose(131.719, 53.913)
                            )

          )
          .setLinearHeadingInterpolation(Math.toRadians(60), Math.toRadians(40))
          .build();
            Path8 = follower.pathBuilder()
          .addPath(
            new BezierCurve(
              new Pose(131.719, 53.913),
            new Pose(90.671, 63.579),
            new Pose(84.916, 79.217)
            )
          )
          .setLinearHeadingInterpolation(Math.toRadians(40), Math.toRadians(48))
          .build();

      Path9 = follower.pathBuilder()
          .addPath(
            new BezierCurve(
              new Pose(84.916, 79.217),
            new Pose(81.183, 86.065),
            new Pose(125.875, 82.249)
            )
          )
          .setLinearHeadingInterpolation(Math.toRadians(48), Math.toRadians(0))
          .build();

      Path10 = follower.pathBuilder()
          .addPath(
            new BezierLine(
              new Pose(125.875, 82.249),
            new Pose(88.435, 108.426)
            )
          )
          .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(34))
          .build();

//            Path5.setHeadingInterpolator(
//                    HeadingInterpolator.piecewise(
//                            new HeadingInterpolator.PiecewiseNode(
//                                    0,
//                                    0.5,
//                                    HeadingInterpolator.linear(Math.toRadians(48),0)
//                            ),
//                            new HeadingInterpolator.PiecewiseNode(
//                                    0.5,
//                                    1.0,
//                                    HeadingInterpolator.linear(0,Math.toRadians(30))
//                            )
//                    )
//            );
        }
    }


    public int autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(paths.Path1);
                setPathState(1);
                break;
            case 1:
                if (follower.getCurrentTValue()>0.8) {
                    startShooting();
                    setPathState(2);
                }
                break;
            case 2:
                if (!inShoot) {
                    setPathState(3);
                }
                break;
            case 3:
                follower.followPath(paths.Path2);
                setPathState(4);
                break;
            case 4:
                if (!follower.isBusy()) {
                    intake.setPower(-1);
                    pathTimer.resetTimer();
                    setPathState(5);
                }
                break;
            case 5:
                if (pathTimer.getElapsedTimeSeconds() > 0.1) {
                    follower.followPath(paths.Path3, 0.6, true);
                    setPathState(6);
                }
                break;
            case 6:
                if (!follower.isBusy()) {
                    pathTimer.resetTimer();
                    setPathState(7);
                }
                break;
            case 7:
                if (pathTimer.getElapsedTimeSeconds() > 0.1) {
                    gate.setPosition(0.4628); // Close gate after first intake
                    setPathState(8);
                }
                break;
            case 8:
                follower.followPath(paths.Path4);
                shooter.setVelocity(-1700);
                setPathState(9);
                break;
            case 9:
                if (!follower.isBusy()) {
                    pathTimer.resetTimer();
                    setPathState(10);
                }
                break;
            case 10:
                if (pathTimer.getElapsedTimeSeconds() > 0.1) {
                    intake.setPower(0);
                    shooterSpeed = -1800;
                    firstBallSpeed = -1800;
                    startShooting();
                    setPathState(11);
                }
                break;
            case 11:
                if(!inShoot){
                    setPathState(12);
                }
                break;
            case 12:
                follower.followPath(paths.Path5);
                setPathState(13);
                break;
            case 13:
                if(!follower.isBusy()){
                    if(pathTimer.getElapsedTimeSeconds() > 0.2) {
                        follower.followPath(paths.Path6);
                        intake.setPower(-1);
                        setPathState(14);
                    }
                }
            case 14:
                if(!follower.isBusy()){
                    if(pathTimer.getElapsedTimeSeconds()>1) {
                        follower.followPath(paths.Path7, 0.4, true);
                        setPathState(15);
                    }
                }
                break;
            case 15:
                if(!follower.isBusy()){
                    if(pathTimer.getElapsedTimeSeconds()>2) {
                        gate.setPosition(0.4628); // Close gate after fourth intake cycle
                        intake.setPower(-0.3);
                        shooter.setVelocity(shooterSpeed);
                        follower.followPath(paths.Path8);
                        setPathState(16);
                    }
                }
                break;
            case 16:
                if(!follower.isBusy()){
                    pathTimer.resetTimer();
                    startShooting();
                    setPathState(17);
                }
                break;
            case 17:
                if (!inShoot) {
                    setPathState(18); // Start second cycle
                }
                break;
            // Second cycle of paths 6-8
            case 18: // get gate (second cycle)
                follower.followPath(paths.Path5);
                setPathState(19);
                break;
            case 19: // go around (second cycle)
                if(!follower.isBusy()){
                    if(pathTimer.getElapsedTimeSeconds() > 0.2) {
                        follower.followPath(paths.Path6);
                        intake.setPower(-1);
                        setPathState(20);
                    }
                }
                break;
            case 20: // wiggle (second cycle)
                if(!follower.isBusy()){
                    if(pathTimer.getElapsedTimeSeconds()>1) {
                        follower.followPath(paths.Path7, 0.4, true);
                        setPathState(21);
                    }
                }
                break;
            case 21: // go to shooting spot (second cycle)
                if(!follower.isBusy()){
                    if(pathTimer.getElapsedTimeSeconds()>2) {
                        gate.setPosition(0.4628); // Close gate after second intake
                        intake.setPower(-0.3);
                        shooter.setVelocity(shooterSpeed);
                        follower.followPath(paths.Path8);
                        setPathState(22);
                    }
                }
                break;
            case 22: // shoot (second cycle)
                if(!follower.isBusy()){
                    pathTimer.resetTimer();
                    startShooting();
                    setPathState(23);
                }
                break;
            case 23:
                if (!inShoot) {
                    setPathState(24); // Start final intake and shoot sequence
                }
                break;
            // Final sequence using Path9 and Path10
            case 24: // intake path (similar to path3)
                intake.setPower(-1);
                pathTimer.resetTimer();
                setPathState(25);
                break;
            case 25:
                if (pathTimer.getElapsedTimeSeconds() > 0.1) {
                    follower.followPath(paths.Path9, 0.8, true);
                    setPathState(26);
                }
                break;
            case 26:
                if (!follower.isBusy()) {
                    pathTimer.resetTimer();
                    setPathState(27);
                }
                break;
            case 27:
                if (pathTimer.getElapsedTimeSeconds() > 0.1) {
                    gate.setPosition(0.4628); // Close gate after final intake
                    setPathState(28);
                }
                break;
            case 28: // shoot path (similar to path4)
                follower.followPath(paths.Path10);
                shooter.setVelocity(-1600);
                setPathState(29);
                break;
            case 29:
                if (!follower.isBusy()) {
                    pathTimer.resetTimer();
                    setPathState(30);
                }
                break;
            case 30: // final shoot
                if (pathTimer.getElapsedTimeSeconds() > 0.1) {
                    intake.setPower(0);
                    shooterSpeed = -1800;
                    firstBallSpeed = -1800;
                    startShooting();
                    setPathState(31);
                }
                break;
            case 31:
                if (!inShoot) {
                    setPathState(-1); // End autonomous
                }
                break;
        }
        return pathState;
    }

    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    public void scanForAprilTag() {
        if (limeLightWorking) {
            LLResult result = limelight.getLatestResult();
            if (result != null && result.isValid()) {
                if (result.getFiducialResults() != null && !result.getFiducialResults().isEmpty()) {
                    LLResultTypes.FiducialResult fiducial = result.getFiducialResults().get(0);
                    aprilTagId = (int) fiducial.getFiducialId();
                    aprilTagDetected = true;
                }
            }
        }
    }

    public void determineShotOrder() {
        if (aprilTagDetected) {
            // Mapping for second shooting sequence (balls from Path4 intake)
            switch (aprilTagId) {
                case 21:
                    storedShootingOrder = 0; // Normal order: back->middle->front
                    break;
                case 22:
                    storedShootingOrder = 3; // Custom order: front->back->middle
                    break;
                case 23:
                default:
                    storedShootingOrder = 1; // Reverse order: front->middle->back
                    break;
            }
        } else {
            storedShootingOrder = 0; // Default to normal order if no tag detected
        }

        // First shooting sequence always uses normal order
        if (isFirstShoot) {
            shootingOrder = 0;
        } else {
            shootingOrder = storedShootingOrder;
        }
    }

    public void startShooting() {
        if (!inShoot) {
            inShoot = true;
            shootSequenceTimer.reset();

            // Turn on intake and wheel during shooting


            // Always use normal shooting order for all sequences
            shootingOrder = 0; // Normal order: back->middle->front

            // Set initial state based on shooting order
            switch (shootingOrder) {
                case 0: // Normal: back->middle->front
                    shootSequenceState = 1;
                    break;
                case 1: // Reverse: front->middle->back
                    shootSequenceState = 5;
                    break;
                case 2: // Custom: back->front->middle
                    shootSequenceState = 1;
                    break;
                case 3: // New custom: front->back->middle
                    shootSequenceState = 5;
                    break;
            }
        }
    }

    public void updateShootingSequence() {
        if (inShoot) {
            switch (shootingOrder) {
                case 0: // Normal order: back->middle->front
                    updateNormalShootingSequence();
                    break;
                case 1: // Reverse order: front->middle->back
                    updateReverseShootingSequence();
                    break;
                case 2: // Custom order: back->front->middle
                    updateCustomShootingSequence();
                    break;
                case 3: // New custom order: front->back->middle
                    updateNewCustomShootingSequence();
                    break;
            }
        }
    }

    public void updateNormalShootingSequence() {
        switch (shootSequenceState) {
            case 1: // Move linkage to front position and lift front ball
                linkage.setPosition(0.66);  // Move shooter to front position
                shooter.setVelocity(shooterSpeed);
                if (shootSequenceTimer.milliseconds() > 300) { // Wait for linkage to move
                    front.setPosition(0.4); // Push front ball up
                    shootSequenceTimer.reset();
                    shootSequenceState = 2;
                }
                break;

            case 2: // Wait then reset front servo and move to middle
                if (shootSequenceTimer.milliseconds() > 300) { // Wait for ball to shoot
                    front.setPosition(0); // Reset front servo
                    linkage.setPosition(0.85); // Move shooter to middle position
                    shootSequenceTimer.reset();
                    shootSequenceState = 3;
                }
                break;

            case 3: // Move linkage to middle position and lift middle ball
                if (shootSequenceTimer.milliseconds() > 300) { // Wait for linkage to move
                    middle.setPosition(0.4); // Push middle ball up
                    shootSequenceTimer.reset();
                    shootSequenceState = 4;
                }
                break;

            case 4: // Wait then reset middle servo and move to back
                if (shootSequenceTimer.milliseconds() > 300) { // Wait for ball to shoot
                    middle.setPosition(0.04); // Reset middle servo
                    linkage.setPosition(1.0); // Move shooter to back position
                    shootSequenceTimer.reset();
                    shootSequenceState = 5;
                }
                break;

            case 5: // Move linkage to back position and lift back ball
                if (shootSequenceTimer.milliseconds() > 320) { // Wait for linkage to move
                    back.setPosition(0.78); // Push back ball up
                    shootSequenceTimer.reset();
                    shootSequenceState = 6;
                }
                break;

            case 6: // End sequence
                if (shootSequenceTimer.milliseconds() > 500) { // Wait for ball to shoot
                    back.setPosition(0.35); // Reset back servo
                    // Turn off intake and wheel when shooting ends
                    intake.setPower(0);
                    gate.setPosition(0.69); // Open gate after shooting
                    if (isFirstShoot) {
                        isFirstShoot = false; // Mark first shooting as complete
                        keepShooterWarmed = true; // Start keeping shooter warmed up
                        shooter.setVelocity(-1000); // Keep shooter running at -1000 for warm-up
                    } else {
                        shooter.setPower(0); // Turn off shooter for final sequences
                    }
                    inShoot = false;
                    shootSequenceState = 0;
                }
                break;
        }
    }

    public void updateReverseShootingSequence() {
        switch (shootSequenceState) {
            case 5: // Start with front ball (state 5 for consistency)
                linkage.setPosition(0.0);  // Move shooter to front position
                shooter.setVelocity(firstBallSpeed); // Use slower speed for first ball
                if (shootSequenceTimer.milliseconds() > 1000) { // Longer wait for linkage to move
                    front.setPosition(0.6); // Push front ball up
                    shootSequenceTimer.reset();
                    shootSequenceState = 6;
                }
                break;

            case 6: // Wait then reset front servo and move to middle
                if (shootSequenceTimer.milliseconds() > 800) { // Longer wait for ball to shoot
                    front.setPosition(0); // Reset front servo
                    linkage.setPosition(0.18); // Move shooter to middle position
                    shooter.setVelocity(shooterSpeed); // Switch to normal speed for remaining balls
                    shootSequenceTimer.reset();
                    shootSequenceState = 3;
                }
                break;

            case 3: // Move linkage to middle position and lift middle ball
                if (shootSequenceTimer.milliseconds() > 600) { // Longer wait for linkage to move
                    middle.setPosition(0.6); // Push middle ball up
                    shootSequenceTimer.reset();
                    shootSequenceState = 4;
                }
                break;

            case 4: // Wait then reset middle servo and move to back
                if (shootSequenceTimer.milliseconds() > 800) { // Longer wait for ball to shoot
                    middle.setPosition(0); // Keep middle servo up
                    linkage.setPosition(0.3567); // Move shooter to back position
                    shootSequenceTimer.reset();
                    shootSequenceState = 1;
                }
                break;

            case 1: // Move linkage to back position and lift back ball
                if (shootSequenceTimer.milliseconds() > 600) { // Longer wait for linkage to mov¸Åe
                    back.setPosition(0.6); // Push back ball up
                    shootSequenceTimer.reset();
                    shootSequenceState = 2;
                }
                break;

            case 2: // End sequence
                if (shootSequenceTimer.milliseconds() > 800) { // Longer wait for ball to shoot
                    back.setPosition(0); // Reset back servo
                    // Turn off intake and wheel when shooting ends
                    intake.setPower(0);
                    gate.setPosition(0.69); // Open gate after shooting
                    if (isFirstShoot) {
                        isFirstShoot = false; // Mark first shooting as complete
                        keepShooterWarmed = true; // Start keeping shooter warmed up
                        shooter.setVelocity(-1000); // Keep shooter running at -1000 for warm-up
                    } else {
                        shooter.setPower(0); // Turn off shooter for final sequences
                    }
                    inShoot = false;
                    shootSequenceState = 0;
                }
                break;
        }
    }

    public void updateCustomShootingSequence() {
        switch (shootSequenceState) {
            case 1: // Start with back ball
                linkage.setPosition(0.3567);  // Move shooter to back position
                shooter.setVelocity(firstBallSpeed); // Use slower speed for first ball
                if (shootSequenceTimer.milliseconds() > 1000) { // Longer wait for linkage to move
                    back.setPosition(0.6); // Push back ball up
                    shootSequenceTimer.reset();
                    shootSequenceState = 2;
                }
                break;

            case 2: // Wait then reset back servo and move to front
                if (shootSequenceTimer.milliseconds() > 800) { // Longer wait for ball to shoot
                    back.setPosition(0); // Reset back servo
                    linkage.setPosition(0.0); // Move shooter to front position
                    shooter.setVelocity(shooterSpeed); // Switch to normal speed for remaining balls
                    shootSequenceTimer.reset();
                    shootSequenceState = 5;
                }
                break;

            case 5: // Move linkage to front position and lift front ball
                if (shootSequenceTimer.milliseconds() > 600) { // Longer wait for linkage to move
                    front.setPosition(0.6); // Push front ball up
                    shootSequenceTimer.reset();
                    shootSequenceState = 6;
                }
                break;

            case 6: // Wait then reset front servo and move to middle
                if (shootSequenceTimer.milliseconds() > 800) { // Longer wait for ball to shoot
                    front.setPosition(0); // Reset front servo
                    linkage.setPosition(0.18); // Move shooter to middle position
                    shootSequenceTimer.reset();
                    shootSequenceState = 3;
                }
                break;

            case 3: // Move linkage to middle position and lift middle ball
                if (shootSequenceTimer.milliseconds() > 600) { // Longer wait for linkage to move
                    middle.setPosition(0.6); // Push middle ball up
                    shootSequenceTimer.reset();
                    shootSequenceState = 4;
                }
                break;

            case 4: // End sequence
                if (shootSequenceTimer.milliseconds() > 800) { // Longer wait for ball to shoot
                    middle.setPosition(0); // Keep middle servo up
                    // Turn off intake and wheel when shooting ends
                    intake.setPower(0);
                    gate.setPosition(0.69); // Open gate after shooting
                    if (isFirstShoot) {
                        isFirstShoot = false; // Mark first shooting as complete
                        keepShooterWarmed = true; // Start keeping shooter warmed up
                        shooter.setVelocity(-1000); // Keep shooter running at -1000 for warm-up
                    } else {
                        shooter.setPower(0); // Turn off shooter for final sequences
                    }
                    inShoot = false;
                    shootSequenceState = 0;
                }
                break;
        }
    }

    public void updateNewCustomShootingSequence() {
        switch (shootSequenceState) {
            case 5: // Start with front ball
                linkage.setPosition(0.0);  // Move shooter to front position
                shooter.setVelocity(firstBallSpeed); // Use slower speed for first ball
                if (shootSequenceTimer.milliseconds() > 1000) { // Longer wait for linkage to move
                    front.setPosition(0.6); // Push front ball up
                    shootSequenceTimer.reset();
                    shootSequenceState = 6;
                }
                break;

            case 6: // Wait then reset front servo and move to back
                if (shootSequenceTimer.milliseconds() > 800) { // Longer wait for ball to shoot
                    front.setPosition(0); // Reset front servo
                    linkage.setPosition(0.3567); // Move shooter to back position
                    shooter.setVelocity(shooterSpeed); // Switch to normal speed for remaining balls
                    shootSequenceTimer.reset();
                    shootSequenceState = 1;
                }
                break;

            case 1: // Move linkage to back position and lift back ball
                if (shootSequenceTimer.milliseconds() > 600) { // Longer wait for linkage to move
                    back.setPosition(0.6); // Push back ball up
                    shootSequenceTimer.reset();
                    shootSequenceState = 2;
                }
                break;

            case 2: // Wait then reset back servo and move to middle
                if (shootSequenceTimer.milliseconds() > 800) { // Longer wait for ball to shoot
                    back.setPosition(0); // Reset back servo
                    linkage.setPosition(0.18); // Move shooter to middle position
                    shootSequenceTimer.reset();
                    shootSequenceState = 3;
                }
                break;

            case 3: // Move linkage to middle position and lift middle ball
                if (shootSequenceTimer.milliseconds() > 600) { // Longer wait for linkage to move
                    middle.setPosition(0.6); // Push middle ball up
                    shootSequenceTimer.reset();
                    shootSequenceState = 4;
                }
                break;

            case 4: // End sequence
                if (shootSequenceTimer.milliseconds() > 800) { // Longer wait for ball to shoot
                    middle.setPosition(0); // Keep middle servo up
                    // Turn off intake and wheel when shooting ends
                    intake.setPower(0);
                    gate.setPosition(0.69); // Open gate after shooting
                    if (isFirstShoot) {
                        isFirstShoot = false; // Mark first shooting as complete
                        keepShooterWarmed = true; // Start keeping shooter warmed up
                        shooter.setVelocity(-1000); // Keep shooter running at -1000 for warm-up
                    } else {
                        shooter.setPower(0); // Turn off shooter for final sequences
                    }
                    inShoot = false;
                    shootSequenceState = 0;
                }
                break;
        }
    }
}
