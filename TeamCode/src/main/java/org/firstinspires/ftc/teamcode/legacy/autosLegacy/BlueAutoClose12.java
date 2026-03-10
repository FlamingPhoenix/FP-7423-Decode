package org.firstinspires.ftc.teamcode.legacy.autosLegacy;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.legacy.POSCONFIG_OLD;
import org.firstinspires.ftc.teamcode.legacy.pedroPathingLegacy.ConstantsOLD;
import org.firstinspires.ftc.teamcode.BallDetectionIntakeControl;

//@Autonomous(name = "Blue Auto Close 12", group = "Autonomous")
@Disabled
@Configurable
public class BlueAutoClose12 extends OpMode {

    private TelemetryManager panelsTelemetry;
    public Follower follower;
    private Timer pathTimer;
    private int pathState;
    private Paths paths;

    // Limelight hardware
    Limelight3A limelight;
    boolean limeLightWorking = true;
    int aprilTagId = -1;
    boolean aprilTagDetected = false;
    ElapsedTime aprilTagScanTimer = new ElapsedTime();

    // Shooting hardware
    DcMotorEx shooter;
    Servo front, back, middle, linkage;

    // Intake hardware
    DcMotor intake;
    CRServo wheel;
    boolean inShoot = false;
    int shootSequenceState = 0;
    ElapsedTime shootSequenceTimer = new ElapsedTime();
    double shooterSpeed = -1050;
    double firstBallSpeed = -1050;
    int shootingOrder = 0; // 0 = normal (back->middle->front), 1 = reverse (front->middle->back), 2 = middle->back->front
    int storedShootingOrder = 0; // Store AprilTag order for second shooting sequence
    boolean isFirstShoot = true; // Track if this is the first shooting sequence
    boolean keepShooterWarmed = false; // Keep shooter running at -1000 when not shooting

    // Ball detection system
    BallDetectionIntakeControl ballDetection;

    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
        pathTimer = new Timer();
        shootSequenceTimer = new ElapsedTime();

        follower = ConstantsOLD.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(34.000, 135.719, Math.toRadians(180)));

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

        // Initialize intake hardware
        intake = hardwareMap.get(DcMotor.class, "intake");
        wheel = hardwareMap.crservo.get("wheel");

        // Initialize ball detection system
        ballDetection = new BallDetectionIntakeControl(hardwareMap);

        // Setup shooter
        shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(60, 0, 0.2, 17.2));

        paths = new Paths(follower);

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.debug("Limelight", limeLightWorking ? "Active" : "Inactive");
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void loop() {
        follower.update();
        pathState = autonomousPathUpdate();
        updateShootingSequence();

        // Update ball detection system
        ballDetection.updateBallDetection();

        panelsTelemetry.debug("Path State", pathState);
        panelsTelemetry.debug("X", follower.getPose().getX());
        panelsTelemetry.debug("Y", follower.getPose().getY());
        panelsTelemetry.debug("Heading", Math.toDegrees(follower.getPose().getHeading()));
        panelsTelemetry.debug("Shooting", inShoot ? "Active" : "Inactive");
        panelsTelemetry.debug("Shoot State", shootSequenceState);
        panelsTelemetry.debug("AprilTag ID", aprilTagId);
        panelsTelemetry.debug("Shooting Order", shootingOrder);
        panelsTelemetry.debug("Lock Activated", ballDetection.isLockActivated());
        panelsTelemetry.debug("Outtake Active", ballDetection.isInOuttakeSequence());
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void start() {
        shooter.setVelocity(-1100); // Start shooter with -1100 speed for first ball
        linkage.setPosition(POSCONFIG_OLD.FRONT); // Move linkage to first ball (back) position immediately
        setPathState(0);
    }

    public static class Paths {
        public PathChain Path2;
        public PathChain Path3;
        public PathChain Path4;
        public PathChain Path5;
        public PathChain Path6;
        public PathChain Path7;
        public PathChain Path8;
        public PathChain Path9;
        public PathChain Path10;
        public PathChain Path11;
        public PathChain Path12;

        public Paths(Follower follower) {
            Path2 = follower.pathBuilder().addPath(
                new BezierLine(
                  new Pose(34.000, 135.719),

                  new Pose(38.000, 111.408)
                )
              ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(120))

              .build();

            Path3 = follower.pathBuilder().addPath(
                new BezierCurve(
                  new Pose(38.000, 111.408),
                  new Pose(54.000, 107.453),
                  new Pose(52.252, 90.750)
                )
              ).setLinearHeadingInterpolation(Math.toRadians(120), Math.toRadians(0))

              .build();

//            Path4 = follower.pathBuilder().addPath(
//                new BezierLine(
//                  new Pose(52.252, 90.750),
//
//                  new Pose(21.000, 90.750)
//                )
//              ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
//
//              .build();
            Path4 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(52.252, 90.750),
                                    new Pose(4.745, 94.181),
                                    new Pose(32.97528830313015, 80.42174629324546),
                                    new Pose(18.067, 78.305)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .build();


            Path5 = follower.pathBuilder().addPath(
                new BezierLine(
                        new Pose(18.067, 78.305),

                        new Pose(35.000, 103.000)
                )
              ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(118))

              .build();

            Path6 = follower.pathBuilder().addPath(
                new BezierLine(
                  new Pose(35.000, 103.000),

                  new Pose(46.000, 67.500)
                )
              ).setLinearHeadingInterpolation(Math.toRadians(118), Math.toRadians(0))

              .build();

            Path7 = follower.pathBuilder().addPath(
                new BezierLine(
                  new Pose(46.000, 67.500),

                  new Pose(19.000, 67.500)
                )
              ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))

              .build();

            Path8 = follower.pathBuilder().addPath(
                new BezierCurve(
                  new Pose(19.000, 67.000),
                  new Pose(42.000, 78.155),
                  new Pose(35.000, 103.000)
                )
              ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(118))

              .build();

            Path9 = follower.pathBuilder().addPath(
                new BezierLine(
                  new Pose(35.000, 103.000),

                  new Pose(46.000, 44.000)
                )
              ).setLinearHeadingInterpolation(Math.toRadians(131), Math.toRadians(0))

              .build();

            Path10 = follower.pathBuilder().addPath(
                new BezierLine(
                  new Pose(46.000, 44.000),

                  new Pose(19.000, 44.000)
                )
              ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))

              .build();

            Path11 = follower.pathBuilder().addPath(
                new BezierLine(
                  new Pose(19.000, 44.000),

                  new Pose(38.720, 98.842)
                )
              ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(118))

              .build();

            Path12 = follower.pathBuilder().addPath(
                new BezierLine(
                  new Pose(38.720, 98.842),

                  new Pose(29, 82.750)
                )
              ).setLinearHeadingInterpolation(Math.toRadians(118), Math.toRadians(180))

              .build();
        }
    }

    public int autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(paths.Path2);
                setPathState(1);
                break;
            case 1:
                if (!follower.isBusy()) {
                    // Start shooting sequence after Path2 is complete
                    startShooting();
                    setPathState(2);
                }
                break;
            case 2:
                // Wait for shooting to complete
                if (!inShoot) {
                    setPathState(3);
                }
                break;
            case 3:
                follower.followPath(paths.Path3);
                setPathState(4);
                break;
            case 4:
                if (!follower.isBusy()) {
                    // Turn on intake after Path3 completion using ball detection system
                    ballDetection.runNormalIntake();
                    pathTimer.resetTimer();
                    setPathState(5);
                }
                break;
            case 5:
                // Brief pause before starting Path4
                if (pathTimer.getElapsedTimeSeconds() > 0.1) {
                    follower.followPath(paths.Path4, 0.6, true);
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
                // Half second pause after Path4
                if (pathTimer.getElapsedTimeSeconds() > 0.1) {
                    setPathState(8);
                }
                break;
            case 8:
                follower.followPath(paths.Path5);
                // Start charging up shooter while moving to shooting position
                shooter.setVelocity(-1050);
                setPathState(9);
                break;
            case 9:
                if (!follower.isBusy()) {
                    pathTimer.resetTimer();
                    setPathState(10);
                }
                break;
            case 10:
                // 500ms wait after Path5
                if (pathTimer.getElapsedTimeSeconds() > 0.1) {
                    // Stop intake using ball detection system
                    ballDetection.stopIntake();
                    // Override shooter speeds for sequence
                    shooterSpeed = -1050;
                    firstBallSpeed = -1050;
                    startShooting(); // Trigger shooting sequence
                    setPathState(11);
                }
                break;
            case 11:
                // Wait for shooting to complete
                if (!inShoot) {
                    setPathState(12); // Continue to new path sequence
                }
                break;
            case 12:
                follower.followPath(paths.Path6);
                setPathState(13);
                break;
            case 13:
                if (!follower.isBusy()) {
                    // Turn on intake after Path6 completion using ball detection system
                    ballDetection.runNormalIntake();
                    pathTimer.resetTimer();
                    setPathState(14);
                }
                break;
            case 14:
                // Brief pause before starting Path7
                if (pathTimer.getElapsedTimeSeconds() > 0.1) {
                    follower.followPath(paths.Path7, 0.6, true);
                    setPathState(15);
                }
                break;
            case 15:
                if (!follower.isBusy()) {
                    pathTimer.resetTimer();
                    setPathState(16);
                }
                break;
            case 16:
                // Hold intake for 500ms
                if (pathTimer.getElapsedTimeSeconds() > 0.5) {
                    setPathState(17);
                }
                break;
            case 17:
                follower.followPath(paths.Path8);
                // Start charging up shooter while moving to shooting position
                shooter.setVelocity(-1050);
                setPathState(18);
                break;
            case 18:
                if (!follower.isBusy()) {
                    pathTimer.resetTimer();
                    setPathState(19);
                }
                break;
            case 19:
                // 100ms wait after Path8
                if (pathTimer.getElapsedTimeSeconds() > 0.1) {
                    // Stop intake using ball detection system
                    ballDetection.stopIntake();
                    // Override shooter speeds for this sequence
                    shooterSpeed = -1050;
                    firstBallSpeed = -1050;
                    startShooting(); // Trigger shooting sequence
                    setPathState(20);
                }
                break;
            case 20:
                // Wait for shooting to complete
                if (!inShoot) {
                    setPathState(21); // Continue to next sequence
                }
                break;
            case 21:
                follower.followPath(paths.Path9);
                setPathState(22);
                break;
            case 22:
                if (!follower.isBusy()) {
                    // Turn on intake after Path9 completion using ball detection system
                    ballDetection.runNormalIntake();
                    pathTimer.resetTimer();
                    setPathState(23);
                }
                break;
            case 23:
                // Brief pause before starting Path10
                if (pathTimer.getElapsedTimeSeconds() > 0.1) {
                    follower.followPath(paths.Path10, 0.6, true);
                    setPathState(24);
                }
                break;
            case 24:
                if (!follower.isBusy()) {
                    pathTimer.resetTimer();
                    setPathState(25);
                }
                break;
            case 25:
                // Hold intake for 500ms
                if (pathTimer.getElapsedTimeSeconds() > 0.1) {
                    setPathState(26);
                }
                break;
            case 26:
                follower.followPath(paths.Path11);
                // Start charging up shooter while moving to shooting position
                shooter.setVelocity(-1050);
                setPathState(27);
                break;
            case 27:
                if (!follower.isBusy()) {
                    pathTimer.resetTimer();
                    setPathState(28);
                }
                break;
            case 28:
                // 100ms wait after Path11
                if (pathTimer.getElapsedTimeSeconds() > 0.1) {
                    // Stop intake using ball detection system for final sequence
                    ballDetection.stopIntake();
                    // Use same speeds for final sequence
                    shooterSpeed = -1050;
                    firstBallSpeed = -1050;
                    startShooting(); // Trigger final shooting sequence
                    setPathState(29);
                }
                break;
            case 29:
                // Wait for final shooting to complete
                if (!inShoot) {
                    setPathState(30); // Continue to Path12
                }
                break;
            case 30:
                follower.followPath(paths.Path12);
                setPathState(31);
                break;
            case 31:
                if (!follower.isBusy()) {
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

            // Turn on intake and wheel during shooting using ball detection system
            ballDetection.runNormalIntake();

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
            case 1: // Move linkage to back position and lift back ball
                linkage.setPosition(POSCONFIG_OLD.FRONT);  // Move shooter to back position
                shooter.setVelocity(shooterSpeed);
                if (shootSequenceTimer.milliseconds() > 300) { // Wait for linkage to move
                    back.setPosition(0.6); // Push back ball up
                    shootSequenceTimer.reset();
                    shootSequenceState = 2;
                }
                break;

            case 2: // Wait then reset back servo and move to middle
                if (shootSequenceTimer.milliseconds() > 400) { // Wait for ball to shoot
                    back.setPosition(0); // Reset back servo
                    linkage.setPosition(POSCONFIG_OLD.MIDDLE); // Move shooter to middle position
                    shootSequenceTimer.reset();
                    shootSequenceState = 3;
                }
                break;

            case 3: // Move linkage to middle position and lift middle ball
                if (shootSequenceTimer.milliseconds() > 300) { // Wait for linkage to move
                    middle.setPosition(0.7); // Push middle ball up
                    shootSequenceTimer.reset();
                    shootSequenceState = 4;
                }
                break;

            case 4: // Wait then reset middle servo and move to front
                if (shootSequenceTimer.milliseconds() > 400) { // Wait for ball to shoot
                    middle.setPosition(0); // Reset middle servo
                    linkage.setPosition(POSCONFIG_OLD.BACK); // Move shooter to front position
                    shootSequenceTimer.reset();
                    shootSequenceState = 5;
                }
                break;

            case 5: // Move linkage to front position and lift front ball
                if (shootSequenceTimer.milliseconds() > 300) { // Wait for linkage to move
                    front.setPosition(0.6); // Push front ball up
                    shootSequenceTimer.reset();
                    shootSequenceState = 6;
                }
                break;

            case 6: // End sequence
                if (shootSequenceTimer.milliseconds() > 500) { // Wait for ball to shoot
                    front.setPosition(0); // Reset front servo
                    // Turn off intake and wheel when shooting ends using ball detection system
                    ballDetection.stopIntake();
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
                linkage.setPosition(POSCONFIG_OLD.BACK);  // Move shooter to front position
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
                    linkage.setPosition(POSCONFIG_OLD.MIDDLE);// Move shooter to middle position
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
                    linkage.setPosition(POSCONFIG_OLD.FRONT); // Move shooter to back position
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

            case 2: // End sequence
                if (shootSequenceTimer.milliseconds() > 800) { // Longer wait for ball to shoot
                    back.setPosition(0); // Reset back servo
                    // Turn off intake and wheel when shooting ends using ball detection system
                    ballDetection.stopIntake();
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
                linkage.setPosition(POSCONFIG_OLD.FRONT);  // Move shooter to back position
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
                    linkage.setPosition(POSCONFIG_OLD.BACK); // Move shooter to front position
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
                    linkage.setPosition(POSCONFIG_OLD.MIDDLE); // Move shooter to middle position
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
                    // Turn off intake and wheel when shooting ends using ball detection system
                    ballDetection.stopIntake();
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
                linkage.setPosition(POSCONFIG_OLD.BACK);  // Move shooter to front position
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
                    linkage.setPosition(POSCONFIG_OLD.FRONT); // Move shooter to back position
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
                    linkage.setPosition(POSCONFIG_OLD.MIDDLE); // Move shooter to middle position
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
                    // Turn off intake and wheel when shooting ends using ball detection system
                    ballDetection.stopIntake();
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