package org.firstinspires.ftc.teamcode.auto;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
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
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "Pedro Pathing Autonomous", group = "Autonomous")
@Configurable // Panels
public class AutoTest extends OpMode {

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
    Servo front, back, middle, linkage;

    // Intake hardware
    DcMotor intake;
    CRServo wheel;
    boolean inShoot = false;
    int shootSequenceState = 0;
    ElapsedTime shootSequenceTimer = new ElapsedTime();
    double shooterSpeed = -1150;
    double firstBallSpeed = -1100; // Slower speed for first ball
    int shootingOrder = 0; // 0 = normal (back->middle->front), 1 = reverse (front->middle->back), 2 = middle->back->front

    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
        pathTimer = new Timer();
        shootSequenceTimer = new ElapsedTime();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(85.709, 135.502, Math.toRadians(90)));

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
        shooter.setVelocity(firstBallSpeed); // Start shooter with slower speed for first ball
        linkage.setPosition(0.3567); // Move linkage to first ball (back) position immediately
        setPathState(0);
    }

    public static class Paths {

        public PathChain Path1;
        public PathChain Path2;
        public PathChain Path3;
        public PathChain Path4;
        public PathChain Path5;

        public Paths(Follower follower) {
            Path1 = follower.pathBuilder().addPath(
                new BezierLine(
                  new Pose(85.709, 135.502),

                  new Pose(85.709, 100.685)
                )
              ).setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(90))

              .build();

            Path2 = follower.pathBuilder().addPath(
                new BezierLine(
                  new Pose(85.709, 100.685),

                  new Pose(85.709, 101.685)
                )
              ).setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(32))

              .build();

            Path3 = follower.pathBuilder().addPath(
                new BezierLine(
                  new Pose(85.709, 101.685),

                  new Pose(98, 81.500)
                )
              ).setLinearHeadingInterpolation(Math.toRadians(32), Math.toRadians(180))

              .build();

            Path4 = follower.pathBuilder().addPath(
                new BezierLine(
                  new Pose(98, 81.500),

                  new Pose(122.000, 81.500)
                )
              ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
              .build();

            Path5 = follower.pathBuilder().addPath(
                new BezierLine(
                  new Pose(122.000, 81.500),

                  new Pose(109, 103)
                )
              ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(47))
              .build();
        }
    }


    public int autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(paths.Path1);
                setPathState(1);
                break;
            case 1:
                if (!follower.isBusy()) {
                    // Switch to AprilTag pipeline and start scanning
                    if (limeLightWorking) {
                        limelight.pipelineSwitch(2);
                        aprilTagScanTimer.reset();
                    }
                    setPathState(2);
                }
                break;
            case 2:
                // AprilTag scanning phase
                if (limeLightWorking) {
                    scanForAprilTag();
                    // Wait for AprilTag detection or timeout
                    if (aprilTagDetected || aprilTagScanTimer.milliseconds() > 2000) {
                        determineShotOrder();
                        limelight.pipelineSwitch(0); // Switch back to shooting pipeline
                        setPathState(3);
                    }
                } else {
                    // If Limelight not working, use default shooting order
                    shootingOrder = 0;
                    setPathState(3);
                }
                break;
            case 3:
                follower.followPath(paths.Path2);
                setPathState(4);
                break;
            case 4:
                if (!follower.isBusy()) {
                    // Start shooting sequence after Path2 is complete
                    startShooting();
                    setPathState(5);
                }
                break;
            case 5:
                // Wait for shooting to complete
                if (!inShoot) {
                    setPathState(6);
                }
                break;
            case 6:
                follower.followPath(paths.Path3);
                setPathState(7);
                break;
            case 7:
                if (!follower.isBusy()) {
                    // Turn on intake after Path3 completion
                    intake.setPower(-0.9);
                    wheel.setPower(1);
                    pathTimer.resetTimer();
                    setPathState(8);
                }
                break;
            case 8:
                // Brief pause before starting Path4
                if (pathTimer.getElapsedTimeSeconds() > 0.5) {
                    follower.followPath(paths.Path4, 0.3, true);
                    setPathState(9);
                }
                break;
            case 9:
                if (!follower.isBusy()) {
                    pathTimer.resetTimer();
                    setPathState(10);
                }
                break;
            case 10:
                // Hold intake for 500ms at the end
                if (pathTimer.getElapsedTimeSeconds() > 0.5) {
                    setPathState(11);
                }
                break;
            case 11:
                follower.followPath(paths.Path5);
                setPathState(12);
                break;
            case 12:
                if (!follower.isBusy()) {
                    pathTimer.resetTimer();
                    setPathState(13);
                }
                break;
            case 13:
                // 500ms wait after Path5
                if (pathTimer.getElapsedTimeSeconds() > 0.5) {
                    // Stop intake motor but keep wheel running for final sequence
                    intake.setPower(0);
                    // Override shooter speeds for final sequence
                    shooterSpeed = -1050;
                    firstBallSpeed = -1050;
                    startShooting(); // Trigger normal shooting sequence
                    setPathState(14);
                }
                break;
            case 14:
                // Wait for shooting to complete
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
            switch (aprilTagId) {
                case 21:
                    shootingOrder = 1; // Reverse order: front->middle->back
                    break;
                case 22:
                    shootingOrder = 2; // Custom order: back->front->middle
                    break;
                case 23:
                default:
                    shootingOrder = 0; // Normal order: back->middle->front
                    break;
            }
        } else {
            shootingOrder = 0; // Default to normal order if no tag detected
        }
    }

    public void startShooting() {
        if (!inShoot) {
            inShoot = true;
            shootSequenceTimer.reset();

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
                case 2: // Custom order: middle->back->front
                    updateCustomShootingSequence();
                    break;
            }
        }
    }

    public void updateNormalShootingSequence() {
        switch (shootSequenceState) {
            case 1: // Move linkage to back position and lift back ball
                linkage.setPosition(0.3567);  // Move shooter to back position
                shooter.setVelocity(firstBallSpeed); // Use slower speed for first ball
                if (shootSequenceTimer.milliseconds() > 700) { // Wait for linkage to move
                    back.setPosition(0.6); // Push back ball up
                    shootSequenceTimer.reset();
                    shootSequenceState = 2;
                }
                break;

            case 2: // Wait then reset back servo and move to middle
                if (shootSequenceTimer.milliseconds() > 500) { // Wait for ball to shoot
                    back.setPosition(0); // Reset back servo
                    linkage.setPosition(0.18); // Move shooter to middle position
                    shooter.setVelocity(shooterSpeed); // Switch to normal speed for remaining balls
                    shootSequenceTimer.reset();
                    shootSequenceState = 3;
                }
                break;

            case 3: // Move linkage to middle position and lift middle ball
                if (shootSequenceTimer.milliseconds() > 300) { // Wait for linkage to move
                    middle.setPosition(0.6); // Push middle ball up
                    shootSequenceTimer.reset();
                    shootSequenceState = 4;
                }
                break;

            case 4: // Wait then reset middle servo and move to front
                if (shootSequenceTimer.milliseconds() > 500) { // Wait for ball to shoot
                    middle.setPosition(0); // Keep middle servo up
                    wheel.setPower(0); // Stop wheel after second ball
                    linkage.setPosition(0.0); // Move shooter to front position
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
                    shooter.setPower(0);
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
                if (shootSequenceTimer.milliseconds() > 600) { // Longer wait for linkage to move
                    back.setPosition(0.6); // Push back ball up
                    shootSequenceTimer.reset();
                    shootSequenceState = 2;
                }
                break;

            case 2: // End sequence
                if (shootSequenceTimer.milliseconds() > 800) { // Longer wait for ball to shoot
                    back.setPosition(0); // Reset back servo
                    shooter.setPower(0);
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
                    shooter.setPower(0);
                    inShoot = false;
                    shootSequenceState = 0;
                }
                break;
        }
    }
}
