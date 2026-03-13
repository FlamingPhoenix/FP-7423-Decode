package org.firstinspires.ftc.teamcode.autos;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "Red Auto Far NEW", group = "Autonomous")
@Configurable
public class redFarAuto extends OpMode {

    private TelemetryManager panelsTelemetry;
    public Follower follower;
    private Timer pathTimer;
    private int pathState;
    private Paths paths;

    // Shooting hardware
    DcMotorEx shooter;
    Servo front, back, middle, linkage, gate;

    // Intake hardware
    DcMotor intake;
    boolean inShoot = false;
    int shootSequenceState = 0;
    ElapsedTime shootSequenceTimer = new ElapsedTime();
    double shooterSpeed = -2200;
    double firstBallSpeed = -2200;
    int shootingOrder = 0;
    boolean isFirstShoot = true;
    boolean keepShooterWarmed = false;

    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
        pathTimer = new Timer();
        shootSequenceTimer = new ElapsedTime();
        Constants.driveConstants.setUseBrakeModeInTeleOp(true);
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(81.191, 8.973, Math.toRadians(90)));

        // Initialize shooting hardware
        shooter = hardwareMap.get(DcMotorEx.class, "shooter");
        front = hardwareMap.servo.get("front");
        back = hardwareMap.servo.get("back");
        middle = hardwareMap.servo.get("middle");
        linkage = hardwareMap.servo.get("linkage");
        gate = hardwareMap.servo.get("lock");

        // Initialize intake hardware
        intake = hardwareMap.get(DcMotor.class, "intake");

        // Setup shooter with PID from NewTeleOpTest
        shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(60, 0, 0.2, 15));

        paths = new Paths(follower);

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void loop() {
        follower.update();
        pathState = autonomousPathUpdate();
        updateShootingSequence();

        panelsTelemetry.debug("Path State", pathState);
        panelsTelemetry.debug("X", follower.getPose().getX());
        panelsTelemetry.debug("Y", follower.getPose().getY());
        panelsTelemetry.debug("Heading", follower.getPose().getHeading());
        panelsTelemetry.debug("Shooting", inShoot ? "Active" : "Inactive");
        panelsTelemetry.debug("Shoot State", shootSequenceState);
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void start() {
        // Initialize all motors to safe states
        intake.setPower(0);
        shooter.setVelocity(firstBallSpeed);

        // Initialize servo positions to safe states
        linkage.setPosition(0.66);
        front.setPosition(0);
        back.setPosition(0.35);
        middle.setPosition(0.04);
        gate.setPosition(0.69);

        setPathState(0);
    }

    public static class Paths {
        public PathChain Path1;
        public PathChain Path2;
        public PathChain Path3;
        public PathChain Path4;
        public PathChain Path5;
        public PathChain Path6;
        public PathChain Path7;
        public PathChain Path8;
        public PathChain Path9;

        public Paths(Follower follower) {
            Path1 = follower.pathBuilder()
                .addPath(
                    new BezierLine(
                        new Pose(81.191, 8.973),
                        new Pose(84.012, 22.650)
                    )
                )
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(66))
                .build();

            Path2 = follower.pathBuilder()
                .addPath(
                    new BezierLine(
                        new Pose(84.012, 22.650),
                        new Pose(97.882, 36.000)
                    )
                )
                .setLinearHeadingInterpolation(Math.toRadians(66), Math.toRadians(0))
                .build();

            Path3 = follower.pathBuilder()
                .addPath(
                    new BezierLine(
                        new Pose(97.882, 36.000),
                        new Pose(131.469, 36.000)
                    )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

            Path4 = follower.pathBuilder()
                .addPath(
                    new BezierLine(
                        new Pose(131.469, 36.000),
                        new Pose(83.687, 22.760)
                    )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(66))
                .build();

            Path5 = follower.pathBuilder()
                .addPath(
                    new BezierCurve(
                        new Pose(83.687, 22.760),
                        new Pose(105.876, 21.382),
                        new Pose(123.293, 10.000)
                    )
                )
                .setLinearHeadingInterpolation(Math.toRadians(66), Math.toRadians(0))
                .build();

            Path6 = follower.pathBuilder()
                .addPath(
                    new BezierLine(
                        new Pose(123.293, 10.000),
                        new Pose(135.006, 10.000)
                    )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .setTimeoutConstraint(1500) // 2 second timeout
                .build();

            Path7 = follower.pathBuilder()
                .addPath(
                    new BezierLine(
                        new Pose(135.006, 10.000),
                        new Pose(123.434, 9.954)
                    )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))

                .build();

            Path8 = follower.pathBuilder()
                .addPath(
                    new BezierLine(
                        new Pose(123.434, 9.954),
                        new Pose(135.061, 9.895)
                    )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .setTimeoutConstraint(1500) // 2 second timeout
                .build();

            Path9 = follower.pathBuilder()
                .addPath(
                    new BezierLine(
                        new Pose(135.061, 9.895),
                        new Pose(83.867, 22.699)
                    )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(66))
                .build();
        }
    }

    public int autonomousPathUpdate() {
        switch (pathState) {
            case 0: // Path1 - go to shooting position
                follower.followPath(paths.Path1);
                setPathState(1);
                break;
            case 1: // Wait for Path1 to complete, then shoot
                if (follower.getCurrentTValue() > 0.8) {
                    startShooting();
                    setPathState(2);
                }
                break;
            case 2: // Wait for shooting to complete
                if (!inShoot) {
                    setPathState(3);
                }
                break;
            case 3: // Path2 - first part of intake path
                follower.followPath(paths.Path2);
                setPathState(4);
                break;
            case 4: // Wait for Path2 to complete
                if (!follower.isBusy()) {
                    setPathState(5);
                }
                break;
            case 5: // Path3 - second part of intake path
                follower.followPath(paths.Path3, 0.3, true);
                intake.setPower(-1);
                setPathState(6);
                break;
            case 6: // Wait for Path3 to complete, then close gate
                if (!follower.isBusy()) {
                    pathTimer.resetTimer();
                    setPathState(7);
                }
                break;
            case 7: // Wait briefly, then close gate
                if (pathTimer.getElapsedTimeSeconds() > 0.1) {
                    gate.setPosition(0.4628);
                    setPathState(8);
                }
                break;
            case 8: // Path4 - go to shooting position
                follower.followPath(paths.Path4);
                shooter.setVelocity(-2200);
                setPathState(9);
                break;
            case 9: // Wait for Path4 to complete, then shoot
                if (!follower.isBusy()) {
                    pathTimer.resetTimer();
                    setPathState(10);
                }
                break;
            case 10: // Start shooting
                if (pathTimer.getElapsedTimeSeconds() > 0.1) {
                    intake.setPower(0);
                    shooterSpeed = -2200;
                    firstBallSpeed = -2200;
                    startShooting();
                    setPathState(11);
                }
                break;
            case 11: // Wait for shooting to complete
                if (!inShoot) {
                    setPathState(12);
                }
                break;
            case 12: // Path5 - start constant intake sequence
                follower.followPath(paths.Path5);
                intake.setPower(-1); // Start constant intake
                setPathState(13);
                break;
            case 13: // Wait for Path5 to complete
                if (!follower.isBusy()) {
                    setPathState(14);
                }
                break;
            case 14: // Path6 - continue constant intake
                follower.followPath(paths.Path6, 0.4, true);
                // Keep intake running at -1
                setPathState(15);
                break;
            case 15: // Wait for Path6 to complete
                if (!follower.isBusy()) {
                    setPathState(16);
                }
                break;
            case 16: // Path7 - continue constant intake
                follower.followPath(paths.Path7);
                // Keep intake running at -1
                setPathState(17);
                break;
            case 17: // Wait for Path7 to complete
                if (!follower.isBusy()) {
                    setPathState(18);
                }
                break;
            case 18: // Path8 - final constant intake
                follower.followPath(paths.Path8, 0.4, true);
                // Keep intake running at -1
                setPathState(19);
                break;
            case 19: // Wait for Path8 to complete, then close gate
                if (!follower.isBusy()) {
                    pathTimer.resetTimer();
                    setPathState(20);
                }
                break;
            case 20: // Close gate after constant intake sequence
                if (pathTimer.getElapsedTimeSeconds() > 0.1) {
                    gate.setPosition(0.4628);
                    intake.setPower(0); // Stop intake after closing gate
                    setPathState(21);
                }
                break;
            case 21: // Path9 - move to shooting position
                follower.followPath(paths.Path9);
                shooter.setVelocity(shooterSpeed);
                setPathState(22);
                break;
            case 22: // Wait for Path9 to complete
                if (!follower.isBusy()) {
                    pathTimer.resetTimer();
                    setPathState(23);
                }
                break;
            case 23: // Final shooting
                if (pathTimer.getElapsedTimeSeconds() > 0.1) {
                    startShooting();
                    setPathState(24);
                }
                break;
            case 24: // Wait for final shooting to complete
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

    public void startShooting() {
        if (!inShoot) {
            inShoot = true;
            shootSequenceTimer.reset();
            shootingOrder = 0; // Normal order: back->middle->front
            shootSequenceState = 1;
        }
    }

    public void updateShootingSequence() {
        if (inShoot) {
            updateNormalShootingSequence();
        }
    }

    public void updateNormalShootingSequence() {
        switch (shootSequenceState) {
            case 1: // Move linkage to front position and lift front ball
                linkage.setPosition(0.66);
                shooter.setVelocity(shooterSpeed);
                if (shootSequenceTimer.milliseconds() > 1000) {
                    front.setPosition(0.4);
                    shootSequenceTimer.reset();
                    shootSequenceState = 2;
                }
                break;

            case 2: // Wait then reset front servo and move to middle
                if (shootSequenceTimer.milliseconds() > 300) {
                    front.setPosition(0);
                    linkage.setPosition(0.85);
                    shootSequenceTimer.reset();
                    shootSequenceState = 3;
                }
                break;

            case 3: // Move linkage to middle position and lift middle ball
                if (shootSequenceTimer.milliseconds() > 500) {
                    middle.setPosition(0.4);
                    shootSequenceTimer.reset();
                    shootSequenceState = 4;
                }
                break;

            case 4: // Wait then reset middle servo and move to back
                if (shootSequenceTimer.milliseconds() > 300) {
                    middle.setPosition(0.04);
                    linkage.setPosition(1.0);
                    shootSequenceTimer.reset();
                    shootSequenceState = 5;
                }
                break;

            case 5: // Move linkage to back position and lift back ball
                if (shootSequenceTimer.milliseconds() > 520) {
                    back.setPosition(0.78);
                    shootSequenceTimer.reset();
                    shootSequenceState = 6;
                }
                break;

            case 6: // End sequence
                if (shootSequenceTimer.milliseconds() > 500) {
                    back.setPosition(0.35);
                    intake.setPower(0);
                    gate.setPosition(0.69);
                    if (isFirstShoot) {
                        isFirstShoot = false;
                        keepShooterWarmed = true;
                        shooter.setVelocity(-1000);
                    } else {
                        shooter.setPower(0);
                    }
                    inShoot = false;
                    shootSequenceState = 0;
                }
                break;
        }
    }
}