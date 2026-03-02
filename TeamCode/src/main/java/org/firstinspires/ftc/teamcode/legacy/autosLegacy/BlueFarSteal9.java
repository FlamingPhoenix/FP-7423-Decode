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
import com.qualcomm.hardware.limelightvision.Limelight3A;
import org.firstinspires.ftc.teamcode.legacy.pedroPathingLegacy.ConstantsOLD;

//@Autonomous(name = "Blue Far Steal 9", group = "Autonomous")
@Configurable
public class BlueFarSteal9 extends OpMode {

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
    int shootingOrder = 0;
    int storedShootingOrder = 0;
    boolean isFirstShoot = true;
    boolean keepShooterWarmed = false;

    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
        pathTimer = new Timer();
        shootSequenceTimer = new ElapsedTime();

        follower = ConstantsOLD.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(56.000, 8.000, Math.toRadians(90)));

        // Initialize Limelight
        try {
            limelight = hardwareMap.get(Limelight3A.class, "limelight");
            limelight.setPollRateHz(100);
            limelight.start();
            limelight.pipelineSwitch(0);
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

        panelsTelemetry.debug("Path State", pathState);
        panelsTelemetry.debug("X", follower.getPose().getX());
        panelsTelemetry.debug("Y", follower.getPose().getY());
        panelsTelemetry.debug("Heading", Math.toDegrees(follower.getPose().getHeading()));
        panelsTelemetry.debug("Shooting", inShoot ? "Active" : "Inactive");
        panelsTelemetry.debug("Shoot State", shootSequenceState);
        panelsTelemetry.debug("AprilTag ID", aprilTagId);
        panelsTelemetry.debug("Shooting Order", shootingOrder);
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void start() {
        shooter.setVelocity(-1100);
        linkage.setPosition(0.3567);
        setPathState(0);
    }

    public static class Paths {
        public PathChain Path1;
        public PathChain Path2;
        public PathChain Path5;
        public PathChain Path3;
        public PathChain Path4;
        public PathChain Path6;

        public Paths(Follower follower) {
            Path1 = follower.pathBuilder().addPath(
                new BezierLine(
                  new Pose(56.000, 8.000),

                  new Pose(53.733, 8.662)
                )
              ).setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(105))

              .build();

            Path2 = follower.pathBuilder().addPath(
                new BezierCurve(
                  new Pose(53.733, 8.662),
                  new Pose(52.304, 56.681),
                  new Pose(98.261, 57.500)
                )
              ).setLinearHeadingInterpolation(Math.toRadians(105), Math.toRadians(180))

              .build();

            Path5 = follower.pathBuilder().addPath(
                new BezierLine(
                  new Pose(98.261, 57.500),

                  new Pose(125.691, 57.719)
                )
              ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))

              .build();

            Path3 = follower.pathBuilder().addPath(
                new BezierCurve(
                  new Pose(125.691, 57.719),
                  new Pose(77.051, 43.247),
                  new Pose(73.024, 75.012)
                )
              ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(135))

              .build();

            Path4 = follower.pathBuilder().addPath(
                new BezierCurve(
                  new Pose(73.024, 75.012),
                  new Pose(86.983, 11.191),
                  new Pose(13.964, 11.334)
                )
              ).setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(0))

              .build();

            Path6 = follower.pathBuilder().addPath(
                new BezierLine(
                  new Pose(13.964, 11.334),

                  new Pose(53.772, 8.717)
                )
              ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(105))

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
                    intake.setPower(-0.9);
                    wheel.setPower(1);
                    pathTimer.resetTimer();
                    setPathState(5);
                }
                break;
            case 5:
                if (pathTimer.getElapsedTimeSeconds() > 0.1) {
                    follower.followPath(paths.Path5, 0.6, true);
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
                    setPathState(8);
                }
                break;
            case 8:
                follower.followPath(paths.Path3);
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
                if (pathTimer.getElapsedTimeSeconds() > 0.1) {
                    intake.setPower(0);
                    shooterSpeed = -1050;
                    firstBallSpeed = -1050;
                    startShooting();
                    setPathState(11);
                }
                break;
            case 11:
                if (!inShoot) {
                    setPathState(12);
                }
                break;
            case 12:
                follower.followPath(paths.Path4);
                setPathState(13);
                break;
            case 13:
                if (!follower.isBusy()) {
                    intake.setPower(-0.9);
                    wheel.setPower(1);
                    pathTimer.resetTimer();
                    setPathState(14);
                }
                break;
            case 14:
                if (pathTimer.getElapsedTimeSeconds() > 0.1) {
                    follower.followPath(paths.Path6, 0.6, true);
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
                if (pathTimer.getElapsedTimeSeconds() > 0.5) {
                    setPathState(17);
                }
                break;
            case 17:
                intake.setPower(0);
                shooterSpeed = -1050;
                firstBallSpeed = -1050;
                startShooting();
                setPathState(18);
                break;
            case 18:
                if (!inShoot) {
                    setPathState(-1);
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

            intake.setPower(-0.9);
            wheel.setPower(1);

            shootingOrder = 0;

            switch (shootingOrder) {
                case 0:
                    shootSequenceState = 1;
                    break;
                case 1:
                    shootSequenceState = 5;
                    break;
                case 2:
                    shootSequenceState = 1;
                    break;
                case 3:
                    shootSequenceState = 5;
                    break;
            }
        }
    }

    public void updateShootingSequence() {
        if (inShoot) {
            switch (shootingOrder) {
                case 0:
                    updateNormalShootingSequence();
                    break;
                case 1:
                    updateReverseShootingSequence();
                    break;
                case 2:
                    updateCustomShootingSequence();
                    break;
                case 3:
                    updateNewCustomShootingSequence();
                    break;
            }
        }
    }

    public void updateNormalShootingSequence() {
        switch (shootSequenceState) {
            case 1:
                linkage.setPosition(0.3567);
                shooter.setVelocity(shooterSpeed);
                if (shootSequenceTimer.milliseconds() > 400) {
                    back.setPosition(0.6);
                    shootSequenceTimer.reset();
                    shootSequenceState = 2;
                }
                break;

            case 2:
                if (shootSequenceTimer.milliseconds() > 300) {
                    back.setPosition(0);
                    linkage.setPosition(0.18);
                    shootSequenceTimer.reset();
                    shootSequenceState = 3;
                }
                break;

            case 3:
                if (shootSequenceTimer.milliseconds() > 400) {
                    middle.setPosition(0.7);
                    shootSequenceTimer.reset();
                    shootSequenceState = 4;
                }
                break;

            case 4:
                if (shootSequenceTimer.milliseconds() > 300) {
                    middle.setPosition(0);
                    linkage.setPosition(0.0);
                    shootSequenceTimer.reset();
                    shootSequenceState = 5;
                }
                break;

            case 5:
                if (shootSequenceTimer.milliseconds() > 400) {
                    front.setPosition(0.6);
                    shootSequenceTimer.reset();
                    shootSequenceState = 6;
                }
                break;

            case 6:
                if (shootSequenceTimer.milliseconds() > 500) {
                    front.setPosition(0);
                    intake.setPower(0);
                    wheel.setPower(0);
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

    public void updateReverseShootingSequence() {
        switch (shootSequenceState) {
            case 5:
                linkage.setPosition(0.0);
                shooter.setVelocity(firstBallSpeed);
                if (shootSequenceTimer.milliseconds() > 1000) {
                    front.setPosition(0.6);
                    shootSequenceTimer.reset();
                    shootSequenceState = 6;
                }
                break;

            case 6:
                if (shootSequenceTimer.milliseconds() > 800) {
                    front.setPosition(0);
                    linkage.setPosition(0.18);
                    shooter.setVelocity(shooterSpeed);
                    shootSequenceTimer.reset();
                    shootSequenceState = 3;
                }
                break;

            case 3:
                if (shootSequenceTimer.milliseconds() > 600) {
                    middle.setPosition(0.6);
                    shootSequenceTimer.reset();
                    shootSequenceState = 4;
                }
                break;

            case 4:
                if (shootSequenceTimer.milliseconds() > 800) {
                    middle.setPosition(0);
                    linkage.setPosition(0.3567);
                    shootSequenceTimer.reset();
                    shootSequenceState = 1;
                }
                break;

            case 1:
                if (shootSequenceTimer.milliseconds() > 600) {
                    back.setPosition(0.6);
                    shootSequenceTimer.reset();
                    shootSequenceState = 2;
                }
                break;

            case 2:
                if (shootSequenceTimer.milliseconds() > 800) {
                    back.setPosition(0);
                    intake.setPower(0);
                    wheel.setPower(0);
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

    public void updateCustomShootingSequence() {
        switch (shootSequenceState) {
            case 1:
                linkage.setPosition(0.3567);
                shooter.setVelocity(firstBallSpeed);
                if (shootSequenceTimer.milliseconds() > 1000) {
                    back.setPosition(0.6);
                    shootSequenceTimer.reset();
                    shootSequenceState = 2;
                }
                break;

            case 2:
                if (shootSequenceTimer.milliseconds() > 800) {
                    back.setPosition(0);
                    linkage.setPosition(0.0);
                    shooter.setVelocity(shooterSpeed);
                    shootSequenceTimer.reset();
                    shootSequenceState = 5;
                }
                break;

            case 5:
                if (shootSequenceTimer.milliseconds() > 600) {
                    front.setPosition(0.6);
                    shootSequenceTimer.reset();
                    shootSequenceState = 6;
                }
                break;

            case 6:
                if (shootSequenceTimer.milliseconds() > 800) {
                    front.setPosition(0);
                    linkage.setPosition(0.18);
                    shootSequenceTimer.reset();
                    shootSequenceState = 3;
                }
                break;

            case 3:
                if (shootSequenceTimer.milliseconds() > 600) {
                    middle.setPosition(0.6);
                    shootSequenceTimer.reset();
                    shootSequenceState = 4;
                }
                break;

            case 4:
                if (shootSequenceTimer.milliseconds() > 800) {
                    middle.setPosition(0);
                    intake.setPower(0);
                    wheel.setPower(0);
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

    public void updateNewCustomShootingSequence() {
        switch (shootSequenceState) {
            case 5:
                linkage.setPosition(0.0);
                shooter.setVelocity(firstBallSpeed);
                if (shootSequenceTimer.milliseconds() > 1000) {
                    front.setPosition(0.6);
                    shootSequenceTimer.reset();
                    shootSequenceState = 6;
                }
                break;

            case 6:
                if (shootSequenceTimer.milliseconds() > 800) {
                    front.setPosition(0);
                    linkage.setPosition(0.3567);
                    shooter.setVelocity(shooterSpeed);
                    shootSequenceTimer.reset();
                    shootSequenceState = 1;
                }
                break;

            case 1:
                if (shootSequenceTimer.milliseconds() > 600) {
                    back.setPosition(0.6);
                    shootSequenceTimer.reset();
                    shootSequenceState = 2;
                }
                break;

            case 2:
                if (shootSequenceTimer.milliseconds() > 800) {
                    back.setPosition(0);
                    linkage.setPosition(0.18);
                    shootSequenceTimer.reset();
                    shootSequenceState = 3;
                }
                break;

            case 3:
                if (shootSequenceTimer.milliseconds() > 600) {
                    middle.setPosition(0.6);
                    shootSequenceTimer.reset();
                    shootSequenceState = 4;
                }
                break;

            case 4:
                if (shootSequenceTimer.milliseconds() > 800) {
                    middle.setPosition(0);
                    intake.setPower(0);
                    wheel.setPower(0);
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