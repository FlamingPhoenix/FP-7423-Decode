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

@Autonomous(name = "Blue Auto Close NEW", group = "Autonomous")
@Configurable
public class blueCloseAuto extends OpMode {

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
    Servo front, back, middle, linkage, gate;

    // Intake hardware
    DcMotor intake;
    boolean inShoot = false;
    int shootSequenceState = 0;
    ElapsedTime shootSequenceTimer = new ElapsedTime();
    double shooterSpeed = -1620;
    double firstBallSpeed = -1620;
    int shootingOrder = 0;
    int storedShootingOrder = 0;
    boolean isFirstShoot = true;
    boolean keepShooterWarmed = false;

    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
        pathTimer = new Timer();
        shootSequenceTimer = new ElapsedTime();
        Constants.driveConstants.setUseBrakeModeInTeleOp(true);
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(144 - 121.872, 122.018, Math.toRadians(180 - 45)));

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
        gate = hardwareMap.servo.get("lock");

        // Initialize intake hardware
        intake = hardwareMap.get(DcMotor.class, "intake");

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
        public PathChain Path10;

        public Paths(Follower follower) {
            Path1 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(144 - 121.872, 122.018),

                                    new Pose(144 - 97.465, 96.556)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180 - 45), Math.toRadians(180 - 48))

                    .build();

            Path2 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(144 - 97.465, 96.556),
                                    new Pose(144 - 87.004, 62.119),
                                    new Pose(144 - 97.153, 61.186)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180 - 48), Math.toRadians(180 - 0))

                    .build();

            Path3 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(144 - 97.153, 61.186),

                                    new Pose(144 - 131.580, 60.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180 - 0), Math.toRadians(180 - 0))

                    .build();

            Path4 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(144 - 131.580, 60.000),
                                    new Pose(144 - 84.790, 60.526),
                                    new Pose(144 - 88.854, 86.565)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180 - 0), Math.toRadians(180 - 48))

                    .build();

            Path5 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(144 - 88.854, 86.565),
                                    new Pose(144 - 108.242, 51.563),
                                    new Pose(144 - 127.394, 60.257)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180 - 48), Math.toRadians(180 - 0))

                    .build();

            Path6 = follower.pathBuilder()
          .addPath(
            new BezierCurve(
              new Pose(144 - 127.394, 60.257),
            new Pose(144 - 124.072, 58.035),
            new Pose(144 - 131.719, 53.859)
            )
          )
          .setLinearHeadingInterpolation(Math.toRadians(180 - 0), Math.toRadians(180 - 60))
          .build();

            Path7 = follower.pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(144 - 131.719, 53.859),
                                    new Pose(144 - 127.906, 48.269),
                                    new Pose(144 - 136.122, 59.488),
                                    new Pose(144 - 131.719, 53.913)
                            )

          )
          .setLinearHeadingInterpolation(Math.toRadians(180 - 60), Math.toRadians(180 - 40))
          .build();
            Path8 = follower.pathBuilder()
          .addPath(
            new BezierCurve(
              new Pose(144 - 131.719, 53.913),
            new Pose(144 - 90.671, 63.579),
            new Pose(144 - 84.916, 79.217)
            )
          )
          .setLinearHeadingInterpolation(Math.toRadians(180 - 40), Math.toRadians(180 - 48))
          .build();

      Path9 = follower.pathBuilder()
          .addPath(
            new BezierCurve(
              new Pose(144 - 84.916, 79.217),
            new Pose(144 - 81.183, 86.065),
            new Pose(144 - 125.875, 82.249)
            )
          )
          .setLinearHeadingInterpolation(Math.toRadians(180 - 48), Math.toRadians(180 - 0))
          .build();

      Path10 = follower.pathBuilder()
          .addPath(
            new BezierLine(
              new Pose(144 - 125.875, 82.249),
            new Pose(144 - 88.435, 108.426)
            )
          )
          .setLinearHeadingInterpolation(Math.toRadians(180 - 0), Math.toRadians(180 - 34))
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
                    gate.setPosition(0.4628);
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
                        gate.setPosition(0.4628);
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
                    setPathState(18);
                }
                break;
            // Second cycle of paths 6-8
            case 18:
                follower.followPath(paths.Path5);
                setPathState(19);
                break;
            case 19:
                if(!follower.isBusy()){
                    if(pathTimer.getElapsedTimeSeconds() > 0.2) {
                        follower.followPath(paths.Path6);
                        intake.setPower(-1);
                        setPathState(20);
                    }
                }
                break;
            case 20:
                if(!follower.isBusy()){
                    if(pathTimer.getElapsedTimeSeconds()>1) {
                        follower.followPath(paths.Path7, 0.4, true);
                        setPathState(21);
                    }
                }
                break;
            case 21:
                if(!follower.isBusy()){
                    if(pathTimer.getElapsedTimeSeconds()>2) {
                        gate.setPosition(0.4628);
                        intake.setPower(-0.3);
                        shooter.setVelocity(shooterSpeed);
                        follower.followPath(paths.Path8);
                        setPathState(22);
                    }
                }
                break;
            case 22:
                if(!follower.isBusy()){
                    pathTimer.resetTimer();
                    startShooting();
                    setPathState(23);
                }
                break;
            case 23:
                if (!inShoot) {
                    setPathState(24);
                }
                break;
            // Final sequence using Path9 and Path10
            case 24:
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
                    gate.setPosition(0.4628);
                    setPathState(28);
                }
                break;
            case 28:
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
            case 30:
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
                    storedShootingOrder = 0;
                    break;
                case 22:
                    storedShootingOrder = 3;
                    break;
                case 23:
                default:
                    storedShootingOrder = 1;
                    break;
            }
        } else {
            storedShootingOrder = 0;
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

            // Always use normal shooting order for all sequences
            shootingOrder = 0;

            // Set initial state based on shooting order
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
                linkage.setPosition(0.66);
                shooter.setVelocity(shooterSpeed);
                if (shootSequenceTimer.milliseconds() > 300) {
                    front.setPosition(0.4);
                    shootSequenceTimer.reset();
                    shootSequenceState = 2;
                }
                break;

            case 2:
                if (shootSequenceTimer.milliseconds() > 300) {
                    front.setPosition(0);
                    linkage.setPosition(0.85);
                    shootSequenceTimer.reset();
                    shootSequenceState = 3;
                }
                break;

            case 3:
                if (shootSequenceTimer.milliseconds() > 300) {
                    middle.setPosition(0.4);
                    shootSequenceTimer.reset();
                    shootSequenceState = 4;
                }
                break;

            case 4:
                if (shootSequenceTimer.milliseconds() > 300) {
                    middle.setPosition(0.04);
                    linkage.setPosition(1.0);
                    shootSequenceTimer.reset();
                    shootSequenceState = 5;
                }
                break;

            case 5:
                if (shootSequenceTimer.milliseconds() > 320) {
                    back.setPosition(0.78);
                    shootSequenceTimer.reset();
                    shootSequenceState = 6;
                }
                break;

            case 6:
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