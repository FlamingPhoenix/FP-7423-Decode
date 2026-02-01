package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
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
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.shooter.ShootCalculator;

@Autonomous(name = "Simple Auto", group = "Autonomous")
@Configurable
public class SimpleAuto extends OpMode {

    private TelemetryManager panelsTelemetry;
    public Follower follower;
    private Timer pathTimer;
    private int pathState;
    private Paths paths;

    // Limelight hardware
    Limelight3A limelight;
    boolean limeLightWorking = true;
    ShootCalculator shootCalculator;

    // Shooting hardware
    DcMotorEx shooter;
    Servo front, back, middle, linkage;
    boolean inShoot = false;
    int shootSequenceState = 0;
    ElapsedTime shootSequenceTimer = new ElapsedTime();
    double shooterSpeed = -1200;
    double positionCompensation = 15.5; // Start with back ball compensation

    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
        pathTimer = new Timer();
        shootSequenceTimer = new ElapsedTime();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(80.113, 8.000, Math.toRadians(90)));

        // Initialize Limelight
        try {
            limelight = hardwareMap.get(Limelight3A.class, "limelight");
            limelight.setPollRateHz(100);
            limelight.start();
            limelight.pipelineSwitch(0);
            shootCalculator = new ShootCalculator();
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

        // Setup shooter
        shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(75, 0, 0.2, 17.2));

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

        // Update shooter speed from Limelight if available
        if (limeLightWorking && pathState >= 1) {
            LLResult result = limelight.getLatestResult();
            if (result != null && result.isValid()) {
                shooterSpeed = Math.min(-1050, shootCalculator.calculateRPMForTele(result, positionCompensation));
                shooter.setVelocity(shooterSpeed);
            }
        }

        panelsTelemetry.debug("Path State", pathState);
        panelsTelemetry.debug("X", follower.getPose().getX());
        panelsTelemetry.debug("Y", follower.getPose().getY());
        panelsTelemetry.debug("Heading", Math.toDegrees(follower.getPose().getHeading()));
        panelsTelemetry.debug("Shooting", inShoot ? "Active" : "Inactive");
        panelsTelemetry.debug("Shooter Speed", shooterSpeed);
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void start() {
        shooter.setVelocity(shooterSpeed); // Start shooter to reach target speed
        linkage.setPosition(0.3567); // Position linkage for first ball
        setPathState(0);
    }

    public static class Paths {
        public PathChain Path1;
        public PathChain Path2;

        public Paths(Follower follower) {
            Path1 = follower.pathBuilder().addPath(
                new BezierLine(
                    new Pose(80.113, 8.000),
                    new Pose(75.380, 20.451)
                )
            ).setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(62.5))
            .build();

            Path2 = follower.pathBuilder().addPath(
                new BezierLine(
                    new Pose(122.000, 81.500),
                    new Pose(128.462, 75.502)
                )
            ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
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
                    // Start shooting sequence after path completion
                    startShooting();
                    setPathState(2);
                }
                break;
            case 2:
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

    public void startShooting() {
        if (!inShoot) {
            inShoot = true;
            shootSequenceTimer.reset();
            shootSequenceState = 1;
        }
    }

    public void updateShootingSequence() {
        if (inShoot) {
            switch (shootSequenceState) {
                case 1: // Move linkage to back position and lift back ball
                    linkage.setPosition(0.3567);
                    positionCompensation = 15.5; // Back ball compensation
                    if (shootSequenceTimer.milliseconds() > 1500) {
                        back.setPosition(0.6);
                        shootSequenceTimer.reset();
                        shootSequenceState = 2;
                    }
                    break;

                case 2: // Wait then reset back servo and move to middle
                    if (shootSequenceTimer.milliseconds() > 500) {
                        back.setPosition(0);
                        linkage.setPosition(0.25);
                        positionCompensation = 11; // Middle ball compensation
                        shootSequenceTimer.reset();
                        shootSequenceState = 3;
                    }
                    break;

                case 3: // Move linkage to middle position and lift middle ball
                    if (shootSequenceTimer.milliseconds() > 900) {
                        middle.setPosition(0.6);
                        shootSequenceTimer.reset();
                        shootSequenceState = 4;
                    }
                    break;

                case 4: // Wait then reset middle servo and start path
                    if (shootSequenceTimer.milliseconds() > 500) {
                        middle.setPosition(0.2);
                        follower.followPath(paths.Path2);
                        shootSequenceTimer.reset();
                        shootSequenceState = 5;
                    }
                    break;

                case 5: // Wait for path to complete
                    if (!follower.isBusy()) {
                        shootSequenceTimer.reset();
                        shootSequenceState = 6;
                    }
                    break;

                case 6: // Half second pause
                    if (shootSequenceTimer.milliseconds() > 500) {
                        linkage.setPosition(0.0);
                        positionCompensation = 5; // Front ball compensation
                        shootSequenceTimer.reset();
                        shootSequenceState = 7;
                    }
                    break;

                case 7: // Move linkage to front position and lift front ball
                    if (shootSequenceTimer.milliseconds() > 900) {
                        front.setPosition(0.6);
                        shootSequenceTimer.reset();
                        shootSequenceState = 8;
                    }
                    break;

                case 8: // End sequence
                    if (shootSequenceTimer.milliseconds() > 500) {
                        front.setPosition(0);
                        shooter.setPower(0);
                        inShoot = false;
                        shootSequenceState = 0;
                    }
                    break;
            }
        }
    }
}