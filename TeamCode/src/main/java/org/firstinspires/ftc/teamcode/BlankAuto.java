package org.firstinspires.ftc.teamcode;

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
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "Test Auto", group = "Autonomous")
@Configurable // Panels
public class BlankAuto extends OpMode {

    private TelemetryManager panelsTelemetry; // Panels Telemetry instance
    public Follower follower; // Pedro Pathing follower instance
    private Timer pathTimer; // Timer for path state transitions
    private int pathState; // Current autonomous path state (state machine)
    private Paths paths; // Paths defined in the Paths class

    // Shooting hardware
    DcMotorEx shooter;
    Servo front, back, middle, linkage;
    boolean inShoot = false;
    int shootSequenceState = 0;
    ElapsedTime shootSequenceTimer = new ElapsedTime();
    double shooterSpeed = -1200;
    double firstBallSpeed = -1150; // Slower speed for first ball

    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
        pathTimer = new Timer();
        shootSequenceTimer = new ElapsedTime();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(110.699, 135.502, Math.toRadians(90)));

        // Initialize shooting hardware
        shooter = hardwareMap.get(DcMotorEx.class, "shooter");
        front = hardwareMap.servo.get("front");
        back = hardwareMap.servo.get("back");
        middle = hardwareMap.servo.get("middle");
        linkage = hardwareMap.servo.get("linkage");

        // Setup shooter
        shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(60, 0, 0.2, 17.2));

        paths = new Paths(follower); // Build paths

        panelsTelemetry.debug("Status", "Initialized");
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

        public Paths(Follower follower) {
            Path1 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(110.699, 135.502), new Pose(85.709, 100.685))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(35))
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
                    // Start shooting sequence when path is complete
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
            shootSequenceState = 1;
            shootSequenceTimer.reset();
        }
    }

    public void updateShootingSequence() {
        if (inShoot) {
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
                        linkage.setPosition(0.25); // Move shooter to middle position
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
                        middle.setPosition(0); // Reset middle servo
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
    }
}