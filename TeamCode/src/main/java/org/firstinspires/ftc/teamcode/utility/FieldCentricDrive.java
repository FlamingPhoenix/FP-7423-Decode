package org.firstinspires.ftc.teamcode.utility;

import static java.lang.Math.abs;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class FieldCentricDrive {
    DcMotor fl, fr, bl, br;
    IMU imu;
    boolean useIMU = true;
    double heading;
    double multiplier = 0.8;
    double lastHeading = 0;
    double filteredHeading = 0;
    double headingFilterAlpha = 0.9; // Smoothing factor (0.8-0.95 works well)
    double joystickDeadzone = 0.05; // Deadzone to prevent jitter
    /**
     * Initialize motors
     * @param hardwareMap hardwareMap from opmode
     */
    public FieldCentricDrive(HardwareMap hardwareMap){
        fl = hardwareMap.dcMotor.get("fl");
        fr = hardwareMap.dcMotor.get("fr");
        bl = hardwareMap.dcMotor.get("bl");
        br = hardwareMap.dcMotor.get("br");
        //reverse motors
        fr.setDirection(DcMotor.Direction.REVERSE);
        br.setDirection(DcMotor.Direction.REVERSE);
        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.UP, RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
        imu.initialize(parameters);
    }
    /**
     * Initialize motors
     * @param hardwareMap hardwareMap from opmode
     * @param useIMU whether or not to use the IMU
     */
    public FieldCentricDrive(HardwareMap hardwareMap, boolean useIMU){
        fl = hardwareMap.dcMotor.get("fl");
        fr = hardwareMap.dcMotor.get("fr");
        bl = hardwareMap.dcMotor.get("bl");
        br = hardwareMap.dcMotor.get("br");
        //reverse motors
        fr.setDirection(DcMotor.Direction.REVERSE);
        br.setDirection(DcMotor.Direction.REVERSE);
        this.useIMU = useIMU;
        if(useIMU) {
        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.UP, RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
        imu.initialize(parameters);}
        else{
            heading = 0;
        }
    }
    /**
     * Drive based off of gamepad inputs
     * @param gpx gamepad x input
     * @param gpy gamepad y input
     * @param rx gamepad rotation input
     */
    public void drive(double gpx, double gpy, double rx) {
        double botHeading;
        if(useIMU) {
            botHeading= -imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);//might be degrees
        }
        else{
            botHeading = heading;
        }
        double rotX = gpx * Math.cos(botHeading) - gpy * Math.sin(botHeading);
        double rotY = gpx * Math.sin(botHeading) + gpy * Math.cos(botHeading);


        double denominator = Math.max(abs(gpy) + abs(gpx) + abs(rx), 1);
        double flp = (rotY + rotX + rx) / denominator;
        double blp = (rotY - rotX + rx) / denominator;
        double frp = (rotY - rotX - rx) / denominator;
        double brp = (rotY + rotX - rx) / denominator;

        fl.setPower(multiplier*flp);
        bl.setPower(multiplier*blp);
        fr.setPower(multiplier*frp);
        br.setPower(multiplier*brp);
    }
    /**
     * Drive based off of gamepad
     * More functionality because of more access to controls
     * @param gamepad1 gamepad 1
     */
    public void drive(Gamepad gamepad1, double multiplier){
        // Apply deadzone to joystick inputs
        double x = applyDeadzone(-gamepad1.left_stick_x*1.1);
        double y = applyDeadzone(gamepad1.left_stick_y);
        double rx = applyDeadzone(-0.65*gamepad1.right_stick_x);
        double botHeading;
        if(useIMU) {
            double rawHeading = -imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            // Check for heading jumps (IMU reset/drift)
            double headingDiff = Math.abs(rawHeading - lastHeading);
            if (headingDiff > Math.PI) {
                // Large jump detected - likely IMU reset, use raw heading
                filteredHeading = rawHeading;
            } else {
                // Apply low-pass filter to smooth heading
                filteredHeading = headingFilterAlpha * filteredHeading + (1 - headingFilterAlpha) * rawHeading;
            }

            lastHeading = rawHeading;
            botHeading = filteredHeading;
        }
        else{
            botHeading = heading;
        }
        double rotX = x * Math.cos(botHeading) - y * Math.sin(botHeading);
        double rotY = x * Math.sin(botHeading) + y * Math.cos(botHeading);


        double denominator = Math.max(abs(y) + abs(x) + abs(rx), 1);
        double flp = (rotY + rotX + rx) / denominator;
        double blp = (rotY - rotX + rx) / denominator;
        double frp = (rotY - rotX - rx) / denominator;
        double brp = (rotY + rotX - rx) / denominator;


        fl.setPower((multiplier*Math.pow((flp),1)));
        bl.setPower((multiplier*Math.pow((blp),1)));
        fr.setPower((multiplier*Math.pow((frp), 1)));
        br.setPower((multiplier*Math.pow((brp),1)));
    }
    /**
     * Get robot heading in radians
     * @return robot heading
     */
    public double getHeading(){
        if(useIMU) {
            return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        }
        else{
            return heading;
        }
    }
    /**
     * Get robot heading in a specified angle unit
     * @param angleUnit angle unit to return heading in
     * @return robot heading
     */
    public double getHeading(AngleUnit angleUnit){
        if(useIMU) {
            return imu.getRobotYawPitchRollAngles().getYaw(angleUnit);
        }
        else{
            return heading;
        }
    }
    /**
    * Reset the IMU
    */
    public void resetIMU(){
        if(useIMU) {
            imu.resetYaw();
        }
    }
    /**
     * Set the robot heading
     * @param heading heading in radians
     */
    public void setHeading(double heading){
        this.heading = heading;
    }
    public void setMultiplier(double mult){
        this.multiplier = mult;
    }

    /**
     * Apply deadzone to joystick input to prevent jitter
     * @param input raw joystick input
     * @return filtered input with deadzone applied
     */
    private double applyDeadzone(double input) {
        if (Math.abs(input) < joystickDeadzone) {
            return 0.0;
        }
        return input;
    }
}
