package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class Drive {

    /**
     * The four DcMotor members are references to the  robot's four drive motors.
     */
    private DcMotor motorFrontLeft;
    private DcMotor motorBackLeft;
    private DcMotor motorFrontRight;
    private DcMotor motorBackRight;
    private boolean boost;
    private boolean brake;
    private boolean slowmode;

    /**
     * This holds a reference to the owner opmode's telemetry object, it is used to send information
     * to the driver station.
     */
    private Telemetry telemetry;

    private boolean subsystemExists = false;

    /**
     * Initializes and returns an instance of the Drive subsystem that an opmode can use.
     *
     * @param hardwareMap   the opmode's hardware map with the robot configuration
     * @param telemetry     the opmode's telemetry object that allows data to be shared with the driver station.
     */
    public static Drive init (HardwareMap hardwareMap, Telemetry telemetry) {

        // Create a new instance of Drive
        Drive drive = new Drive();
        drive.telemetry = telemetry;
        try{
        // Set get an instance of DcMotor from the hardware map for each of the drive motors.
            drive.motorFrontLeft = hardwareMap.dcMotor.get("motorFrontLeft");
            drive.motorBackLeft = hardwareMap.dcMotor.get("motorBackLeft");
            drive.motorFrontRight = hardwareMap.dcMotor.get("motorFrontRight");
            drive.motorBackRight = hardwareMap.dcMotor.get("motorBackRight");
            drive.setMotorBraking();
            drive.subsystemExists = true;
        } catch (Exception e) {
            telemetry.addData("Drive Instantiation Problem:", "%s", e.toString());
            drive.subsystemExists = false;
        }
        // Return the initialized drive.
        return drive;
    }

    // TODO: Document methods
    public boolean exists () {
        return subsystemExists;
    }

    // TODO: Document methods
    public void setMotorFloating()
    {
        motorFrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motorBackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motorFrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motorBackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    // TODO: Document methods
    public void setMotorBraking()
    {
        motorFrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    // TODO: Document methods
    public void setMotorsMode(DcMotor.RunMode mode)
    {
        motorFrontLeft.setMode(mode);
        motorBackLeft.setMode(mode);
        motorFrontRight.setMode(mode);
        motorBackRight.setMode(mode);
    }

    // TODO: Document methods
    public void setMotorSpeeds (double x, double y, double rx) {
        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio, but only when
        // at least one is out of the range [-1, 1]

        double denominator = (Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1));
        if(!boost || slowmode){
            denominator = denominator*2.2;
        }
        if(brake){
            x=0;
            y=0;
        }else if(slowmode){
            x=x/2;
            y=y/2;
        }
        double frontLeftPower = (y + x + rx) / denominator;
        double backLeftPower = (y - x + rx) / denominator;
        double frontRightPower = (y - x - rx) / denominator;
        double backRightPower = (y + x - rx) / denominator;

        motorFrontLeft.setPower(frontLeftPower);
        motorBackLeft.setPower(backLeftPower);
        motorFrontRight.setPower(frontRightPower);
        motorBackRight.setPower(backRightPower);
    }

    // TODO: Document methods
    public void setMotorPowerForEncoder(double x, double y, double rx){
        double denominator = -1*(Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1));
        double frontLeftPower = (y + x + rx) / denominator;
        double backLeftPower = (y - x + rx) / denominator;
        double frontRightPower = (y - x - rx) / denominator;
        double backRightPower = (y + x - rx) / denominator;

        motorFrontLeft.setPower(frontLeftPower);
        motorBackLeft.setPower(backLeftPower);
        motorFrontRight.setPower(frontRightPower);
        motorBackRight.setPower(backRightPower);
    }

    // TODO: Document methods
    public void setMotorLengthForEncoder(double length, double x, double y, double rx){
        double denominator = -1*(Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1));
        double frontLeftRatio = (y + x + rx) / denominator;
        double backLeftRatio = (y - x + rx) / denominator;
        double frontRightRatio = (y - x - rx) / denominator;
        double backRightRatio = (y + x - rx) / denominator;
        int frontRightPos = (int)(length*frontRightRatio);
        int frontLeftPos = (int)(length*frontLeftRatio);
        int backRightPos = (int)(length*backRightRatio);
        int backLeftPos = (int)(length*backLeftRatio);
        motorFrontLeft.setTargetPosition(frontLeftPos);
        motorBackLeft.setTargetPosition(backLeftPos);
        motorFrontRight.setTargetPosition(frontRightPos);
        motorBackRight.setTargetPosition(backRightPos);
    }

    // TODO: Document methods
    public void setBoost(boolean boost1){
        boost = boost1;
    }

    // TODO: Document methods
    public void setBrake(float howMuchBrake){
        if(howMuchBrake > 0.03 && howMuchBrake < 0.7){
            slowmode = true;
            brake = false;
        }else if(howMuchBrake > 0.7) {
            slowmode = false;
            brake = true;
        }else{
            slowmode = false;
            brake = false;
        }
    }

    // TODO: Document methods
    public double getAngle (double y, double x) {
        return Math.atan2(y,x);
    }

    // TODO: Document methods
    public double getMagnitude (double y, double x) {
        return Math.sqrt (x*x + y*y);
    }

    // TODO: Document methods
    public void setWheelSpeedsAutoStrafeAndTurn (double angle, double magnitude, double turn) {
        double frAndBl = Math.abs(Math.sin(angle - 0.25 * Math.PI) * magnitude + turn);
        double flAndBr = Math.abs(Math.sin(angle + 0.25 * Math.PI) * magnitude + turn);

        double denominator = -1 * (Math.max (Math.max(frAndBl, flAndBr), 1));

        double frontLeftPower = flAndBr / denominator;
        double backLeftPower = frAndBl / denominator;
        double frontRightPower = frAndBl / denominator;
        double backRightPower = flAndBr / denominator;

        motorFrontLeft.setPower(frontLeftPower);
        motorBackLeft.setPower(backLeftPower);
        motorFrontRight.setPower(frontRightPower);
        motorBackRight.setPower(backRightPower);
    }

    // TODO: Document methods
    public OpenGLMatrix transformGamepadToDirection(OpenGLMatrix gamepad, OpenGLMatrix robotDir){
        double gamepad_x = gamepad.get(0,0); //x,y,z
        double gamepad_y = gamepad.get(0,1); //x,y,z
        double gamepad_angle = getAngle(gamepad_x, gamepad_y);

        double robotDir_x = robotDir.get(0,0); //x,y,z
        double robotDir_y = robotDir.get(0,1); //x,y,z
        double robotDir_angle = getAngle(robotDir_x, robotDir_y);

        return gamepad.rotated(AngleUnit.DEGREES, 90, 0,0, 1);
    }

    // TODO: Document methods
    public void directionForLength(double length, double dirX, double dirY, double dirRx){
        setMotorsMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setMotorPowerForEncoder(dirX, dirY, dirRx);
        setMotorLengthForEncoder(length, dirX, dirY, dirRx);
        setMotorsMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    // TODO: Document methods
    public void stopWheels () {
        motorFrontLeft.setPower(0);
        motorBackLeft.setPower(0);
        motorFrontRight.setPower(0);
        motorBackRight.setPower(0);
    }

    /**
     * Moves the robot side to side without turning.  Negative inches strafes left.
     *
     * @param inches how far to move
     * @param speed how fast to move
     */
    public void strafeRight (int inches, double speed) {
        // TODO: Implement this method.  Emperically calculate the moves based on telemetry from teleop Test mode.
    }

    /**
     * Moves the robot forward without turning.  Negative inches moves backwards.
     *
     * @param inches how far to move
     * @param speed how fast to move
     */
    public void moveForward (int inches, double speed) {
        // TODO: Implement this method.  Emperically calculate the moves based on telemetry from teleop Test mode.
    }
}