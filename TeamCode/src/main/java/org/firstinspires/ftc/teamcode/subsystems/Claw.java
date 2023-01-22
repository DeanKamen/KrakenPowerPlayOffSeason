package org.firstinspires.ftc.teamcode.subsystems;

import android.hardware.SensorEventListener;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * This class is used to control Proto's Claw mechanism.
 */
public class Claw {

    private Telemetry telemetry;
    private Servo servoClaw;
    private SensorEventListener sel;
    private boolean claw_locked_open;
    private boolean clawState; //true is closed

    private boolean subsystemExists;

    /**
     * Initializes and returns an instance of the Claw subsystem that an opmode can use.
     *
     * @param hardwareMap   the opmode's hardware map with the robot configuration
     * @param telemetry     the opmode's telemetry object that allows data to be shared with the driver station.
     */
    public static Claw init (HardwareMap hardwareMap, Telemetry telemetry) {

        // Create a new instance of Drive
        Claw claw = new Claw();
        claw.telemetry = telemetry;
        try {
            claw.servoClaw = hardwareMap.get(Servo.class, "claw");
            claw.subsystemExists = true;
            telemetry.addLine("Claw Subsystem: does exist and is initialized.");
            claw.claw_locked_open = false;
        } catch (Exception e) {
            claw.subsystemExists = false;
            telemetry.addLine("Claw Subsystem: does not exist and is not initialized.");
        }

        // Return the initialized drive.
        telemetry.update();
        return claw;
    }

    /**
     * Getter for subsystemExists.
     *
     * @return true if the subsystem was instantiated and exists on the robot, false if it does not.
     */
    public boolean exists () {
        return subsystemExists;
    }

    public void setClawState(boolean buttonDown){
        telemetry.addData("Claw Position" , servoClaw.getPosition());
        telemetry.update();
        if (buttonDown && !claw_locked_open) {
            servoClaw.setPosition(0.9);
            clawState = true;
        } else {
            servoClaw.setPosition(0);
            clawState = false;
        }
    }

    public void lockOpen(boolean lock){
        if(lock){
            claw_locked_open = true;
            setClawState(false);
        }
        else{
            claw_locked_open = false;
        }
    }

    public boolean getClawState(){
        return clawState;
    }
}
