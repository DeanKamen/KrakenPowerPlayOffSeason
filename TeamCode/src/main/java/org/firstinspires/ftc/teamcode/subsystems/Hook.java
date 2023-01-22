package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * This class is used to control Proto's Hook mechanism.
 */
public class Hook {
    private static final double HOOK_HOLD_POSITION = 0.35;
    private static final double HOOK_RELEASE_POSITION = 0.0;

    private Servo servoHook;
    private double hookServoPosition = 0.0;
    private boolean subsystemExists = false;
    private boolean hookHold;
    private boolean prevButtonState;
    private Telemetry telemetry;
    private boolean prev_state;

    /**
     * Factory method to create a Hook object.
     *
     * @param hardwareMap contextual hardware map, needed to get an instance of servo
     * @param telemetry contextual telemetry object, needed to send telemetry logging messages to the drver station.
     * @return an instance of Hook
     */
    public static Hook init (HardwareMap hardwareMap, Telemetry telemetry) {
        Hook hook = new Hook();
        hook.telemetry = telemetry;
        hook.prev_state = false;
        try {
            hook.servoHook = hardwareMap.get(Servo.class, "cone_hook");
            // This makes the start position open, we need a special sticker to warn people per the robot rules.
            hook.servoHook.setDirection(Servo.Direction.FORWARD);
            hook.subsystemExists = true;
            hook.prevButtonState = false;
            hook.hookHold = false;
            telemetry.addLine("Hook Subsystem: does exist and is initialized.");
        } catch (Exception e) {
            hook.subsystemExists = false;
            telemetry.addLine("Hook Subsystem: does not exist and is not initialized.");
        }
        telemetry.update();
        return hook;
    }

    /**
     * Getter for subsystemExists.
     *
     * @return true if the subsystem was instantiated and exists on the robot, false if it does not.
     */
    public boolean exists () {
        return subsystemExists;
    }
    /**
     * Test method to slowly move the servo to the right.
     */
    public void bumpUp () {
        hookServoPosition += 0.01;
        telemetry.addLine("Hook Subsystem: bumping up to "+hookServoPosition);
        servoHook.setPosition(hookServoPosition);
    }

    /**
     * Test method to slowly move the servo to the left.
     */
    public void bumpDown () {
        hookServoPosition -= 0.01;
        telemetry.addLine("Hook Subsystem: bumping down to "+hookServoPosition);
        servoHook.setPosition(hookServoPosition);
    }

    /**
     * TODO: COMPLETE DOCUMENTING THIS METHOD
     * @param closed
     */
    public void setHook(boolean closed){

        telemetry.addLine ("Hook Subsystem: setState = " + closed);

        if (!closed) {
            telemetry.addLine ("Hook Subsystem: setting open.");
            servoHook.setPosition(HOOK_RELEASE_POSITION);
            hookHold = false;
        } else {
            telemetry.addLine ("Hook Subsystem: seting closed.");
            servoHook.setPosition(HOOK_HOLD_POSITION);
            hookHold = true;
        }
    }

    public void setToZero () {
        telemetry.addLine ("Hook Subsystem: setting servo to position 0");
        servoHook.setPosition(HOOK_RELEASE_POSITION);
        hookHold = false;
    }

    public void toggleHook(boolean button) {
        if(button && !prevButtonState){
            if (hookHold) {
                telemetry.addLine ("Hook Subsystem: Releasing cone.");
                servoHook.setPosition(HOOK_RELEASE_POSITION);
                hookHold = false;
            } else {
                telemetry.addLine ("Hook Subsystem: gripping cone.");
                servoHook.setPosition(HOOK_HOLD_POSITION);
                hookHold = true;
            }
        }

        prevButtonState = button;
    }

    public boolean getHookState(){
        return hookHold;
    }
}
