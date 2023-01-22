package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="Vuforia Teleop", group="Linear Opmode")
public class vumarkTeleOp extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        // Get the subsystems initialized
        Drive drive = Drive.init(hardwareMap, telemetry);
        Claw claw = Claw.init(hardwareMap, telemetry);
        Lift lift = Lift.init(hardwareMap, telemetry);
        Hook hook = Hook.init(hardwareMap, telemetry);

        telemetry.addData ("Subsystem Status:", "%s", "Drive="+drive.exists()+", Claw="+claw.exists()+", Lift="+lift.exists()+", Hook="+hook.exists());
        boolean dpadUpPressed = false;
        boolean dpadDownPressed = false;
        telemetry.update();
        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            // Get all of the current gamepad inputs.

            if (drive.exists()) {
                // Get drive data from left and right sticks and run drive methods
                double y = -gamepad1.left_stick_y; // Remember, this is reversed!
                double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
                double rx = gamepad1.right_stick_x;
                boolean boost = gamepad1.left_stick_button;
                float brake = gamepad1.left_trigger;
                telemetry.addData("Left Stick: (x,y,boost,brake)", "%s", x + ", " + y + ", " + boost + ", " + brake);
                drive.setBoost(boost);
                drive.setBrake(brake);
                drive.setMotorSpeeds(x, y, rx);
            }

            // Get claw data from right trigger on the first gamepad
            float rightTrigger = gamepad1.right_trigger;
            telemetry.addData ("Claw Trigger Value:", "%f.2", rightTrigger);
            if (claw.exists()) {
                claw.setClawState(rightTrigger > 0.01);
            } else {
                telemetry.addLine("Error: Trying to toggle the claw but the claw doesn't exist.");
            }
            // Get hook data from the gamepad (a/X)
            boolean aCross = gamepad1.a;
            telemetry.addData ("Hook Toggle Button Press:", "%b", aCross);
            if (hook.exists()) {
                hook.toggleHook(gamepad1.cross);
                telemetry.addData("Hook Holding?: ", "%b", hook.getHookState());
            } else {
                telemetry.addLine ("Error: Trying to toggle the hook but the hook doesn't exist.");
            }

            // Get input and send it to lift
            if (lift.exists()) {
                telemetry.addData ("DPad Pressed (up,down):", "%s", gamepad1.dpad_up+"' "+gamepad1.dpad_down);
                lift.move_conditionally(gamepad1.dpad_up,gamepad1.dpad_down);
            }
        }
    }
}
