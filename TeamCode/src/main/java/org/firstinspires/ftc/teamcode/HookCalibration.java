package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Hook;

@TeleOp(name="Hook Calibration Teleop", group="Linear Opmode")
public class HookCalibration extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        // Get the subsystems initialized
        Hook hook = Hook.init(hardwareMap, telemetry);

        telemetry.addData("Subsystem Status:", "%s", ", Hook=" + hook.exists());
        boolean dpadUpPressed = false;
        boolean dpadDownPressed = false;


        telemetry.update();
        waitForStart();

        if (isStopRequested()) {
            hook.setToZero();
            return;
        }

        while (opModeIsActive()) {

            // Get hook data from the gamepad (a/X)
            boolean aCross = gamepad1.a;
            telemetry.addData ("Hook Toggle Button Press:", "%b", aCross);
            if (hook.exists()) {
                hook.toggleHook(gamepad1.cross);
                telemetry.addData("Hook Holding?: ", "%b", hook.getHookState());
            } else {
                telemetry.addLine ("Error: Trying to toggle the hook but the hook doesn't exist.");
            }

        }
        hook.setToZero();
    }
}
