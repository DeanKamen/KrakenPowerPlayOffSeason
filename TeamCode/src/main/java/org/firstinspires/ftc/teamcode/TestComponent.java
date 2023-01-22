package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.subsystems.*;

@TeleOp
public class TestComponent extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Lift lift = Lift.init(hardwareMap, telemetry);
        Hook hook = Hook.init(hardwareMap, telemetry);
        WebcamObjectDetection camera = WebcamObjectDetection.init(hardwareMap, telemetry);

        boolean noBumper = true;

        telemetry.addData ("Runtime:", "%f.1 seconds", this.getRuntime());
        telemetry.addData ("Current Press:", "N/A");
        telemetry.update();


        waitForStart();
        getRuntime();
        if (isStopRequested()) return;

        while (opModeIsActive()) {
            String objectRecognized = camera.getRecognizedObject();
        }
    }
}
