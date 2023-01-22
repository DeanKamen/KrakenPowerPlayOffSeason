package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.subsystems.*;


@Autonomous

public class MecanumAuto extends LinearOpMode {

    Drive drive;
    AprilTagObjectDetection camera;
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.update();
        camera = AprilTagObjectDetection.init(hardwareMap, telemetry);
        drive = Drive.init(hardwareMap, telemetry);
        telemetry.addData("Done:", "%S", "no");
        telemetry.update();
        int res = -1;

        waitForStart();

        for(int i = 0; i < 10; i++){
            sleep(1000);
            res = camera.getRecognizedObject();
            if(res != -1)
                break;
        }
        goForInches(1, direction.forward);
        if (res == 0) {
            goForInches(28, direction.left);
        }else if(res == 2){
            goForInches(28, direction.right);
        }else{}
        goForInches(29, direction.forward);



        drive.setBrake(0.9f);
        telemetry.addData("Done:", "%S", "yes");
        telemetry.update();
    }

    void goForMillis(int time, double x, double y, double rx){
        drive.setBrake(0.0f);
        drive.setMotorSpeeds(x ,y, rx);
        sleep(time);
        drive.setBrake(0.9f);
//something,something, and another something
    }
    void goForInches(double inches, direction dir){
        int local_millis_fb = (int) ((inches / 21) * 1150);
        int local_millis_lr = (int) ((inches / 10) * 795);
        if(dir == direction.forward){
            goForMillis(local_millis_fb, 0, 0.4, 0);
        }else if(dir == direction.backward){
            goForMillis(local_millis_fb, 0, -0.4, 0);
        }else if(dir == direction.left){
            goForMillis(local_millis_lr, -0.4, 0, 0);
        }else if(dir == direction.right) {
            goForMillis(local_millis_lr, 0.4, 0, 0);
        }
    }
    enum direction{
        forward,
        backward,
        left,
        right
    }

}
