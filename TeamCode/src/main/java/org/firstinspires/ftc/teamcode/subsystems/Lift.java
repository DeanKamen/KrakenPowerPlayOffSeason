package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Lift {

    private DcMotor liftMotor;
    private Telemetry telemetry;
    boolean prevUpButtonState;
    boolean prevDownButtonState;

    int targetLiftState; //0,1,2,3
    int prevLiftState;

    private final double steps_per_rotation = 384.5;
    private final int spool_dia = 112;

    private final static double MOTOR_SPEED = 0.5;

    private final double target_ground_height = 100.0;
    private final double target_low_height = 355.6;
    private final double target_med_height = 609.6;
    private final double target_high_height = 863.6;

    private final double ground_pos = (target_ground_height/spool_dia) * steps_per_rotation;
    private final double low_pos = (target_low_height/spool_dia) * steps_per_rotation;
    private final double med_pos = (target_med_height/spool_dia) * steps_per_rotation;
    private final double high_pos = (target_high_height/spool_dia) * steps_per_rotation;

    private boolean subsystemExists;

    public static Lift init (HardwareMap hardwareMap, Telemetry telemetry) {
        Lift lift = new Lift();
        lift.telemetry = telemetry;
        try {
            lift.liftMotor = hardwareMap.dcMotor.get("liftMotor");
            lift.liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            lift.liftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
            // SRD ADDED FOLLOWING LINE 11/30/2022
            lift.telemetry = telemetry;
            lift.telemetry.addData("Lift Position:", "%d steps", lift.liftMotor.getCurrentPosition());
            lift.telemetry.update();
            lift.liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            lift.telemetry.update();
            // The previous line sets the current position to zero so make sure the lift is all the way down!

            lift.prevUpButtonState = false;
            lift.prevDownButtonState = false;
            lift.targetLiftState = 0;
            lift.prevLiftState = 0;
            lift.subsystemExists = true;
        } catch (Exception e) {
            telemetry.addData("Lift Instantiation Problem:", "%s", e.toString());
            lift.subsystemExists = false;
        }
        return lift;
    }


    //if (prevUpButtonState == false && upButton == true) {
    public void moveUp () {
        telemetry.addData("Current Press:", "DPAD UP");
        if (targetLiftState < 4) {
            targetLiftState += 1;
            go_to_position_based_on_state(targetLiftState);
        } else {
            telemetry.addLine ("Lift: Could not move up anymore.");
        }
        telemetry.update();
    }


    public void moveDown() {
        telemetry.addData ("Current Press:", "DPAD DOWN");
        if (targetLiftState > 0) {
            targetLiftState -= 1;
            go_to_position_based_on_state(targetLiftState);
        } else {
            telemetry.addLine("Lift: Could not move down anymore.");
        }
        telemetry.update();
    }

    public void go_to_position_based_on_state(int state){
        telemetry.addData("Lift SM State:", "%d", state);
        if(state == 0){
            liftMotor.setTargetPosition(0);//cone pickup pos
        }else if(state == 1){
            liftMotor.setTargetPosition(((int) ground_pos));
        }else if(state == 2){
            liftMotor.setTargetPosition(((int) low_pos));
        }else if(state == 3){
            liftMotor.setTargetPosition(((int) med_pos));
        }else if(state == 4){
            liftMotor.setTargetPosition(((int) high_pos));
        }else{
            liftMotor.setTargetPosition(((int) ground_pos));
        }
        liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftMotor.setPower(MOTOR_SPEED);
        telemetry.update();
    }

    /**
     * There is a flaw in this method that has to do with Runtime and execution.
     * @param up_button
     * @param down_button
     */
    public void move_conditionally(boolean up_button, boolean down_button){

        telemetry.addData("Lift State Change (up,prevUp,down,prevDown:", "%S", up_button + ", " + prevUpButtonState + ", "+down_button+", " + prevDownButtonState);
        telemetry.addData("Lift Desired location: ", "%S", targetLiftState + " " + liftMotor.getCurrentPosition() + " " + liftMotor.getTargetPosition());
        if(up_button && !prevUpButtonState){
            moveUp();
        }else if(down_button && !prevDownButtonState){
            moveDown();
        }
        prevDownButtonState = down_button;
        prevUpButtonState = up_button;
    }
    public void move_conditionally(boolean up_button, boolean down_button, Hook hook, Claw claw){

        telemetry.addData("Lift State Change (up,prevUp,down,prevDown:", "%S", up_button + ", " + prevUpButtonState + ", "+down_button+", " + prevDownButtonState);
        telemetry.addData("Lift Desired location: ", "%S", targetLiftState + " " + liftMotor.getCurrentPosition() + " " + liftMotor.getTargetPosition());
        if(up_button && !prevUpButtonState){
            if(getLiftState() == 0){
                if(hook.getHookState() == false) //hook closed
                {
                    hook.setHook(true);
                }
                if(claw.getClawState()){ //claw closed
                    claw.lockOpen(true);
                }
                try{
                    Thread.sleep(500);
                }catch(Exception e){
                }
            }
            moveUp();
        }else if(down_button && !prevDownButtonState){
            if(getLiftState() == 1) {
                if (hook.getHookState()) //hook open
                {
                    hook.setHook(false);
                }
                if (claw.getClawState() && (getLiftState() == 1)) { //claw closed
                    claw.lockOpen(true);
                }
                try{
                    Thread.sleep(500);
                }catch(Exception e){
                }
            }

            moveDown();
        }
        if(Math.abs(liftMotor.getCurrentPosition() - liftMotor.getTargetPosition()) < 10){
            claw.lockOpen(false);
        }
        prevDownButtonState = down_button;
        prevUpButtonState = up_button;
    }

    /**
     * TODO: This should be in an abstract class the this inherits from.
     * @return
     */
    public boolean exists() {
        return subsystemExists;
    }

    public int getLiftState(){
        return targetLiftState;
    }
}
