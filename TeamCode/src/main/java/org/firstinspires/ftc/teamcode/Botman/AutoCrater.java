package org.firstinspires.ftc.teamcode.Botman;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "Crater Side", group = "Botman")
public class AutoCrater extends OpMode {

    HardwareBotman robot = new HardwareBotman();

    private boolean modeChange = false;
    private boolean axisChanged = false;
    private boolean gyroRead = false;

    private final String TAG = "Autonomous";

    private static final int sampleRate = 4; //How many samples to take, per second.
    private static final int timeDelay = 1000/sampleRate; //Delay between samples in milliseconds



    private final double EXTENSION_TARGET = 10750;
    private final double LEFT_MINERAL_THETA = 15;
    private final double RIGHT_MINERAL_THETA = -15;

    private final double DEPOT_THETA = -135; //Rotation target for going from


    ElapsedTime elapsedTime = new ElapsedTime();


    public enum CurrentState{
        PREP,               //Preparation for the match, sets up the timers and starts moving the robot
        LOWER,              //Lower from the hook and onto the field
        FORWARD,            //Briefly move forward from the hook so that we're not touching
        EVALUATE_MINERALS,  //Stop scanning for the minerals and start
        TURN_TO_MINERAL,    //Turn to the mineral in order to knock it off
        REMOVE_MINERAL,     //Go forward to knock off the mineral

        CRATER_TURN_TO_DEPOT, //Turn towards the depot, from the crater
        CRATER_NAV_TO_DEPOT,  //Navigate to the depot from the crater

        DEPOT_TURN_TO_DEPOT,
        DEPOT_GO_TO_DEPOT,

        DEPOSIT_OBJECT,
        STOP
    }

    private CurrentState state = CurrentState.PREP;


    @Override
    public void init() {

        robot.init(hardwareMap, false);
    }

    @Override
    public void init_loop(){

        if(!modeChange) {
            robot.gyroConfigMode();
            elapsedTime.reset();
            modeChange = true;
        } else if(!axisChanged && elapsedTime.milliseconds()>200){
            robot.changeAxis();
            elapsedTime.reset();
            axisChanged = true;
        } else if(!gyroRead && axisChanged) {
            robot.gyroReadMode();
            gyroRead = true;
            elapsedTime.reset();
        } else if(gyroRead && elapsedTime.milliseconds() > 200) {
            telemetry.addData("1 Program State: ", state);
            telemetry.addData("2 Robot Heading: ", robot.heading());
        }

    }

    @Override
    public void start() {
        robot.extension.setDirection(DcMotor.Direction.REVERSE);
        robot.RunByEncoders();
        //robot.tfod.deactivate();

    }


    @Override
    public void loop() {

        telemetry.addData("1 Program State: ", state);
        telemetry.addData("2 Robot Heading: ", robot.heading());
        telemetry.addData("3 Left Motor: ", robot.leftWheel.getPower());
        telemetry.addData("4 Right Motor: ", robot.rightWheel.getPower());
        telemetry.addData("5 Lift Motor: ", robot.extension.getPower());
        telemetry.addData("6 Timer", elapsedTime.toString());
        telemetry.addData("7 Mineral Location", robot.mineralState.toString());

        robot.setMineralJoystick(gamepad1);

        switch (state){
            case PREP:
                elapsedTime.reset();
                robot.extension.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.extension.setPower(1.0);
                state = CurrentState.LOWER;
                elapsedTime.reset();
                break;
            case LOWER:
                if (robot.extension.getCurrentPosition() >= EXTENSION_TARGET || elapsedTime.seconds() > 5){
                    elapsedTime.reset();
                    robot.extension.setPower(0.0);
                    state = CurrentState.FORWARD;
                }

                break;
            case FORWARD:         //Moves the robot forward to completely detach from the latch.  May need modification.
                if (robot.leftWheel.getCurrentPosition()>1000 || gamepad1.a){

                    robot.leftWheel.setPower(0.0);
                    robot.rightWheel.setPower(0.0);
                    state = CurrentState.TURN_TO_MINERAL;
                }
                else {
                    robot.leftWheel.setPower(.4);
                    robot.rightWheel.setPower(.4);
                }
                break;

            case TURN_TO_MINERAL:
                switch (robot.mineralState){
                    case GOLD_CENTER:
                        elapsedTime.reset();
                        state = CurrentState.REMOVE_MINERAL;
                        break;
                    case GOLD_LEFT:
                        robot.leftWheel.setPower(-0.3);
                        robot.rightWheel.setPower(0.3);
                        Log.v(TAG, "Now turning to left mineral, robot heading is: " + robot.heading() + ", Target Heading is: " + LEFT_MINERAL_THETA);
                        if (robot.heading() > LEFT_MINERAL_THETA){
                            elapsedTime.reset();
                            state = CurrentState.REMOVE_MINERAL;
                        }
                        break;
                    case GOLD_RIGHT:
                        robot.leftWheel.setPower(0.3);
                        robot.rightWheel.setPower(-0.3);
                        Log.v(TAG, "Now turning to left mineral, robot heading is: " + robot.heading() + ", Target Heading is: " + RIGHT_MINERAL_THETA);
                        if (robot.heading() < RIGHT_MINERAL_THETA){
                            elapsedTime.reset();
                            state = CurrentState.REMOVE_MINERAL;
                        }
                        break;
                    default:
                        elapsedTime.reset();
                        state = CurrentState.REMOVE_MINERAL;
                        break;
                }
                break;

            case REMOVE_MINERAL:
                if (robot.leftWheel.getCurrentPosition()>2200  || gamepad1.dpad_down){
                    robot.rightWheel.setPower(0.0);
                    robot.leftWheel.setPower(0.0);
                    state = CurrentState.STOP;
                }
                else {
                    robot.rightWheel.setPower(0.4);
                    robot.leftWheel.setPower(0.4);
                }
                break;

            case CRATER_TURN_TO_DEPOT:
                state = CurrentState.STOP;
                break;

            case CRATER_NAV_TO_DEPOT:
                state = CurrentState.STOP;
                break;

            case STOP:
                robot.leftWheel.setPower(0.0);
                robot.rightWheel.setPower(0.0);
                robot.extension.setPower(0.0);
                telemetry.addData("Alert:", "You are at the end of the Program");
                break;
        }


    }

    @Override
    public void stop() {

    }
}
