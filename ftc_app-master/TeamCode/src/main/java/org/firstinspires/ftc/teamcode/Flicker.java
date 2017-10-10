package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
//import org.firstinspires.ftc.teamcode.Autonomous.PathSeg;

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_TO_POSITION;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_WITHOUT_ENCODER;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.STOP_AND_RESET_ENCODER;
import static org.firstinspires.ftc.teamcode.Constants.PULSES_PER_OUTPUT_NEVERREST60;
import static org.firstinspires.ftc.teamcode.Constants.STOP;
import static org.firstinspires.ftc.teamcode.Constants.PULSES_PER_OUTPUT_NEVERREST60;
import static org.firstinspires.ftc.teamcode.Constants.STOP;

/**
 * Created by Rob on 10/17/2016.
 */
public class Flicker {
    // This define the Flicker used to "flick" the particle into the center goal.
    //

    private final int GEAR_IN = 40;
    private final int GEAR_OUT = 80;
    private final int PULSES_PER_REVOLUTION = PULSES_PER_OUTPUT_NEVERREST60 *
            GEAR_OUT / GEAR_IN;

    private Telemetry telemetry = null;

    public final double DEFAULT_SPEED = 1;
    private final int RETRACT = 45;  // degrees to retract
    private final double INITIALIZE_POWER = -0.1;

    private DcMotor flickerMotor = null;
    private DcMotor.Direction direction = null;
    public int flickerCounter = 0;
    public int flickerState = 0;

    double lastEncoderValue;
    double currentEncoderValue;

    private enum State {
        STATE_INITIAL,
        STATE_POWER_TO_STALL,
        STATE_RETRACT,
        STATE_STOP,
    }

    // Loop cycle time stats variables
    public ElapsedTime mRuntime = new ElapsedTime();   // Time into round.

    private ElapsedTime mStateTime = new ElapsedTime();  // Time into current state

    private State mCurrentState;    // Current State Machine State.
//    private PathSeg[] mCurrentPath;     // Array to hold current path
    private int mCurrentSeg;      // Index of the current leg in the current path

    public Flicker() {

    }

    public void init(HardwareMap hwMap, Telemetry telemetryIn, DcMotor.Direction direction) { //throws InterruptedException {
        // This define the Flicker
        // hwmap - hardware map for robot
        // direction - what is direction for flicker to work
        // revolution - what is the encoder count for one shaft revolution

        this.telemetry = telemetryIn;

        flickerMotor = hwMap.dcMotor.get("flicker_motor");
        flickerMotor.setPower(STOP);
        flickerMotor.setDirection(direction);
        flickerMotor.setMode(STOP_AND_RESET_ENCODER);
        mCurrentState = State.STATE_INITIAL;
        initialise();
    }

    public void init(HardwareMap hwMap, Telemetry telemetryIn) {
        this.telemetry = telemetryIn;

        flickerMotor = hwMap.dcMotor.get("flicker_motor");
        flickerMotor.setPower(STOP);
        flickerMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        flickerMotor.setMode(STOP_AND_RESET_ENCODER);
        mCurrentState = State.STATE_INITIAL;
        initialise();
    }

    public void init_loop() {
    }

    public void start() {
    }

    public void loop() {
    }

    public void stop() {
    }

    private void initialise() {

//        lastEncoderValue = flickerMotor.getCurrentPosition();
        currentEncoderValue = flickerMotor.getCurrentPosition();
        flickerMotor.setMode(RUN_WITHOUT_ENCODER);
        flickerMotor.setPower(INITIALIZE_POWER);

        do {
            lastEncoderValue = currentEncoderValue;
        } while (Math.abs(lastEncoderValue - currentEncoderValue) > 10);

        flickerMotor.setPower(STOP);
        flickerMotor.setMode(RUN_TO_POSITION);
        flickerMotor.setTargetPosition(flickerMotor.getCurrentPosition() -
                (RETRACT * PULSES_PER_REVOLUTION));
        flickerMotor.setPower(INITIALIZE_POWER);
        flickerMotor.setMode(STOP_AND_RESET_ENCODER);
        flickerMotor.setMode(RUN_TO_POSITION);

//        flickerCounter++;
//        switch (mCurrentState)
//        {
//            case STATE_INITIAL:         // Stay in this state until encoders are both Zero.
//                if (true)
//                {
//                    flickerState = 0;
//                    currentEncoderValue = flickerMotor.getCurrentPosition();
//                    flickerMotor.setMode(RUN_USING_ENCODER);
//                    flickerMotor.setPower(INITIALIZE_POWER);
//                    newState(State.STATE_POWER_TO_STALL);  // Next State:
//                }
//                else
//                {
//                    // Display Diagnostic data for this state.
//                    telemetry.addData("1", String.format("L %5d - R %5d "));
//                }
//
//                break;
//
//            case STATE_POWER_TO_STALL: // Follow path until last segment is completed
////                if (Math.abs(lastEncoderValue - currentEncoderValue) > 10 )
//                if (flickerState > 100 )
//                {
//                    flickerMotor.setPower(STOP);
//                    flickerMotor.setMode(RUN_TO_POSITION);
//                    flickerMotor.setTargetPosition(flickerMotor.getCurrentPosition() - RETRACT);
//                    newState(State.STATE_RETRACT);      // Next State:
//                }
//                else
//                {
//                    flickerState++;
//                    // Display Diagnostic data for this state.
//                    telemetry.addData("1", String.format("%d of %d.", mCurrentSeg, mCurrentPath.length));
//                }
//                break;
//
//            case STATE_RETRACT: // Follow path until last segment is completed
//                if (!flickerMotor.isBusy())
//                {
//                    flickerMotor.setMode(STOP_AND_RESET_ENCODER);
//                    flickerMotor.setMode(RUN_TO_POSITION);
//                    newState(State.STATE_STOP);      // Next State:
//                }
//                else
//                {
//                    flickerState++;
//                    // Display Diagnostic data for this state.
//                    telemetry.addData("1", String.format("%d of %d. ", mCurrentSeg, mCurrentPath.length));
//                }
//                break;
//
//            case STATE_STOP:
//                break;
//        }
    }

    public void flick(double power) {
        flickerMotor.setMode(RUN_TO_POSITION);
        flickerMotor.setPower(power);
        flickerMotor.setTargetPosition((flickerMotor.getCurrentPosition() +
                PULSES_PER_REVOLUTION));
        telemetry.addData("Flicker-flick(param)", "");
    }

    public void flick() {
        flickerMotor.setMode(RUN_TO_POSITION);
        flickerMotor.setPower(DEFAULT_SPEED);
        flickerMotor.setTargetPosition((flickerMotor.getCurrentPosition() +
                PULSES_PER_REVOLUTION));
        telemetry.addData("Flicker-flick(default)", "");
    }

    public boolean busy() {
        return (flickerMotor.isBusy());

    }
}
