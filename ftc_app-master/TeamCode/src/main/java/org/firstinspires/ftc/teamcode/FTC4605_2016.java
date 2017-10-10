package org.firstinspires.ftc.teamcode;

/**
 * Created by Rob on 10/17/2016.
 */


/* Copyright (c) 2014 Qualcomm Technologies Inc

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Qualcomm Technologies Inc nor the names of its contributors
may be used to endorse or promote products derived from this software without
specific prior written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE. */

//package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Hardware;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import static java.lang.Math.atan2;
import static java.lang.Math.hypot;
import static java.lang.Math.sqrt;
import static org.firstinspires.ftc.teamcode.PathSegRob.DriveType.ENCODER;
import static org.firstinspires.ftc.teamcode.PathSegRob.DriveType.GYRO;
import static org.firstinspires.ftc.teamcode.PathSegRob.DriveType.TIME;
import static org.firstinspires.ftc.teamcode.PathSegRob.DriveType.ENCODER;
import static org.firstinspires.ftc.teamcode.PathSegRob.DriveType.GYRO;
import static org.firstinspires.ftc.teamcode.PathSegRob.DriveType.TIME;
//import static org.firstinspires.ftc.teamcode.PathSegRob.DriveType.GYRO;
//import static org.firstinspires.ftc.teamcode.PathSegRob.DriveType.TIME;

//------------------------------------------------------------------------------
// Extends the OpMode class to provide a Example Autonomous code
//------------------------------------------------------------------------------
/* This opMode does the following steps:
 * 0) Wait till the encoders show reset to zero.
 * 1) Drives to the vicinity of the beacon using encoder counts
 * 2) Use the Legacy light sensor to locate the white line
 * 3) Tracks the line until the wall is reached
 * 4) Pushes up against wall to get square using constant power an time.
 * 5) Deploys the Climbers using the servo
 * 6) Drives to the Mountain using encoder counts
 * 7) Climbs the Mountain using constant speed and time
 * 8) Stops and waits for end of Auto
 *
 * The code is executed as a state machine.  Each "State" performs a specific task which takes time
 * to execute.
 * An "Event" can cause a change in the state.  One or more "Actions" are performed when moving on
 * to next state
 */

@TeleOp(name = "FTC4605_2016", group = "State")
//@Disabled
public class FTC4605_2016 extends OpMode {
    Telemetry telemetry;
    Hardware hwMap;

    // A list of system States.
    private enum DriveStates {
        STATE_INITIAL,
        STATE_1,
        STATE_2,
        STATE_3,
        STATE_4,
        STATE_5,
        STATE_6,
        STATE_STOP,
    }

    private enum FlickerStates {
        wait_to_cycle,
        flick
    }

    // Define path as series of segments. Segment can be time, encoder or gyro based. Each requires a
// different form.
// time = PathSegRob(TIME, speed (-1-+1), turn (-1-+1), strafe (-1-+1), time)
// gyro = PathSegRob(GYRO, speed (-1-+1), turn (-1-+1), strafe (-1-+1), new heading)
// encoder = PathSegRob(ENCODER, speed (-1-+1), turn (-1-+1), strafe (-1-+1), leftFront, leftRear,
//                      rightFront, rightRear)
    final PathSegRob[] TestPath = {
            new PathSegRob(TIME, 0.2, 0, 0, 10.0),
            new PathSegRob(GYRO, 0, 0.2, 0, 20),
            new PathSegRob(TIME, 0, 0, 0.2, 10.0),
            new PathSegRob(ENCODER, 0.2, 0, 0, 10, 10, 10, 10)
    };

    //--------------------------------------------------------------------------
    // Robot device Objects
    //--------------------------------------------------------------------------
    HardwareAlanbot2017 robot = new HardwareAlanbot2017();
    // Loop cycle time stats variables
    public ElapsedTime mRuntime = new ElapsedTime();   // Time into round.

    private DriveStates mCurrentDriveState;    // Current State Machine State.
    private ElapsedTime mDriveStateTime = new ElapsedTime();  // Time into current state

    private FlickerStates CurrentFlickerState;
    private ElapsedTime mFlickerStateTime = new ElapsedTime();

    double Angle;
    double Speed;

    boolean mode = false;

    public void test() {
        double Speed = -gamepad1.left_stick_y;
        double Turn = gamepad1.left_stick_x;
        double Strafe = gamepad1.right_stick_x;
        robot.drive.holonomic(Speed, Turn, Strafe, 1.0);
    }

    //--------------------------------------------------------------------------
    // Demo Hardware
    //--------------------------------------------------------------------------
    public FTC4605_2016() {
    }

    //--------------------------------------------------------------------------
    // init
    //--------------------------------------------------------------------------
    @Override
    public void init() {
        robot.init(hardwareMap, telemetry, gamepad1, gamepad2);
        mRuntime.reset();           // Zero game clock
    }

    //--------------------------------------------------------------------------
    // loop
    //--------------------------------------------------------------------------
    // @Override
    public void init_loop() {
        // Keep resetting encoders and show the current values
        robot.drive.init_loop();
        telemetry.addData("Init loop ", "%4.1f", mRuntime.time());
        if (gamepad1.a){
            mode = true;
        } else {
            mode = false;
        }
    }

    //--------------------------------------------------------------------------
    // start
    //--------------------------------------------------------------------------
    @Override
    public void start() {
        // Setup Robot devices, set initial state and start game clock
        telemetry.addData("start", mRuntime.time());
//        robot.drive.start();
//        robot.sweeper.start();
//        robot.flicker.start();
//        robot.lift.start();
//        robot.beacon.start();
//        robot.sensors.start();
        mRuntime.reset();           // Zero game clock
        if (mode) {
            newDriveState(DriveStates.STATE_INITIAL);
            newFlickerState(FlickerStates.wait_to_cycle);
        }
    }

    //--------------------------------------------------------------------------
    // loop
    //--------------------------------------------------------------------------
    @Override
    public void loop() {
        // Send the current state info (state and time) back to first line of driver station telemetry.
//        telemetry.addData("loop", String.format("%4.1f ", mStateTime.time()) + mCurrentState.toString());
        telemetry.addData("State: ", "%s  Time: %4.1f", mCurrentDriveState, mDriveStateTime.time());
        // Execute the current state.  Each STATE's case code does the following:
        // 1: Look for an EVENT that will cause a STATE change
        // 2: If an EVENT is found, take any required ACTION, and then set the next STATE
        //   else
        // 3: If no EVENT is found, do processing for the current STATE and send TELEMETRY data for STATE.
        //

        if (mode) {

// Drive state machine
            switch (mCurrentDriveState) {
                case STATE_INITIAL:         // Stay in this state until encoders are both Zero.
                    robot.drive.startPath(TestPath);                 // Action: Load path to beacon
                    newDriveState(DriveStates.STATE_1);  // Next State:
//                newDriveState(State.STATE_STOP);  // Next State:
                    telemetry.addData("STATE: ", mCurrentDriveState);
                    break;

                case STATE_1: // Follow path until last segment is completed
                    if (robot.drive.pathComplete())
//                if (gamepad1.a)
                    {
                        newDriveState(DriveStates.STATE_2);      // Next State:
                        robot.sweeper.sweep();
//                    robot.drive.holonomic(0, -0.5, 0);               // Action: Start rotating left
                    } else {
                        // Display Diagnostic data for this state.
                        telemetry.addData("State: ", "%s  Time: %4.1f", mCurrentDriveState,
                                mDriveStateTime.time());
                    }
                    break;

                case STATE_2: // Follow path until last segment is completed
                    if (mDriveStateTime.time() > 2.0) {
                        newDriveState(DriveStates.STATE_3);      // Next State:
                        robot.sweeper.stop();
                        robot.flicker.flick();
                    } else {
                        // Display Diagnostic data for this state.
                        telemetry.addData("State: ", "%s  Time: %4.1f", mCurrentDriveState,
                                mDriveStateTime.time());
                    }
                    break;

                case STATE_3: // Follow path until last segment is completed
                    if (mDriveStateTime.time() > 2.0) {
                        newDriveState(DriveStates.STATE_4);      // Next State:
                        robot.lift.raise();
                    } else {
                        // Display Diagnostic data for this state.
                        telemetry.addData("State: ", "%s  Time: %4.1f", mCurrentDriveState,
                                mDriveStateTime.time());
                    }
                    break;

                case STATE_4: // Follow path until last segment is completed
                    if (mDriveStateTime.time() > 2.0) {
                        newDriveState(DriveStates.STATE_5);      // Next State:
                        robot.lift.stop();
                        robot.drive.startPath(TestPath);                 // Action: Load path to beacon
                    } else {
                        // Display Diagnostic data for this state.
                        telemetry.addData("State: ", "%s  Time: %4.1f", mCurrentDriveState,
                                mDriveStateTime.time());
                    }
                    break;

                case STATE_5: // Follow path until last segment is completed
                    if (robot.drive.pathComplete())
//                if (mDriveStateTime.time() > 2.0)
                    {
                        newDriveState(DriveStates.STATE_STOP);      // Next State:
                        robot.lift.lower();
                    } else {
                        // Display Diagnostic data for this state.
                        telemetry.addData("State: ", "%s  Time: %4.1f", mCurrentDriveState,
                                mDriveStateTime.time());
                    }
                    break;

                case STATE_6: // Follow path until last segment is completed
                    if (robot.drive.pathComplete())
//                if (mDriveStateTime.time() > 2.0)
                    {
                        robot.stop();
                        newDriveState(DriveStates.STATE_STOP);      // Next State:
                    } else {
                        // Display Diagnostic data for this state.
                        telemetry.addData("State: ", "%s  Time: %4.1f", mCurrentDriveState,
                                mDriveStateTime.time());
                    }
                    break;

                case STATE_STOP:
                    // Display Diagnostic data for this state.
                    robot.stop();
                    telemetry.addData("State: ", "%s  Time: %4.1f", mCurrentDriveState,
                            mDriveStateTime.time());
                    break;

            }
        } else {

            telemetry.addData("JoyStick", "%4.2f  %4.2f  %4.2f", gamepad1.left_stick_x,
                    gamepad1.left_stick_y, gamepad1.right_stick_x);

            Angle = atan2(gamepad1.left_stick_x, -gamepad1.left_stick_y);
            Speed = com.qualcomm.robotcore.util.Range.clip(
                    sqrt(hypot(gamepad1.left_stick_x, gamepad1.left_stick_y)), -1.0, 1.0);
            telemetry.addData("Holo2", "%4.2f  %4.2f", Speed, Angle);
            robot.drive.holonomic2(Speed, Angle, gamepad1.right_stick_x);
        }

        switch (CurrentFlickerState) {
            case wait_to_cycle:
                if (gamepad1.right_bumper) {
                    robot.flicker.flick();
                    newFlickerState(FlickerStates.flick);
                }
                break;
            case flick:
                if (!robot.flicker.busy()) {
                    newFlickerState(FlickerStates.wait_to_cycle);
                }
                break;
        }

        if (gamepad1.right_trigger > 0.5) {
            robot.sweeper.sweep();
        } else {
            robot.sweeper.stop();
        }

        if (gamepad1.y) {
            robot.lift.raise();
        } else {
            if (gamepad1.a) {
                robot.lift.lower();

            }
        }
    }

    //--------------------------------------------------------------------------
    // stop
    //--------------------------------------------------------------------------
    @Override
    public void stop() {
        // Ensure that the motors are turned off.
        robot.stop();

    }

    //--------------------------------------------------------------------------
    // User Defined Utility functions here....
    //--------------------------------------------------------------------------
    void test1(double Speed, double Angle, double Turn) {

    }

    //--------------------------------------------------------------------------
    //  Transition to a new state.
    //--------------------------------------------------------------------------
    private void newDriveState(DriveStates newDriveStates) {
        // Reset the state time, and then change to next state.
        mDriveStateTime.reset();
        mCurrentDriveState = newDriveStates;
    }

    //--------------------------------------------------------------------------
    //  Transition to a new state.
    //--------------------------------------------------------------------------
    private void newFlickerState(FlickerStates newFlickerState) {
        // Reset the state time, and then change to next state.
        mFlickerStateTime.reset();
        CurrentFlickerState = newFlickerState;
    }
}
