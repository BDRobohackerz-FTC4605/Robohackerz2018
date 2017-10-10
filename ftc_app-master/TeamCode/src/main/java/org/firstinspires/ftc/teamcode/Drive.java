package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_TO_POSITION;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.STOP_AND_RESET_ENCODER;
import static com.qualcomm.robotcore.util.Range.clip;
import static com.qualcomm.robotcore.util.Range.scale;
import static java.lang.Math.PI;
import static java.lang.Math.abs;
import static java.lang.Math.cos;
import static java.lang.Math.max;
import static java.lang.Math.sin;
import static org.firstinspires.ftc.teamcode.Constants.PULSES_PER_OUTPUT_NEVERREST40;
import static org.firstinspires.ftc.teamcode.Constants.STOP;
import static org.firstinspires.ftc.teamcode.LibraryFTC4605.scaleInput;

/**
 * Created by Rob on 10/17/2016.
 */
public class Drive {

    boolean DEBUG = true;
    boolean NO_DEBUG = false;

    /* drive constants */
    private static final int TRACTION_MOTOR_GEAR_TEETH = 80;
    private static final int TRACTION_AXLE_GEAR_TEETH = 40;
    private static final double WHEEL_DIAMETER = 4.0;

    private static final double PULSES_PER_INCH = (float) ((PULSES_PER_OUTPUT_NEVERREST40
            * (TRACTION_MOTOR_GEAR_TEETH / TRACTION_AXLE_GEAR_TEETH)) / (PI * WHEEL_DIAMETER));
    public HardwareAlanbot2017 robot = null;

    private HardwareMap hwMap = null;
    private Telemetry telemetry = null;
    private DcMotor.Direction leftDirection = null;
    private DcMotor.Direction rightDirection = null;

    private DcMotor leftFrontMotor = null;
    private DcMotor leftRearMotor = null;
    private DcMotor rightFrontMotor = null;
    private DcMotor rightRearMotor = null;

    private int leftFrontEncoderTarget;
    private int leftRearEncoderTarget;
    private int rightFrontEncoderTarget;
    private int rightRearEncoderTarget;

    private ElapsedTime timerState = new ElapsedTime();

    private enum State {STATE_INITIAL, STATE_RUN, STATE_STOP}

    private State currentState;

    private GyroSensor gyro;

    public Drive() {
    }

    public void init(HardwareMap hwMap, Telemetry telemetry, DcMotor.Direction leftDirection,
                     DcMotor.Direction rightDirection) {
        this.hwMap = hwMap;
        this.telemetry = telemetry;
        this.leftDirection = leftDirection;
        this.rightDirection = rightDirection;

        init(hwMap, telemetry, leftDirection, rightDirection, false);
    }

        public void init(HardwareMap hwMap, Telemetry telemetry, DcMotor.Direction leftDirection,
                         DcMotor.Direction rightDirection, boolean DEBUG) {
            this.hwMap = hwMap;
            this.telemetry = telemetry;
            this.leftDirection = leftDirection;
            this.rightDirection = rightDirection;
            this.DEBUG = DEBUG;

        leftFrontMotor = hwMap.dcMotor.get("frontLeft");
        leftFrontMotor.setPower(STOP);
        leftFrontMotor.setDirection(leftDirection);
        leftFrontMotor.setMode(STOP_AND_RESET_ENCODER);

        if (hwMap.dcMotor.get("backLeft") != null) {
            leftRearMotor = hwMap.dcMotor.get("backLeft");
            leftRearMotor.setPower(STOP);
            leftRearMotor.setDirection(leftDirection);
            leftRearMotor.setMode(STOP_AND_RESET_ENCODER);
        }

        rightFrontMotor = hwMap.dcMotor.get("frontRight");
        rightFrontMotor.setPower(STOP);
        rightFrontMotor.setDirection(rightDirection);
        rightFrontMotor.setMode(STOP_AND_RESET_ENCODER);

        if (hwMap.dcMotor.get("backRight") != null) {
            rightRearMotor = hwMap.dcMotor.get("backRight");
            rightRearMotor.setPower(STOP);
            rightRearMotor.setDirection(rightDirection);
            rightRearMotor.setMode(STOP_AND_RESET_ENCODER);
        }

        if (hwMap.gyroSensor.get("gyro") != null) {
            gyro = hwMap.gyroSensor.get("gyro");
            gyro.calibrate();
        }
    }

    public void init_loop() {
    }

    public void start() {
        newState(State.STATE_STOP);
    }

    public void stop() {
        useConstantSpeed();
        setDriveSpeed(0, 0, 0, 0);

        if(DEBUG) {
            telemetry.addData("", "STOP");
            telemetry.addData("", "%4.2f  %4.2f", leftFrontMotor.getPower(), rightFrontMotor.getPower());
            telemetry.addData("", "%4.2f  %4.2f", leftRearMotor.getPower(), rightRearMotor.getPower());
        }
    }

    public void tank(double LeftSpeed, double RightSpeed, double MAX_SPEED) {
        leftFrontMotor.setPower(clip(scaleInput(LeftSpeed), -MAX_SPEED, +MAX_SPEED));
        if (leftRearMotor != null) {
            leftRearMotor.setPower(clip(scaleInput(LeftSpeed), -MAX_SPEED, +MAX_SPEED));
        }
        rightFrontMotor.setPower(clip(scaleInput(RightSpeed), -MAX_SPEED, MAX_SPEED));
        if (rightRearMotor != null) {
            rightRearMotor.setPower(clip(scaleInput(RightSpeed), -MAX_SPEED, MAX_SPEED));

            if(DEBUG) {
                telemetry.addData("tank", "%4.2f  %4.2f", LeftSpeed, RightSpeed);
                telemetry.addData("", "%4.2f  %4.2f", leftFrontMotor.getPower(), rightFrontMotor.getPower());
                telemetry.addData("", "%4.2f  %4.2f", leftRearMotor.getPower(), rightRearMotor.getPower());
            }
        }
    }

    public void tank(double LeftSpeed, double RightSpeed) {
        tank(LeftSpeed, RightSpeed, 1);

    }

    public void arcade(double Speed, double Turn, double MAX_SPEED) {
        holonomic(clip(scaleInput(Speed), -MAX_SPEED, MAX_SPEED), clip(scaleInput(Turn), -MAX_SPEED, MAX_SPEED), 0.0);

    }

    public void arcade(double Speed, double Turn) {
        holonomic(Speed, Turn, 0.0);

    }

    public void POV(double Speed, double Turn, double MAX_SPEED) {
        holonomic(clip(scaleInput(Speed), -MAX_SPEED, MAX_SPEED), clip(scaleInput(Turn), -MAX_SPEED, MAX_SPEED), 0.0);

    }

    public void POV(double Speed, double Turn) {
        holonomic(Speed, Turn, 0.0);

    }

    public void holonomic(double Speed, double Turn, double Strafe, double MAX_SPEED) {

//      Left Front = +Speed + Turn - Strafe      Right Front = +Speed - Turn + Strafe
//      Left Rear  = +Speed + Turn + Strafe      Right Rear  = +Speed - Turn - Strafe
        double Magnitude = abs(Speed) + abs(Turn) + abs(Strafe);
        Magnitude = (Magnitude > 1) ? Magnitude : 1; //Set scaling to keep -1,+1 range

        leftFrontMotor.setPower(scale((scaleInput(Speed) + scaleInput(Turn) - scaleInput(Strafe)),
                -Magnitude, +Magnitude, -MAX_SPEED, +MAX_SPEED));
        if (leftRearMotor != null) {
            leftRearMotor.setPower(scale((scaleInput(Speed) + scaleInput(Turn) + scaleInput(Strafe)),
                    -Magnitude, +Magnitude, -MAX_SPEED, +MAX_SPEED));
        }
        rightFrontMotor.setPower(scale((scaleInput(Speed) - scaleInput(Turn) + scaleInput(Strafe)),
                -Magnitude, +Magnitude, -MAX_SPEED, +MAX_SPEED));
        if (rightRearMotor != null) {
            rightRearMotor.setPower(scale((scaleInput(Speed) - scaleInput(Turn) - scaleInput(Strafe)),
                    -Magnitude, +Magnitude, -MAX_SPEED, +MAX_SPEED));
        }

        if(DEBUG) {
            telemetry.addData("holo", "%4.2f  %4.2f  %4.2f", Speed, Turn, Strafe);
            telemetry.addData("", "%4.2f  %4.2f", leftFrontMotor.getPower(), rightFrontMotor.getPower());
            telemetry.addData("", "%4.2f  %4.2f", leftRearMotor.getPower(), rightRearMotor.getPower());
        }
    }

    public void holonomic(double Speed, double Turn, double Strafe) {
        holonomic(Speed, Turn, Strafe, 1.0);

    }

    public void holonomic2(double Speed, double Angle, double Turn) {
        // Speed is speed the robot will translate (-1, 1)
        // Angle is the angle the robot will move at (Speed + Strafe) (0, 2pi)
        // Turn is the speed/direction of rotation (-1, 1)
        double dleftFrontMotor = Speed * sin(Angle/* + PI/4*/) + Turn;
        double dleftRearMotor = (Speed * cos(Angle/* + PI/4*/) - Turn);
        double drightFrontMotor = (Speed * cos(Angle/* + PI/4*/) + Turn);
        double drightRearMotor = (Speed * sin(Angle/* + PI/4*/) - Turn);

        //scale to keep motor input (-1, 1)
        double Max;
        Max = max(abs(dleftFrontMotor), abs(dleftRearMotor));
        Max = max(Max, abs(drightFrontMotor));
        Max = max(Max, abs(drightRearMotor));
        Max = (Max > 1) ? Max : 1;

        leftFrontMotor.setPower(dleftFrontMotor / Max);
        leftRearMotor.setPower(dleftRearMotor / Max);
        rightFrontMotor.setPower(drightFrontMotor / Max);
        rightRearMotor.setPower(drightRearMotor / Max);

        telemetry.addData("", "%4.5f  %4.5f", dleftFrontMotor / Max, drightFrontMotor / Max);
        telemetry.addData("", "%4.5f  %4.5f", dleftRearMotor / Max, drightRearMotor / Max);

        telemetry.addData("", "%4.5f  %4.5f", dleftFrontMotor, drightFrontMotor);
        telemetry.addData("", "%4.5f  %4.5f", dleftRearMotor, drightRearMotor);
        telemetry.addData("Max", Max);

        if(DEBUG) {
            telemetry.addData("holo2 ", "%4.2f  %4.2f  %4.2f", Speed, Angle, Turn);
            telemetry.addData("", "%4.2f  %4.2f", leftFrontMotor.getPower(), rightFrontMotor.getPower());
            telemetry.addData("", "%4.2f  %4.2f", leftRearMotor.getPower(), rightRearMotor.getPower());
        }
    }

    public void holonomicVuforia(double Speed, double Angle, double Turn, double heading) {

    }

    //--------------------------------------------------------------------------
    // setEncoderTarget( LeftEncoder, RightEncoder);
    // Sets Absolute Encoder Position
    //--------------------------------------------------------------------------
    public void setEncoderTarget(int leftFrontEncoder, int leftRearEncoder, int rightFrontEncoder, int rightRearEncoder) {
//        if ((leftFrontMotor.getMode() == RUN_TO_POSITION) &&
//                (leftRearMotor.getMode() == RUN_TO_POSITION) &&
//                (rightFrontMotor.getMode() == RUN_TO_POSITION) &&
//                (rightRearMotor.getMode() == RUN_TO_POSITION)) {
        leftFrontMotor.setTargetPosition(leftFrontEncoderTarget = leftFrontEncoder);
        leftRearMotor.setTargetPosition(leftRearEncoderTarget = leftRearEncoder);
        rightFrontMotor.setTargetPosition(rightFrontEncoderTarget = rightFrontEncoder);
        rightRearMotor.setTargetPosition(rightRearEncoderTarget = rightRearEncoder);
//        } else {
//            //throw.error
//
//        }
    }

    //--------------------------------------------------------------------------
    // addEncoderTarget( LeftEncoder, RightEncoder);
    // Sets relative Encoder Position.  Offset current targets with passed data
    //--------------------------------------------------------------------------
    public void addEncoderTarget(int leftFrontEncoder, int leftRearEncoder, int rightFrontEncoder, int rightRearEncoder) {
//        if ((leftFrontMotor.getMode() == RUN_TO_POSITION) &&
//                (leftRearMotor.getMode() == RUN_TO_POSITION) &&
//                (rightFrontMotor.getMode() == RUN_TO_POSITION) &&
//                (rightRearMotor.getMode() == RUN_TO_POSITION)) {
        leftFrontMotor.setTargetPosition(leftFrontEncoderTarget += leftFrontEncoder);
        leftRearMotor.setTargetPosition(leftRearEncoderTarget += leftRearEncoder);
        rightFrontMotor.setTargetPosition(rightFrontEncoderTarget += rightFrontEncoder);
        rightRearMotor.setTargetPosition(rightRearEncoderTarget += rightRearEncoder);
//        } else {
//            //throw.error
//
//        }
    }

    //--------------------------------------------------------------------------
    // setDrivePower( LeftPower, RightPower);
    //--------------------------------------------------------------------------
    public void setDrivePower(double leftFrontPower, double leftRearPower, double rightFrontPower, double rightRearPower) {
//        if ((leftFrontMotor.getMode() == RUN_WITHOUT_ENCODER) &&
//                (leftRearMotor.getMode() == RUN_WITHOUT_ENCODER) &&
//                (rightFrontMotor.getMode() == RUN_WITHOUT_ENCODER) &&
//                (rightRearMotor.getMode() == RUN_WITHOUT_ENCODER)) {
        leftFrontMotor.setPower(leftFrontPower);
        leftRearMotor.setPower(leftRearPower);
        rightFrontMotor.setPower(rightFrontPower);
        rightRearMotor.setPower(rightRearPower);
//        } else {
//            //throw.error
//
//        }
    }

    public double getDrivePowerLeftFront() {
        return leftFrontMotor.getPower();
    }

    public double getDrivePowerLeftRear() {
        return leftRearMotor.getPower();
    }

    public double getDrivePowerRightFront() {
        return rightFrontMotor.getPower();
    }

    public double getDrivePowerRightRear() {
        return rightRearMotor.getPower();
    }

    //--------------------------------------------------------------------------
    // setDriveSpeed( LeftSpeed, RightSpeed);
    //--------------------------------------------------------------------------
    public void setDriveSpeed(double leftFrontSpeed, double leftRearSpeed, double rightFrontSpeed, double rightRearSpeed) {
//        if ((leftFrontMotor.getMode() == RUN_USING_ENCODER) &&
//                (leftRearMotor.getMode() == RUN_USING_ENCODER) &&
//                (rightFrontMotor.getMode() == RUN_USING_ENCODER) &&
//                (rightRearMotor.getMode() == RUN_USING_ENCODER)) {
        setDrivePower(leftFrontSpeed, leftRearSpeed, rightFrontSpeed, rightRearSpeed);
//        } else {
//            //throw.error
//
//        }
    }

    //--------------------------------------------------------------------------
    // runToPosition ()
    // Set both drive motors to encoder servo mode (requires encoders)
    //--------------------------------------------------------------------------
    public void runToPosition() {
        setDriveMode(DcMotor.RunMode.RUN_TO_POSITION);

    }

    //--------------------------------------------------------------------------
    // useConstantSpeed ()
    // Set both drive motors to constant speed (requires encoders)
    //--------------------------------------------------------------------------
    public void useConstantSpeed() {
        setDriveMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

    //--------------------------------------------------------------------------
    // useConstantPower ()
    // Set both drive motors to constant power (encoders NOT required)
    //--------------------------------------------------------------------------
    public void useConstantPower() {
        setDriveMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    }

    //--------------------------------------------------------------------------
    // resetDriveEncoders()
    // Reset both drive motor encoders, and clear current encoder targets.
    //--------------------------------------------------------------------------
    public void resetDriveEncoders() {
        setEncoderTarget(0, 0, 0, 0);
        setDriveMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    //--------------------------------------------------------------------------
    // syncEncoders()
    // Load the current encoder values into the Target Values
    // Essentially synch's the software with the hardware
    //--------------------------------------------------------------------------
    public void synchEncoders() {
        //	get and set the encoder targets
        leftFrontEncoderTarget = leftFrontMotor.getCurrentPosition();
        leftRearEncoderTarget = leftRearMotor.getCurrentPosition();
        rightFrontEncoderTarget = rightFrontMotor.getCurrentPosition();
        rightRearEncoderTarget = rightRearMotor.getCurrentPosition();
    }

    //--------------------------------------------------------------------------
    // setDriveMode ()
    // Set both drive motors to new mode if they need changing.
    //--------------------------------------------------------------------------
    public void setDriveMode(DcMotor.RunMode mode) {
        // Ensure the motors are in the correct mode.
        if (leftFrontMotor.getMode() != mode)
            leftFrontMotor.setMode(mode);

        if (leftRearMotor.getMode() != mode)
            leftRearMotor.setMode(mode);

        if (rightFrontMotor.getMode() != mode)
            rightFrontMotor.setMode(mode);

        if (rightRearMotor.getMode() != mode)
            rightRearMotor.setMode(mode);
    }

    //--------------------------------------------------------------------------
    // getLeftFrontPosition ()
    // Return Left Front Encoder count
    //--------------------------------------------------------------------------
    public int getLeftFrontPosition() {
        return leftFrontMotor.getCurrentPosition();

    }

    //--------------------------------------------------------------------------
    // getLeftRearPosition ()
    // Return Left Front Encoder count
    //--------------------------------------------------------------------------
    public int getLeftRearPosition() {
        return leftRearMotor.getCurrentPosition();

    }

    //--------------------------------------------------------------------------
    // getRightFrontPosition ()
    // Return Right Front Encoder count
    //--------------------------------------------------------------------------
    public int getRightFrontPosition() {
        return rightFrontMotor.getCurrentPosition();

    }

    //--------------------------------------------------------------------------
    // getRightRearPosition ()
    // Return Right Rear Encoder count
    //--------------------------------------------------------------------------
    public int getRightRearPosition() {
        return rightRearMotor.getCurrentPosition();

    }

    //--------------------------------------------------------------------------
    // moveComplete()
    // Return true if motors have both reached the desired encoder target
    //--------------------------------------------------------------------------
    public boolean moveComplete() {
        //  return (!leftFrontMotor.isBusy()v && !leftRearMotor.isBusy() &&
        // !RightFrontMotor.isBusy() && !RightRearMotor.isBusy());
        return ((Math.abs(getLeftFrontPosition() - leftFrontEncoderTarget) < 10) &&
                (Math.abs(getLeftRearPosition() - leftRearEncoderTarget) < 10) &&
                (Math.abs(getRightFrontPosition() - rightFrontEncoderTarget) < 10) &&
                (Math.abs(getRightRearPosition() - rightRearEncoderTarget) < 10));
    }

    //--------------------------------------------------------------------------
    // encodersAtZero()
    // Return true if both encoders read zero (or close)
    //--------------------------------------------------------------------------
    public boolean encodersAtZero() {
        return ((Math.abs(getLeftFrontPosition()) < 5) && (Math.abs(getLeftRearPosition()) < 5) &&
                (Math.abs(getRightFrontPosition()) < 5) && (Math.abs(getRightRearPosition()) < 5));
    }

    private PathSegRob[] mCurrentPath;     // Array to hold current path
    private int mCurrentSeg;      // Index of the current leg in the current path

    /*
        Begin the first leg of the path array that is passed tatin.
        Calls startSeg() to actually load the encoder targets.
     */
    public void startPath(PathSegRob[] path) {
        mCurrentPath = path;    // Initialize path array
        mCurrentSeg = 0;
//            synchEncoders();        // Lock in the current position
//            runToPosition();        // Enable RunToPosition mode
        newState(State.STATE_INITIAL);
        startSeg();             // Execute the current (first) Leg
        telemetry.addData("startPath", mCurrentPath);
    }

    /*
        Starts the current leg of the current path.
        Must call startPath() once before calling this
        Each leg adds the new relative movement onto the running encoder totals.
        By not reading and using the actual encoder values, this avoids accumulating errors.
        Increments the leg number after loading the current encoder targets
     */
    private void startSeg() {
        if (mCurrentPath != null) {
//                Load up the next motion based on the current segment.
//                telemetry.addData("mCurrentPath ", mCurrentPath[mCurrentSeg].toString());
            telemetry.addData("mCurrentSeg ", mCurrentSeg);
            telemetry.addData("timerState ", timerState.time());

            switch (mCurrentPath[mCurrentSeg].Type) {

                case TIME:
//                    DriveType Type, double Speed, double Turn, double Strafe, double Time)

                    switch (currentState) {
                        case STATE_INITIAL:
                            telemetry.addData("startSeg:TIME:STATE_INITIAL", timerState.time());
                            holonomic(mCurrentPath[mCurrentSeg].Speed,
                                    mCurrentPath[mCurrentSeg].Turn,
                                    mCurrentPath[mCurrentSeg].Strafe);
                            newState(State.STATE_RUN);
                            break;

                        case STATE_RUN:
                            if (timerState.time() > mCurrentPath[mCurrentSeg].Time) {
                                stop();
                                mCurrentSeg++;  // Move index to next segment of path
                                newState(State.STATE_INITIAL);
                            } else {
                                telemetry.addData("startSeg:TIME:STATE_RUN", timerState.time());
                            }
                            break;

                        case STATE_STOP: {
                            stop();
                            break;
                        }
                    }
                    break;

                case ENCODER:
//                    DriveType Type, double Speed, double Turn, double Strafe,
//                    int leftFrontEncoder, int leftRearEncoder, int rightFrontEncoder,
//                    int rightRearEncoder

                    switch (currentState) {

                        case STATE_INITIAL:
                            setDriveMode(RUN_TO_POSITION);
                            addEncoderTarget(mCurrentPath[mCurrentSeg].leftFrontEncoder * (int) PULSES_PER_INCH,
                                    mCurrentPath[mCurrentSeg].leftRearEncoder * (int) PULSES_PER_INCH,
                                    mCurrentPath[mCurrentSeg].rightFrontEncoder * (int) PULSES_PER_INCH,
                                    mCurrentPath[mCurrentSeg].rightRearEncoder * (int) PULSES_PER_INCH);
                            setDrivePower(mCurrentPath[mCurrentSeg].Speed,
                                    mCurrentPath[mCurrentSeg].Speed,
                                    mCurrentPath[mCurrentSeg].Speed,
                                    mCurrentPath[mCurrentSeg].Speed);
                            newState(State.STATE_RUN);
                            telemetry.addData("startSeg:ENCODER:STATE_INITIAL", timerState.time());
                            telemetry.addData("Encoders - Front: ", "%6d  %6d", getLeftFrontPosition(), getRightFrontPosition());
                            telemetry.addData("Encoders - Rear:  ", "%6d  %6d", getLeftRearPosition(), getRightRearPosition());

                            break;

                        case STATE_RUN:
//                                if(gamepad1.a){
                            if ((!leftFrontMotor.isBusy() && !leftRearMotor.isBusy() &&
                                    !rightFrontMotor.isBusy() && !rightRearMotor.isBusy()) || (timerState.time() > 2)) {
                                stop();
                                mCurrentSeg++;  // Move index to next segment of path
                                newState(State.STATE_INITIAL);
                            } else {
                                telemetry.addData("startSeg:ENCODER:STATE_RUN", timerState.time());
                                telemetry.addData("Encoders - Front: ", "%6d  %6d", getLeftFrontPosition(), getRightFrontPosition());
                                telemetry.addData("Encoders - Rear:  ", "%6d  %6d", getLeftRearPosition(), getRightRearPosition());
                            }
                            break;

                        case STATE_STOP: {
                            stop();
                            break;
                        }
                    }
                    break;

                case GYRO:
//                    DriveType Type, double speed, double turn, double strafe, int heading)

//                        int headingError = -((int)(gamepad1.left_stick_x * 360) - mCurrentPath[mCurrentSeg].Heading);
                    int headingError = -(gyro.getHeading() - mCurrentPath[mCurrentSeg].Heading);
                    headingError = (headingError > 180) ? headingError - 360 : headingError;
                    headingError = (headingError < -180) ? headingError + 360 : headingError;

                    if (headingError > 5) {
                        holonomic(0, -mCurrentPath[mCurrentSeg].Turn, 0);
                    } else {
                        if (headingError < -5) {
                            holonomic(0, mCurrentPath[mCurrentSeg].Turn, 0);
                        } else {
                            stop();
                            mCurrentSeg++;
                        }
                    }
                    telemetry.addData("startSeg:GYRO ", "%4d   %4d", mCurrentPath[mCurrentSeg].Heading, gyro.getHeading());
                    break;

                case DES_ORI:
                    //DriveType Type, double Speed, Destination Destination, double Turn, int Orientation
//                    double direction = atan2(mCurrentPath[mCurrentSeg].Destination.Y, mCurrentPath[mCurrentSeg].Destination.X);
//                    int OrientationBot = mCurrentPath[mCurrentSeg].Orientation -  gyro.getHeading();
//                    holonomic2(mCurrentPath[mCurrentSeg].Speed, direction, OrientationBot);

                    break;
            }
        }
    }

    /*
        Determines if the current path is complete
        As each segment completes, the next segment is started unless there are no more.
        Returns true if the last leg has completed and the robot is stopped.
     */
    public boolean pathComplete() {
        // Wait for this Segement to end and then see what's next.
        if (moveComplete()) {
            // Start next Segement if there is one.
            if (mCurrentSeg < mCurrentPath.length) {
                startSeg();
            } else  // Otherwise, stop and return done
            {
                mCurrentPath = null;
                mCurrentSeg = 0;
                stop();
                newState(State.STATE_STOP);
                return true;
            }
        }
        return false;
    }


    //--------------------------------------------------------------------------
    //  Transition to a new state.
    //--------------------------------------------------------------------------
    private void newState(State newState) {
        // Reset the state time, and then change to next state.
        timerState.reset();
        currentState = newState;
    }
}
