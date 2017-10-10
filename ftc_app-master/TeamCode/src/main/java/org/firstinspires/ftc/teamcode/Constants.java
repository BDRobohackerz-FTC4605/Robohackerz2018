package org.firstinspires.ftc.teamcode;

/**
 * Created by Rob on 11/22/2016.
 */
public class Constants {

    //universal

    protected static final float MM_PER_INCH = 25.4f;

    // motor constants
    protected static final int PULSES_PER_OUTPUT_NEVERREST3_7 = 103; // NeverRest20
    protected static final int PULSES_PER_OUTPUT_NEVERREST20 = 560; // NeverRest20
    protected static final int PULSES_PER_OUTPUT_NEVERREST40 = 1120; // NeverRest40
    protected static final int PULSES_PER_OUTPUT_NEVERREST60 = 1680; // NeverRest60

    protected static final int PULSES_PER_OUTPUT_TETRIX = 1440; // Tetrix

    protected static final int PULSES_PER_OUTPUT_REV = 2240; // Rev Robotics


    //colors
    static enum FtcColor {RED, BLUE};

    //robot
    protected static final float ROBOT_WIDTH = 18 * MM_PER_INCH;               // mm

    //field
    protected static final float FTC_FIELD_WIDTH = (12 * 12 - 2) * MM_PER_INCH;  // mm
    protected static final float FTC_FIELD_HEIGHT = 200.0f;

    //Vuforia targets
    protected static final float TARGET_HEIGHT = 160.0f;                     // mm

    //convenience
    protected static final double STOP = 0;

    //gamepad buttons
    static final int GAMEPAD1_X = 1;
    static final int GAMEPAD1_A = 2;
    static final int GAMEPAD1_B = 3;
    static final int GAMEPAD1_Y = 4;
    static final int GAMEPAD1_LB = 5;
    static final int GAMEPAD1_RB = 6;
    static final int GAMEPAD1_LT = 7;
    static final int GAMEPAD1_RT = 8;
    static final int GAMEPAD1_BK = 9;
    static final int GAMEPAD1_START = 10;
    static final int GAMEPAD1_LJ = 11;
    static final int GAMEPAD1_RJ = 12;
    static final int GAMEPAD1_DU = 13;
    static final int GAMEPAD1_DR = 14;
    static final int GAMEPAD1_DD = 15;
    static final int GAMEPAD1_DL = 16;

    static final int GAMEPAD2_X = 21;
    static final int GAMEPAD2_A = 22;
    static final int GAMEPAD2_B = 23;
    static final int GAMEPAD2_Y = 24;
    static final int GAMEPAD2_LB = 25;
    static final int GAMEPAD2_RB = 26;
    static final int GAMEPAD2_LT = 27;
    static final int GAMEPAD2_RT = 28;
    static final int GAMEPAD2_BK = 29;
    static final int GAMEPAD2_START = 30;
    static final int GAMEPAD2_LJ = 31;
    static final int GAMEPAD2_RJ = 32;
    static final int GAMEPAD2_DU = 33;
    static final int GAMEPAD2_DR = 34;
    static final int GAMEPAD2_DD = 35;
    static final int GAMEPAD2_DL = 36;
}
