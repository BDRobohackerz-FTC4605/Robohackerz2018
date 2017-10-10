package org.firstinspires.ftc.teamcode;

/**
 * Created by Rob on 9/12/2016.
 */
public class LibraryFTC4605 {

//    public static final int PULSES_PER_OUTPUT_NEVERREST20 = 560; // NeverRest20
//    public static final int PULSES_PER_OUTPUT_NEVERREST40 = 1120; // NeverRest40
//    public static final int PULSES_PER_OUTPUT_NEVERREST60 = 1680; // NeverRest60
//    public static final int PULSES_PER_OUTPUT_TETRIX = 1440; // Tetrix

//
//    {
//        public static boolean[][] buttonCheckingNew = {{false, false, false, false, false, false, false, false, false, false, false, false, false, false, false, false},
//                                                       {false, false, false, false, false, false, false, false, false, false, false, false, false, false, false, false}};
//        //                                             {0    , 1    , 2    , 3    , 4    , 5    , 6    , 7    , 8    , 9    , 10   , 11   , 12   , 13   , 14   , 15   }
//        //                                             {X    , A    , B    , Y    , LB   , RB   , LT   , RT   , BK   , ST   , LJ   , RJ   , DU   , DR,  , DD,  , DL   }
//        //                                             {     ,      ,      ,      ,      ,      ,      ,      , BACK , START,      ,      ,      ,      ,      ,      }
//        static final int gamepad1 = 0;
//        static final int gamepad2 = 1;
//        static final int X = 0;
//        static final int A = 1;
//        static final int B = 2;
//        static final int Y = 3;
//        static final int LB = 4;
//        static final int RB = 5;
//        static final int LT = 6;
//        static final int RT = 7;
//        static final int BK = 8;
//        static final int BACK = 8;
//        static final int ST = 9;
//        static final int START = 9;
//        static final int LJ = 10;
//        static final int RJ = 11;
//        static final int DU = 12;
//        static final int DR = 13;
//        static final int DD = 14;
//        static final int DL = 15;
//
//    }


    public static boolean[] buttonChecking = {false, false, false, false, false, false, false, false, false, false, false, false, false, false, false, false, false, false, false, false, false};
    //                                       {1    , 2    , 3    , 4    , 5    , 6    , 7    , 8    , 9    , 10   , 11   , 12   , 13   , 14   , 15   , 16   , 17   , 18   , 19   , 20   }
    //                                       {1x   ,      , 1b   ,2b    ,      ,      ,      ,      ,      ,1start,2x    ,2a    ,      ,2y    ,2r bum,      ,2r tri,      ,      ,2start}

//    static final int GAMEPAD1_X = 1;
//    static final int GAMEPAD1_A = 2;
//    static final int GAMEPAD1_B = 3;
//    static final int GAMEPAD1_Y = 4;
//    static final int GAMEPAD1_LB = 5;
//    static final int GAMEPAD1_RB = 6;
//    static final int GAMEPAD1_LT = 7;
//    static final int GAMEPAD1_RT = 8;
//    static final int GAMEPAD1_BK = 9;
//    static final int GAMEPAD1_START = 10;
//    static final int GAMEPAD1_LJ = 11;
//    static final int GAMEPAD1_RJ = 12;
//    static final int GAMEPAD1_DU = 13;
//    static final int GAMEPAD1_DR = 14;
//    static final int GAMEPAD1_DD = 15;
//    static final int GAMEPAD1_DL = 16;
//
//    static final int GAMEPAD2_X = 21;
//    static final int GAMEPAD2_A = 22;
//    static final int GAMEPAD2_B = 23;
//    static final int GAMEPAD2_Y = 24;
//    static final int GAMEPAD2_LB = 25;
//    static final int GAMEPAD2_RB = 26;
//    static final int GAMEPAD2_LT = 27;
//    static final int GAMEPAD2_RT = 28;
//    static final int GAMEPAD2_BK = 29;
//    static final int GAMEPAD2_START = 30;
//    static final int GAMEPAD2_LJ = 31;
//    static final int GAMEPAD2_RJ = 32;
//    static final int GAMEPAD2_DU = 33;
//    static final int GAMEPAD2_DR = 34;
//    static final int GAMEPAD2_DD = 35;
//    static final int GAMEPAD2_DL = 36;


    //Check for single button press, as opposed to holding down - Used mainly for toggles
    public static boolean checkButton(boolean check, int btn, int trackEdge, boolean buttonChecking[]) {
        int edge = -1;

        //If we're not tracking a button, and we've pressed a button, log the press
        //Triggers on first pulse
        if (!buttonChecking[btn - 1] && check) {
            buttonChecking[btn - 1] = true;
            edge = 0;
        }

        //If button is being tracked, and we're no longer pressing it, stop tracking
        //Triggers on last pulse (when released)
        if (buttonChecking[btn - 1] && !check) {
            buttonChecking[btn - 1] = false;
            edge = 1;
        }

        //Return value based on leading or trailing edge
        //If edge has been triggered, and matches the edge we're watching, then return true, else false
        return trackEdge == edge;
    }

//     double scaleInput(double dVal)  {
//
//
//* This method scales the joystick input so for low joystick values, the
//* scaled value is less than linear.  This is to make it easier to drive
//* the robot more precisely at slower speeds.
//
//
//        double[] scaleArray = new double[]{ 0.0, 0.05, 0.09, 0.10, 0.12, 0.15, 0.18, 0.24, 0.30,
//                0.36, 0.43, 0.50, 0.60, 0.72, 0.85, 1.00, 1.00};
//
//        // get the corresponding index for the array.
//        int index = (int) (dVal * 16.0);
//        if (index < 0) {
//            index = -index;
//        } else if (index > 16) {
//            index = 16;
//        }
//
//        double dScale = 0.0;
//        if (dVal < 0) {
//            dScale = -scaleArray[index];
//        } else {
//            dScale = scaleArray[index];
//        }
//
//        return dScale;
//    }

    static double scaleInput(double value) {
        //This enhances sensitivity at slow speed and less sensitivity at higher input
//        return (Math.copySign(value * value, value));
//      return (Math.tanh(value)/Math.tanh(1));  //suggested by some but flattens high vs low end
//      return((value / 1.07) * (0.62 * (value * value) + 0.45));
        return (Math.pow(value, 3.0));
    }


//    /////////////////////////////////////////////////////////////////////////////////////////////
//    // Example DCMotor helper methods
//
//    public void setEncoderTarget(int XXXXEncoder, ...)
//    public void addEncoderTarget(int XXXXEncoder, ...)
//    public void setDrivePower(double XXXXPower, ...)
//    public void setDriveSpeed(double XXXXSpeed, ...)
//    public void runToPosition()
//    public void useConstantSpeed()
//    public void useConstantPower()
//    public void resetDriveEncoders()
//    public void synchEncoders()
//    public void setDriveMode(DcMotor.RunMode mode)
//    public int getXXXXPosition()
//    public boolean moveComplete()
//    public boolean encodersAtZero()
//
//    //--------------------------------------------------------------------------
//    // setEncoderTarget( LeftEncoder, RightEncoder);
//    // Sets Absolute Encoder Position
//    //--------------------------------------------------------------------------
//    public void setEncoderTarget(int XXXXEncoder, ...)
//    {
//        XXXXMotor.setTargetPosition(XXXXEncoderTarget = XXXXEncoder);
//        ...
//    }
//
//    //--------------------------------------------------------------------------
//    // addEncoderTarget( LeftEncoder, RightEncoder);
//    // Sets relative Encoder Position.  Offset current targets with passed data
//    //--------------------------------------------------------------------------
//    public void addEncoderTarget(int XXXXEncoder, ...)
//    {
//        XXXXMotor.setTargetPosition(XXXXEncoderTarget += XXXXEncoder);
//        ...
//
//    //--------------------------------------------------------------------------
//    // setDrivePower( XXXXPower, ...);
//    //--------------------------------------------------------------------------
//    public void setDrivePower(double XXXXPower, ...)
//    {
//        XXXXMotor.setPower(XXXXPower);
//        ....
//    }
//
//    //--------------------------------------------------------------------------
//    // setDriveSpeed( XXXXSpeed, ...);
//    //--------------------------------------------------------------------------
//    public void setDriveSpeed(double XXXXSpeed, ...)
//    {
//        setDrivePower(XXXXSpeed, ....);
//    }
//
//    //--------------------------------------------------------------------------
//    // runToPosition ()
//    // Set both drive motors to encoder servo mode (requires encoders)
//    //--------------------------------------------------------------------------
//    public void runToPosition()
//    {
//        setDriveMode(DcMotor.RunMode.RUN_TO_POSITION);
//    }
//
//    //--------------------------------------------------------------------------
//    // useConstantSpeed ()
//    // Set both drive motors to constant speed (requires encoders)
//    //--------------------------------------------------------------------------
//    public void useConstantSpeed()
//    {
//        setDriveMode(DcMotor.RunMode.RUN_USING_ENCODER);
//    }
//
//    //--------------------------------------------------------------------------
//    // useConstantPower ()
//    // Set both drive motors to constant power (encoders NOT required)
//    //--------------------------------------------------------------------------
//    public void useConstantPower()
//    {
//        setDriveMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//    }
//
//    //--------------------------------------------------------------------------
//    // resetDriveEncoders()
//    // Reset both drive motor encoders, and clear current encoder targets.
//    //--------------------------------------------------------------------------
//    public void resetDriveEncoders()
//    {
//        setEncoderTarget(0, 0 ,0 ,0);
//        setDriveMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//    }
//
//    //--------------------------------------------------------------------------
//    // syncEncoders()
//    // Load the current encoder values into the Target Values
//    // Essentially synch's the software with the hardware
//    //--------------------------------------------------------------------------
//    public void synchEncoders()
//    {
//        //	get and set the encoder targets
//        XXXXEncoderTarget =  XXXXMotor.getCurrentPosition();
//        ....
//    }
//
//    //--------------------------------------------------------------------------
//    // setDriveMode ()
//    // Set both drive motors to new mode if they need changing.
//    //--------------------------------------------------------------------------
//    public void setDriveMode(DcMotor.RunMode mode)
//    {
//        // Ensure the motors are in the correct mode.
//        if (XXXXMotor.getMode() != mode)
//            XXXXMotor.setMode(mode);
//
//        ....
//    }
//
//    //--------------------------------------------------------------------------
//    // getLeftFrontPosition ()
//    // Return Left Front Encoder count
//    //--------------------------------------------------------------------------
//    public int XXXXPosition()
//    {
//        return XXXX.getCurrentPosition();
//    }
//
//    //--------------------------------------------------------------------------
//    // encodersAtZero()
//    // Return true if both encoders read zero (or close)
//    //--------------------------------------------------------------------------
//    public boolean encodersAtZero()
//    {
//        return ((Math.abs(getXXXXPosition()) < 5) && ...) < 5));
//    }
///////////////////////////////////////////////////////////////////////////////////////////////////
}
