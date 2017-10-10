package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Constants.FtcColor;

import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.FORWARD;
import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE;
//import static com.qualcomm.robotcore.hardware.Servo.Direction.FORWARD;

/**
 * This is NOT an opmode.
 * <p/>
 * This class can be used to define all the specific hardware for Alanbot2017.
 * <p/>
 * This hardware class assumes the following device names have been configured on the robot:
 * Note:  All names are lower case and some have single spaces between words.
 * <p/>
 * Motor channel:  Left Front drive motor:       "left_front_motor"
 * Motor channel:  Right Front drive motor:      "right_front_motor"
 * Motor channel:  Left Rear drive motor:        "left_rear_motor"
 * Motor channel:  Right Rear drive motor:       "right_rear_motor"
 */
public class HardwareAlanbot2017 {
    public Telemetry telemetry = null;
    //    private Gamepad gamepad1;
//    private Gamepad gamepad2;
    /* constants */
    /* universal */
    /* sweeper constants */

    private final double DEFAULT_SWEEPER = 1;
    private final double DEFAULT_UNLOAD = 0.5;

    /* beacon constants */

    FtcColor beaconColor = null;
    
    private static final double BEACON_SERVO_CENTER = 0.5;
    private static final double BEACON_SERVO_LEFT = 0;
    private static final double BEACON_SERVO_RIGHT = 1;

    FtcColor alliance = null;
    //
    // If you copy our code, please register your own account and generate your own free license key at this site:
    // https://developer.vuforia.com/license-manager
    //
/*    private final String VUFORIA_LICENSE_KEY =
            "AdCwzDH/////AAAAGeDkDS3ukU9+lIXc19LMh+cKk29caNhOl8UqmZOymRGwVwT1ZN8uaPdE3Q+zceDu9AKNsqL9qLblSFV" +
                    "/x8Y3jfOZdjMFs0CQSQOEyWv3xfJsdSmevXDQDQr+4KI31HY2YSf/KB/kyxfuRMk4Pi+vWS+oLl65o7sWPiyFgzoM74ENyb" +
                    "j4FgteD/2b6B+UFuwkHWKBNpp18wrpkaiFfr/FCbRFcdWP5mrjlEZM6eOj171dybw97HPeZbGihnnxOeeUv075O7P167AVq" +
                    "aiPy2eRK7OCubR32KXOqQKoyF6AXp+qu2cOTApXS5dqOOseEm+HE4eMF0S2Pld3i5AWBIR+JlPXDuc9LwoH2Q8iDwUK1+4g";
*/
//    private final String VUFORIA_LICENSE_KEY =
//            "AVvRb+j/////AAAAGabuHrQd6kQ7l5YaZFAEzC6AmLVcjBBw1TKdOQhxTBsNH/lNWBs70Q3M5qMoXQyOrF1IP" +
//                    "TU6moBI3dDs+e93BWS4bQ2/LdMj10COpd8u9BAXwXgjAuXa3gkmQ3FvTtz78/1ynZ25zMe0po8fOM" +
//                    "ttCGhcL/PU1eaDmvI2Cdr41Lj3LJc2ASoLw8wWbJ2I1kTHCDRw/63m6zQklqWPCTHfLMOmdGRE1lg" +
//                    "P/ukUtmbvWXnUCGPxI1YiKkwIzPk7CyNeUnRSXiXmPHp3Gp5kNj+9YmjheT5pJe9QID2AR+a8fJQZ" +
//                    "IYcLkODLGj83xcwBJPXupOWGb/R9dRevEx4CJWa8SNuGrcqxGPDeVP4CfEpTf0B1";
//    protected final int CAMERAVIEW_ID = R.id.cameraMonitorViewId;
//    protected final VuforiaLocalizer.CameraDirection CAMERA_DIR = VuforiaLocalizer.CameraDirection.BACK;
//    protected final String TRACKABLES_FILE = "FTC_2016-17";
//    //
//    // Note that the order of the targets must match the order in the FTC_2016-17.xml file.
//    //
//    private FtcVuforiaRwh.Target[] targets =
//            {
//                    //
//                    // Blue alliance beacon 1.
//                    //
//                    new FtcVuforiaRwh.Target("wheels", 90.0f, 0.0f, 0.0f,
//                            12.0f * MM_PER_INCH, FTC_FIELD_WIDTH / 2.0f, TARGET_HEIGHT),
//                    //
//                    // Red alliance beacon 2.
//                    //
//                    new FtcVuforiaRwh.Target("tools", 90.0f, 0.0f, 90.0f,
//                            -FTC_FIELD_WIDTH / 2.0f, 30.0f * MM_PER_INCH, TARGET_HEIGHT),
//                    //
//                    // Blue alliance beacon 2 location.
//                    //
//                    new FtcVuforiaRwh.Target("legos", 90.0f, 0.0f, 0.0f,
//                            -30.0f * MM_PER_INCH, FTC_FIELD_WIDTH / 2.0f, TARGET_HEIGHT),
//                    //
//                    // Red alliance beacon 1 location.
//                    //
//                    new FtcVuforiaRwh.Target("gears", 90.0f, 0.0f, 90.0f,
//                            -FTC_FIELD_WIDTH / 2.0f, -12.0f * MM_PER_INCH, TARGET_HEIGHT)
////                    new FtcVuforiaRwh.Target("gears", 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f)
//            };
//    //
//    // Phone location: Mounted center on the front of the robot with the back camera facing outward.
//    //
//    private FtcVuforiaRwh.PhoneLocationOnRobot phoneLocationOnRobot =
//            new FtcVuforiaRwh.PhoneLocationOnRobot(90.0f, 0.0f, 0.0f, 0.0f, ROBOT_WIDTH / 2.0f, 0.0f);
//
//    private final boolean TRACK_ROBOT_LOC = true;
////    private final boolean SPEECH_ENABLED = true;
//
//    //    private HalDashboard dashboard;
////    private FtcVuforiaRwh vuforia;
//    private OpenGLMatrix lastKnownRobotLocation = null;
//    //    private TextToSpeech textToSpeech = null;
//    private boolean[] targetsFound = null;


    HardwareMap hwMap = null;
    private ElapsedTime period = new ElapsedTime();

    Drive drive = new Drive();
    Conveyor sweeper = new Conveyor();
    Flicker flicker = new Flicker();
    Lift lift = new Lift();
    Beacon beacon = new Beacon();
    Sensors sensors = new Sensors();
//    FtcVuforiaRwh vuforia = new FtcVuforiaRwh(telemetry, VUFORIA_LICENSE_KEY, CAMERAVIEW_ID, CAMERA_DIR,
//            TRACKABLES_FILE, targets.length, targets, phoneLocationOnRobot);

    /* Constructor */
    public HardwareAlanbot2017() {

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap, Telemetry telemetry, Gamepad gamepad1, Gamepad gamepad2) { //, HardwareAlanbot2017 robot, Gamepad gamepad1, Gamepad gamepad2) {
        // Save reference to Hardware map
        hwMap = ahwMap;
        this.telemetry = telemetry;
        telemetry.addData("Hardware_init", "");

        drive.init(ahwMap, telemetry, FORWARD, REVERSE); //, robot, gamepad1, gamepad2);
        sweeper.init(ahwMap, telemetry);
        flicker.init(ahwMap, telemetry, FORWARD);
        lift.init(ahwMap, telemetry, FORWARD, 1.0, -1.0);
        beacon.init(ahwMap, telemetry);
        sensors.init(ahwMap, telemetry);
//        vuforia.init();

    }

    public void stop() {

        drive.stop();
        sweeper.stop();
        flicker.stop();
        lift.stop();
        beacon.stop();
        sensors.stop();

    }

    /***
     * waitForTick implements a periodic delay. However, this acts like a metronome with a regular
     * periodic tick.  This is used to compensate for varying processing times for each cycle.
     * The function looks at the elapsed cycle time, and sleeps for the remaining time interval.
     *
     * @param periodMs Length of wait cycle in mSec.
     */
    public void waitForTick(long periodMs) {

        long remaining = periodMs - (long) period.milliseconds();

        // sleep for the remaining portion of the regular cycle period.
        if (remaining > 0) {
            try {
                Thread.sleep(remaining);
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
            }
        }

        // Reset the cycle clock for the next pass.
        period.reset();
    }


}

