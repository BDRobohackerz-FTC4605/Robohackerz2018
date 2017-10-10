package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_USING_ENCODER;
import static org.firstinspires.ftc.teamcode.Constants.STOP;

/**
 * class for Lift
 * lift(power)
 */
public class Lift {
    private Telemetry telemetry = null;

    private DcMotor liftMotor = null;
    private DcMotor.Direction direction = null;
    private double DEFAULT_RAISE;
    private double DEFAULT_LOWER;

    public Lift() {

    }

    void init(HardwareMap hwmap, Telemetry telemetryIn, DcMotor.Direction directionIn, double DEFAULT_RAISE_IN, double DEFAULT_LOWER_IN) {

        this.telemetry = telemetryIn;

        direction = directionIn;
        DEFAULT_RAISE = DEFAULT_RAISE_IN;
        DEFAULT_LOWER = DEFAULT_LOWER_IN;

        liftMotor = hwmap.dcMotor.get("lift_motor");
        liftMotor.setPower(STOP);
        liftMotor.setDirection(direction);
        liftMotor.setMode(RUN_USING_ENCODER);
    }

    public void init(HardwareMap hwmap, Telemetry telemetryIn, double DEFAULT_RAISE_IN, double DEFAULT_LOWER_IN) {

        this.telemetry = telemetryIn;

        DEFAULT_RAISE = DEFAULT_RAISE_IN;
        DEFAULT_LOWER = DEFAULT_LOWER_IN;

        liftMotor = hwmap.dcMotor.get("lift_motor");
        liftMotor.setPower(STOP);
        liftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        liftMotor.setMode(RUN_USING_ENCODER);
    }

    public void init_loop() {
    }

    ;

    public void start() {

    }

    public void loop() {
        telemetry.addData("Lift:loop", "");

    }

    public void stop() {
    }

    void lift(double power) {
        liftMotor.setPower(Range.scale(power, -1, 1, DEFAULT_LOWER, DEFAULT_RAISE));
        telemetry.addData("Lift:lift", "");
    }

    void raise() {
        liftMotor.setPower(DEFAULT_RAISE);
        telemetry.addData("Lift:raise(default)", "");
    }

    void raise(double raise) {
        liftMotor.setPower(raise);
        telemetry.addData("Lift:raise(param)", "");
    }

    void lower() {
        liftMotor.setPower(DEFAULT_LOWER);
        telemetry.addData("Lift:lower(default)", "");
    }

    void lower(double lower) {
        liftMotor.setPower(lower);
        telemetry.addData("Lift:lower(param)", "");
    }
}
