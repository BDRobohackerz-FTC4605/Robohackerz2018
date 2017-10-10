package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_USING_ENCODER;
import static org.firstinspires.ftc.teamcode.Constants.STOP;
import static org.firstinspires.ftc.teamcode.Constants.STOP;

/**
 * Created by Rob on 10/17/2016.
 */
public class Conveyor {
    private Telemetry telemetry = null;
    private DcMotor sweeperMotor = null;
    private DcMotor.Direction direction = null;

    public final double DEFAULT_SWEEPER = 1;
    public final double DEFAULT_UNLOAD = 0.5;

    public Conveyor() {

    }

    public void init(HardwareMap hwmap, Telemetry telemetryIn, DcMotor.Direction direction) {

        this.telemetry = telemetryIn;

        sweeperMotor = hwmap.dcMotor.get("sweeper_motor");
        sweeperMotor.setPower(STOP);
        sweeperMotor.setDirection(direction);
        sweeperMotor.setMode(RUN_USING_ENCODER);
    }

    public void init(HardwareMap hwmap, Telemetry telemetryIn) {

        this.telemetry = telemetryIn;

        sweeperMotor = hwmap.dcMotor.get("sweeper_motor");
        sweeperMotor.setPower(STOP);
        sweeperMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        sweeperMotor.setMode(RUN_USING_ENCODER);
    }

    public void init_loop() {
    }

    ;

    public void start() {
    }

    public void loop() {
        telemetry.addData("Sweeper:loop", "");
    }

    void stop() {
        sweeperMotor.setPower(STOP);
        telemetry.addData("Sweeper-stop", "");
    }

    void sweep(double power) {
        sweeperMotor.setPower(power);
        telemetry.addData("Sweeper-sweep(param)", "");
    }

    void sweep() {
        sweeperMotor.setPower(DEFAULT_SWEEPER);
        telemetry.addData("Sweeper-sweep(default)", "");
    }

    void unload(double power) {
        sweeperMotor.setPower(-power);
        telemetry.addData("Sweeper-unload(param)", "");
    }

    void unload() {
        sweeperMotor.setPower(-DEFAULT_UNLOAD);
        telemetry.addData("Sweeper-unload(default)", "");
    }

}
