package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Created by Rob on 10/23/2016.
 */
public class Beacon {
    private Telemetry telemetry = null;

    private Servo beaconServo;
    private HardwareMap hwMap;
    public final double BEACON_SERVO_CENTER = 0.5;

    public Beacon() {

    }

    public void init(HardwareMap hwMap, Telemetry telemetryIn) {
        this.telemetry = telemetryIn;

        // Define and initialize Beacon servo
        beaconServo = hwMap.servo.get("beacon_servo");

        // Set beacon servo direction
        beaconServo.setDirection(Servo.Direction.FORWARD);

        // Set beacon servo range
        beaconServo.scaleRange(0, 1);

        // Set beacon servo power
        beaconServo.setPosition(BEACON_SERVO_CENTER);

        // Enable servo controller
        beaconServo.getController().pwmEnable();

    }

    public void init_loop() {
    }

    public void start() {
    }

    public void loop() {

    }

    public void stop() {
        beaconServo.getController().pwmDisable();

    }

}
