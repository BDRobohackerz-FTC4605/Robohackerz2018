package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Created by Rob on 10/23/2016.
 */
public class Sensors {
    private Telemetry telemetry = null;

    HardwareMap hwMap;
    DeviceInterfaceModule DIM;

    void Sensors() {    }

    public void init(HardwareMap hwMap, Telemetry telemetryIn) {
        this.telemetry = telemetryIn;

        DIM = hwMap.deviceInterfaceModule.get("Device_Interface_Module");
    }

    public void start() {    }

    public void loop() {
        telemetry.addData("Sensors", "");
    }

    public void stop() {    }

    public void init_loop() {    }
}
