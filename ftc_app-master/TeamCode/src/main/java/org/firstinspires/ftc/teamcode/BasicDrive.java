package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_WITHOUT_ENCODER;
import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.FORWARD;
import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE;
import static com.qualcomm.robotcore.util.Range.scale;


@TeleOp (name = "BasicDrive" , group = "blah")
public class BasicDrive extends OpMode{
    private DcMotor FRight;
    private DcMotor BRight;
    private DcMotor BLeft;
    private DcMotor FLeft;
    @Override
    public void init() {
        FLeft = hardwareMap.dcMotor.get("frontLeft");
        FRight = hardwareMap.dcMotor.get("frontRight");
        BLeft = hardwareMap.dcMotor.get("backLeft");
        BRight = hardwareMap.dcMotor.get("backRight");
        FLeft.setMode(RUN_WITHOUT_ENCODER);
        FRight.setMode(RUN_WITHOUT_ENCODER);
        BLeft.setMode(RUN_WITHOUT_ENCODER);
        BRight.setMode(RUN_WITHOUT_ENCODER);
        BLeft.setDirection(FORWARD);
        FLeft.setDirection(FORWARD);
        FRight.setDirection(REVERSE);
        BRight.setDirection(REVERSE);
    }

    @Override
    public void loop() {
        float speed = -gamepad1.right_stick_y;
        float direction = gamepad1.right_stick_x;
        float strafe = gamepad1.left_stick_x;

        float Magnitude = Math.abs(speed) + Math.abs(direction) + Math.abs(strafe);
        if (Magnitude < 1) {
            Magnitude = 1;
        }
        FLeft.setPower(scale(speed + direction - strafe, -Magnitude, Magnitude, -1, 1));
        FRight.setPower(scale(speed - direction + strafe, -Magnitude, Magnitude, -1, 1));
        BLeft.setPower(scale(speed + direction + strafe, -Magnitude, Magnitude, -1, 1));
        BRight.setPower(scale(speed - direction - strafe, -Magnitude, Magnitude, -1, 1));

    }
}
