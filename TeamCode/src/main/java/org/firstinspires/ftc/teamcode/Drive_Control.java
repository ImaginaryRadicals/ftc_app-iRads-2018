package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;


@TeleOp(name="Drive", group="Test")

public class Drive_Control extends OpMode {

    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor DRIVE_FRONT_LEFT = null;
    private DcMotor DRIVE_FRONT_RIGHT = null;
    private DcMotor DRIVE_BACK_LEFT = null;
    private DcMotor DRIVE_BACK_RIGHT = null;

    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");

        DRIVE_FRONT_LEFT = hardwareMap.get(DcMotor.class, "DRIVE_FRONT_LEFT");
        DRIVE_FRONT_RIGHT = hardwareMap.get(DcMotor.class, "DRIVE_FRONT_RIGHT");
        DRIVE_BACK_LEFT = hardwareMap.get(DcMotor.class, "DRIVE_BACK_LEFT");
        DRIVE_BACK_RIGHT = hardwareMap.get(DcMotor.class, "DRIVE_BACK_RIGHT");

        DRIVE_FRONT_LEFT.setDirection(DcMotor.Direction.FORWARD);
        DRIVE_BACK_LEFT.setDirection(DcMotor.Direction.REVERSE);
        DRIVE_FRONT_RIGHT.setDirection(DcMotor.Direction.FORWARD);
        DRIVE_BACK_RIGHT.setDirection(DcMotor.Direction.REVERSE);

        telemetry.addData("Status ", "Motors have been Initialized");
    }

    @Override
    public void init_loop() {
    }

    @Override
    public void start() {
        runtime.reset();
    }

    @Override
    public void loop() {

        double front_left;
        double rear_left;
        double front_right;
        double rear_right;

        double forward = -gamepad1.left_stick_y;
        double right = gamepad1.right_stick_x;
        double clockwise = gamepad1.left_stick_x;
        front_left = forward + clockwise + right;
        rear_left = forward - clockwise + right;
        front_right = forward + clockwise - right;
        rear_right = forward - clockwise - right;

        double max;

        max = Math.abs(front_left);
        if (Math.abs(front_right) < max) max = Math.abs(front_right);
        if (Math.abs(rear_right) < max) max = Math.abs(rear_right);
        if (Math.abs(rear_left) < max) max = Math.abs(rear_left);

        if (max > 1) {
            front_left /= max;
            front_right /= max;
            rear_left /= max;
            rear_right /= max;

        }

        DRIVE_FRONT_LEFT.setPower(front_left);
        DRIVE_FRONT_RIGHT.setPower(rear_left);
        DRIVE_BACK_LEFT.setPower(front_right);
        DRIVE_BACK_RIGHT.setPower(rear_right);


    }
}
