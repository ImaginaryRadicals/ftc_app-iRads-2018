package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import static java.lang.Math.abs;

@TeleOp

public class Robot_Test extends OpMode {

    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor DRIVE_FRONT_LEFT = null;
    private DcMotor DRIVE_FRONT_RIGHT = null;
    private DcMotor DRIVE_BACK_LEFT = null;
    private DcMotor DRIVE_BACK_RIGHT = null;
    private DcMotor ARM = null;
    private DcMotor WRIST = null;
    private DcMotor FEEDER = null;
    private double Arm_Speed = 0.2;
    private double Wrist_Speed = 0.2;
    private double Feeder_Speed = 0.2;

    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");

        DRIVE_FRONT_LEFT = hardwareMap.get(DcMotor.class, "DRIVE_FRONT_LEFT");
        DRIVE_BACK_LEFT = hardwareMap.get(DcMotor.class, "DRIVE_BACK_LEFT");
        DRIVE_FRONT_RIGHT = hardwareMap.get(DcMotor.class, "DRIVE_FRONT_RIGHT");
        DRIVE_BACK_RIGHT = hardwareMap.get(DcMotor.class, "DRIVE_BACK_RIGHT");
        ARM = hardwareMap.get(DcMotor.class, "ARM");
        WRIST = hardwareMap.get(DcMotor.class, "WRIST");
        FEEDER = hardwareMap.get(DcMotor.class,"FEEDER");

        DRIVE_FRONT_LEFT.setDirection(DcMotor.Direction.REVERSE);
        DRIVE_BACK_LEFT.setDirection(DcMotor.Direction.REVERSE);
        DRIVE_FRONT_RIGHT.setDirection(DcMotor.Direction.FORWARD);
        DRIVE_BACK_RIGHT.setDirection(DcMotor.Direction.FORWARD);
        ARM.setDirection(DcMotor.Direction.FORWARD);
        WRIST.setDirection(DcMotor.Direction.FORWARD);
        FEEDER.setDirection(DcMotor.Direction.FORWARD);

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
        double right = gamepad1.left_stick_x;
        double clockwise = gamepad1.right_stick_x;
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

        if (gamepad1.a)
            ARM.setPower(Arm_Speed);
        else if (gamepad1.y)
            ARM.setPower(-Arm_Speed);

        if (gamepad1.b)
            WRIST.setPower(Wrist_Speed);
        else if (gamepad1.x)
            WRIST.setPower(-Wrist_Speed);

        if (gamepad1.right_bumper)
        FEEDER.setPower(Feeder_Speed);
        else if (gamepad1.left_bumper)
            FEEDER.setPower(-Feeder_Speed);
    }
}
