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

public class mecanum_drive_test extends OpMode {

    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor DRIVE_FRONT_LEFT = null;
    private DcMotor DRIVE_FRONT_RIGHT = null;
    private DcMotor DRIVE_BACK_LEFT = null;
    private DcMotor DRIVE_BACK_RIGHT = null;
    private DcMotor arm_Motor = null;
    private DcMotor wrist_Motor = null;
    private DcMotor feeder_Motor = null;
    private double Arm_Speed = 0.2;
    private double Wrist_Speed = 0.2;
    private double Feeder_Speed = 0.2;

    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");

        DRIVE_FRONT_LEFT = hardwareMap.get(DcMotor.class, "left_front");
        DRIVE_FRONT_RIGHT = hardwareMap.get(DcMotor.class, "left_rear");
        DRIVE_BACK_LEFT = hardwareMap.get(DcMotor.class, "right_front");
        DRIVE_BACK_RIGHT = hardwareMap.get(DcMotor.class, "right_rear");
        arm_Motor = hardwareMap.get(DcMotor.class, "arm_motor");
        wrist_Motor = hardwareMap.get(DcMotor.class, "wrist_motor");
        feeder_Motor = hardwareMap.get(DcMotor.class,"feeder_motor");

        DRIVE_FRONT_LEFT.setDirection(DcMotor.Direction.REVERSE);
        DRIVE_FRONT_RIGHT.setDirection(DcMotor.Direction.REVERSE);
        DRIVE_BACK_LEFT.setDirection(DcMotor.Direction.FORWARD);
        DRIVE_BACK_RIGHT.setDirection(DcMotor.Direction.FORWARD);
        arm_Motor.setDirection(DcMotor.Direction.FORWARD);
        wrist_Motor.setDirection(DcMotor.Direction.FORWARD);
        feeder_Motor.setDirection(DcMotor.Direction.FORWARD);

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
            arm_Motor.setPower(Arm_Speed);
        else if (gamepad1.y)
            arm_Motor.setPower(-Arm_Speed);

        if (gamepad1.b)
            wrist_Motor.setPower(Wrist_Speed);
        else if (gamepad1.x)
            wrist_Motor.setPower(-Wrist_Speed);

        if (gamepad1.right_bumper)
        feeder_Motor.setPower(Feeder_Speed);
        else if (gamepad1.left_bumper)
            feeder_Motor.setPower(-Feeder_Speed);
    }
}
