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
    private DcMotor left_Front = null;
    private DcMotor left_Rear = null;
    private DcMotor right_Front = null;
    private DcMotor right_Rear = null;

    @Override
    public void init() {
        telemetry.addData("Status", "Jerry is very dumb!");

        left_Front = hardwareMap.get(DcMotor.class, "left_front");
        left_Rear = hardwareMap.get(DcMotor.class, "left_rear");
        right_Front = hardwareMap.get(DcMotor.class, "right_front");
        right_Rear = hardwareMap.get(DcMotor.class, "right_rear");

        left_Front.setDirection(DcMotor.Direction.FORWARD);
        left_Rear.setDirection(DcMotor.Direction.FORWARD);
        right_Front.setDirection(DcMotor.Direction.FORWARD);
        right_Rear.setDirection(DcMotor.Direction.FORWARD);

        telemetry.addData("Status ", "Josh is ok but still a noob");
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
        rear_left = forward - clockwise - right;
        front_right = forward + clockwise - right;
        rear_right = forward - clockwise + right;

        double max;

        max = Math.abs(front_left);
        if (Math.abs(front_right) < max) max = Math.abs(front_right);
        if (Math.abs(rear_right) < max) max = Math.abs(rear_right);
        if (Math.abs(rear_left) < max) max = Math.abs(rear_left);

        if (max > 1) {
            front_left/=max;
            front_right/=max;
            rear_left/=max;
            rear_right/=max;

        }

        left_Front.setPower(front_left);
        left_Rear.setPower(rear_left);
        right_Front.setPower(front_right);
        right_Rear.setPower(rear_right);

    }
}
