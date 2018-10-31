package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import static java.lang.Math.abs;

@TeleOp(name="Drive+Arm", group="Test")

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
    private double Feeder_Speed = 0.6;
    private Servo LEFT_FLIPPER = null;
    private Servo RIGHT_FLIPPER = null;

    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");

        DRIVE_FRONT_LEFT = hardwareMap.get(DcMotor.class, "DRIVE_FRONT_LEFT");
        DRIVE_FRONT_RIGHT = hardwareMap.get(DcMotor.class, "DRIVE_FRONT_RIGHT");
        DRIVE_BACK_LEFT = hardwareMap.get(DcMotor.class, "DRIVE_BACK_LEFT");
        DRIVE_BACK_RIGHT = hardwareMap.get(DcMotor.class, "DRIVE_BACK_RIGHT");
        ARM = hardwareMap.get(DcMotor.class, "ARM");
        WRIST = hardwareMap.get(DcMotor.class, "WRIST");
        FEEDER = hardwareMap.get(DcMotor.class,"FEEDER");
        LEFT_FLIPPER = hardwareMap.get(Servo.class, "LEFT_FLIPPER");
        RIGHT_FLIPPER = hardwareMap.get(Servo.class, "RIGHT_FLIPPER");

        DRIVE_FRONT_LEFT.setDirection(DcMotor.Direction.FORWARD);
        DRIVE_BACK_LEFT.setDirection(DcMotor.Direction.REVERSE);
        DRIVE_FRONT_RIGHT.setDirection(DcMotor.Direction.FORWARD);
        DRIVE_BACK_RIGHT.setDirection(DcMotor.Direction.REVERSE);
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

        if (gamepad1.a == true) {
            ARM.setPower(Arm_Speed);
        }
        else if (gamepad1.y == true) {
            ARM.setPower(-Arm_Speed);
        }
        else{
            ARM.setPower(0);
        }


        if (gamepad1.b == true) {
            WRIST.setPower(Wrist_Speed);
        }
        else if (gamepad1.x == true) {
            WRIST.setPower(-Wrist_Speed);
        }
        else{
            WRIST.setPower(0);
        }


        if (gamepad1.right_bumper == true) {
            FEEDER.setPower(Feeder_Speed);
        }
        else if (gamepad1.left_bumper== true) {
            FEEDER.setPower(-Feeder_Speed);
        }
        else{
            FEEDER.setPower(0);
        }

        if (gamepad1.right_trigger >= 0.2) {
            LEFT_FLIPPER.setPosition(0.75);
            RIGHT_FLIPPER.setPosition(0.75);
        }
        else if (gamepad1.right_trigger < 0.2){
            LEFT_FLIPPER.setPosition(1);
            RIGHT_FLIPPER.setPosition(0);
        }
    }
}
