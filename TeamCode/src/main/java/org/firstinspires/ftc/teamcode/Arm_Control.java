package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="Arm", group="Test")

public class Arm_Control extends OpMode {

    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor ARM = null;
    private DcMotor WRIST = null;
    private DcMotor FEEDER = null;
    private double Arm_Speed = 0.2;
    private double Wrist_Speed = 0.2;
    private double Feeder_Speed = 0.2;

    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");

        ARM = hardwareMap.get(DcMotor.class, "ARM");
        WRIST = hardwareMap.get(DcMotor.class, "WRIST");
        FEEDER = hardwareMap.get(DcMotor.class,"FEEDER");

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
    public void loop(){

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
