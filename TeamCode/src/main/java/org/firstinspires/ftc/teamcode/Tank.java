package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp (name = "Tank", group = "Test")
public class Tank extends RobotHardware {

    @Override
    public void init() {
        super.init();
    }

    @Override
    public void loop() {
        super.loop();

        setDriveForTank(-gamepad1.left_stick_y, -gamepad1.right_stick_y);

        setPower(MotorName.FEEDER,gamepad1.right_trigger);
        setPower(MotorName.ARM,gamepad1.right_stick_x);
        setPower(MotorName.WRIST,gamepad1.left_stick_x);
    }


}
