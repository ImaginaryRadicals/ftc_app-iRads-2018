package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Utilities.Controller;

@TeleOp (name = "Manual", group = "Standard")
public class Manual extends RobotHardware {

    Controller controller1 = null;
    Controller controller2 = null;

    @Override
    public void init() {
        super.init();

        controller1 = new Controller (gamepad1);
        controller2 = new Controller (gamepad2);

    }

    @Override
    public void loop() {
        super.loop();

        controller1.update();
        controller2.update();

        telemetry.addData("Period ", df_prec.format(getAveragePeriodSec()));

        double drive = -controller1.left_stick_y;
        double turn  =  controller1.left_stick_x;
        double leftPower  = Range.clip(drive + turn, -1.0, 1.0) ;
        double rightPower = Range.clip(drive - turn, -1.0, 1.0) ;
        setDriveForTank(leftPower, rightPower);

        setPower(MotorName.FEEDER, controller1.right_trigger);
        setPower(MotorName.ARM, -controller1.right_stick_y);
        setPower(MotorName.WRIST, controller1.right_stick_x);
    }


}
