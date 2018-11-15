package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Utilities.AutoDrive;
import org.firstinspires.ftc.teamcode.Utilities.Controller;
import org.firstinspires.ftc.teamcode.Utilities.Mutable;


@TeleOp(name = "Control System Test", group = "Testing")
public class ControlSystemTest extends RobotHardware {

    Controller controller1 = null;
    int simulated_ticks = 0;
    Mutable<Double> lastPower = new Mutable<>(0.0);
    Mutable<Boolean> arrived = new Mutable<>(false);
    Mutable<Integer> errorSignal = new Mutable<>(0);

    @Override
    public void init() {
        super.init();
        controller1 = new Controller(gamepad1);
    }


    @Override
    public void loop() {
        super.loop();
        controller1.update();

        // Emulate encoder ticks for arm position.
        int maxTicks = 1000;
        simulated_ticks = (int) (-1*controller1.left_stick_y*maxTicks);


        driveMotorToPosSim(MotorName.ARM, simulated_ticks, 1.0, lastPower, arrived, errorSignal);
        telemetry.addData("Target Position", simulated_ticks);
        telemetry.addData("Error Signal", errorSignal.get());
        telemetry.addData("Arrived?", arrived.toString());
        telemetry.addData("Last Power", lastPower.get());







    }
    public boolean driveMotorToPosSim (MotorName motorName, int targetTicks, double power,
                                       Mutable<Double> LastPower, Mutable<Boolean> Arrived, Mutable<Integer> ErrorSignal) {
        power = Range.clip(Math.abs(power), 0, 1);
        int poweredDistance = 20;
        int arrivedDistance = 90;
        int rampThreshold = 400;
        double maxRampPower = 0.8;
        double minRampPower = 0;
        int errorSignal = getEncoderValue(motorName) - targetTicks;
        ErrorSignal.set(errorSignal);
        double direction = errorSignal > 0 ? -1.0: 1.0;
        double rampDownRatio = AutoDrive.rampDown(Math.abs(errorSignal), rampThreshold, minRampPower, maxRampPower);

        if (Math.abs(errorSignal) > poweredDistance)
        {setPower(motorName, direction * power * rampDownRatio);
            LastPower.set(direction * power * rampDownRatio);
        }

        if(Math.abs(errorSignal) < arrivedDistance) {
            Arrived.set(true);
            return true;
        }else {
            Arrived.set(false);
            return false;
        }
    }



    public boolean driveMotorToPos (MotorName motorName, int targetTicks, double power) {
        power = Range.clip(Math.abs(power), 0, 1);
        int poweredDistance = 20;
        int arrivedDistance = 90;
        int rampThreshold = 400;
        double maxRampPower = 0.8;
        double minRampPower = 0;
        int errorSignal = getEncoderValue(motorName) - targetTicks;
        double direction = errorSignal > 0 ? -1.0: 1.0;
        double rampDownRatio = AutoDrive.rampDown(Math.abs(errorSignal), rampThreshold, minRampPower, maxRampPower);

        if (Math.abs(errorSignal) > poweredDistance)
        {setPower(motorName, direction * power * rampDownRatio);
        }

        if(Math.abs(errorSignal) < arrivedDistance) {
            return true;
        }else {
            return false;
        }
    }



}




