package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Utilities.Color;
import org.firstinspires.ftc.teamcode.Utilities.IMUUtilities;


/**
 * Created by Ashley on 12/14/2017.
 */

@TeleOp(name="Diagnostic", group="Diagnostic")
public class DiagnosticOpMode extends Manual {

    IMUUtilities imuHelper;

    @Override
    public void init() {
        super.init();
        //imuHelper = new IMUUtilities(this, "IMU_1");
        telemetry.addData("Diagnostic Mode ", " Initialized");
    }

    @Override
    public void loop() {
        super.loop();
        //imuHelper.update();
        showDiagnosticTelemetry();
        //imuHelper.displayTelemetry();
    }


    public void showDiagnosticTelemetry() {

        telemetry.addData("Period Average (sec)", df_prec.format(getAveragePeriodSec()));
        telemetry.addData("Period Max (sec)", df_prec.format(getMaxPeriodSec()));

        // Show color sensor telemetry only if sensor is attached
        if (colorSensorExists(ColorSensorName.MINERAL_COLOR)) {
            displayColorSensorTelemetry();
        }

        // Display all ODS sensor light levels
//        for (OpticalDistanceSensorName o : OpticalDistanceSensorName.values()) {
//            telemetry.addData(o.name(), df_prec.format(getOpticalDistanceSensorLightLevel(o)));
//        }

        telemetry.addLine(); // Create Space

        // Display all servo positions
        for (ServoName s : ServoName.values()) {
            telemetry.addData(s.name(), df.format(getAngle(s)));
        }

        telemetry.addLine(); // Create Space

        // Display all motor encoder values
        for (MotorName m : MotorName.values()) {
            telemetry.addData(m.name(), getEncoderValue(m));
        }
    }


}
