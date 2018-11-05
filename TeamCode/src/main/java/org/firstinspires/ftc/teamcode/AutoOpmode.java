package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.Utilities.AutoDrive;
import org.firstinspires.ftc.teamcode.Utilities.Color;
import org.firstinspires.ftc.teamcode.Utilities.Constants;
import org.firstinspires.ftc.teamcode.Utilities.Controller;
import org.firstinspires.ftc.teamcode.Utilities.MecanumNavigation;

public class AutoOpmode extends RobotHardware {


    public MecanumNavigation mecanumNavigation;
    public AutoDrive autoDrive;
    protected Color.Ftc robotColor;
    protected StartPosition robotStartPos;
//    protected AutoDeluxeStateMachine autoDeluxeStateMachine;
//    protected SimpleVuforia vuforia;
//    public RelicRecoveryVuMark glyphPositionVuMark = RelicRecoveryVuMark.UNKNOWN;
    private Thread thread;
    public Controller controller;




    @Autonomous(name="auto.Red.Crater", group="Auto")
    public static class AutoRedCrater extends AutoOpmode {
        @Override public void init() {
            robotColor = Color.Ftc.RED;
            robotStartPos = StartPosition.FIELD_CRATER;
            super.init();
        }
    }

    @Autonomous(name="auto.Red.Depot", group="Auto")
    public static class AutoRedDepot extends AutoOpmode {
        @Override public void init() {
            robotColor = Color.Ftc.RED;
            robotStartPos = StartPosition.FIELD_DEPOT;
            super.init();
        }
    }

    @Autonomous(name="auto.Blue.Crater", group="Auto")
    public static class AutoBlueCrater extends AutoOpmode {
        @Override public void init() {
            robotColor = Color.Ftc.BLUE;
            robotStartPos = StartPosition.FIELD_CRATER;
            super.init();
        }
    }

    @Autonomous(name="auto.Blue.Depot", group="Auto")
    public static class AutoBlueDepot extends AutoOpmode {
        @Override public void init() {
            robotColor = Color.Ftc.BLUE;
            robotStartPos = StartPosition.FIELD_DEPOT;
            super.init();
        }
    }

    @Override
    public void init() {
        super.init();
        controller = new Controller(gamepad1);
        thread = new Thread(new VuforiaLoader());
        thread.start();
        mecanumNavigation = new MecanumNavigation(this,
                new MecanumNavigation.DriveTrainMecanum(
                        Constants.WHEELBASE_LENGTH_IN, Constants.WHEELBASE_WIDTH_IN,
                        Constants.DRIVE_WHEEL_DIAMETER_INCHES, Constants.DRIVE_WHEEL_STEPS_PER_ROT,
                        Constants.DRIVE_WHEEL_LATERAL_RATIO));
        mecanumNavigation.initialize(new MecanumNavigation.Navigation2D(0, 0, 0),
                new MecanumNavigation.WheelTicks(getEncoderValue(MotorName.DRIVE_FRONT_LEFT),
                        getEncoderValue(MotorName.DRIVE_FRONT_RIGHT),
                        getEncoderValue(MotorName.DRIVE_BACK_LEFT),
                        getEncoderValue(MotorName.DRIVE_BACK_RIGHT)));
        autoDrive = new AutoDrive(this, mecanumNavigation);
        // Finally, construct the state machine.
//        autoDeluxeStateMachine = new AutoDeluxeStateMachine(this, robotColor, robotStartPos);
        telemetry.addData("Initialization:", "Successful!");
    }

    @Override
    public void init_loop() {
        super.init_loop();
        controller.update();

//        if (vuforia == null) {
//            telemetry.addData("Vuforia:", "LOADING...");
//        } else {
//            telemetry.addData("Vuforia:", "INITIALIZED");
//        }
        displayColorSensorTelemetry();
    }

    @Override
    public void start() {
        super.init();
        // Ensure starting position at origin, even if wheels turned since initialize.
        mecanumNavigation.update();
        mecanumNavigation.setCurrentPosition(new MecanumNavigation.Navigation2D(0,0,0));
//        autoDeluxeStateMachine.init();
    }

    @Override
    public void loop() {
        super.loop();
        controller.update();
        mecanumNavigation.update();
//        try {
//            RelicRecoveryVuMark vuMark = vuforia.detectMark();
//            setVumark(vuMark); // Store last non-UNKNOWN vumark detected.
//            telemetry.addData("Vuforia Glyph Position", vuMark);
//        } catch (Exception e) {
//            telemetry.addData("Vuforia", "NOT INITIALIZED");
//        }
//        autoDeluxeStateMachine.update();
        mecanumNavigation.displayPosition();
//        telemetry.addData("Current State", autoDeluxeStateMachine.state.toString());
        telemetry.addLine();
        displayColorSensorTelemetry();
    }




    // Initialize vuforia in a separate thread to avoid init() hangups.
    class VuforiaLoader implements Runnable {
        public void run() {
//            vuforia = new SimpleVuforia(getVuforiaLicenseKey(), AutoDeluxe.this, false, false);
        }
    }

}
