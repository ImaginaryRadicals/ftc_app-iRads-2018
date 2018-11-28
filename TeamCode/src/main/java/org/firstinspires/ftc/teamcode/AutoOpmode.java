package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Utilities.InteractiveInit;
import org.firstinspires.ftc.teamcode.Utilities.Mutable;
import org.firstinspires.ftc.teamcode.Utilities.RobotStateMachine;
import org.firstinspires.ftc.teamcode.Utilities.AutoDrive;
import org.firstinspires.ftc.teamcode.Utilities.Color;
import org.firstinspires.ftc.teamcode.Utilities.Constants;
import org.firstinspires.ftc.teamcode.Utilities.Controller;
import org.firstinspires.ftc.teamcode.Utilities.MecanumNavigation;
import org.firstinspires.ftc.teamcode.Utilities.SimpleVision;
import org.firstinspires.ftc.teamcode.Utilities.TimingMonitor;

public class AutoOpmode extends RobotHardware {


    public TimingMonitor timingMonitor;
    public MecanumNavigation mecanumNavigation;
    public AutoDrive autoDrive;
    protected Color.Ftc robotColor;
    protected StartPosition robotStartPos;
    protected RobotStateMachine robotStateMachine;
    public SimpleVision simpleVision;
    private Thread thread;
    public Controller controller;

    public Mutable<Boolean> Simple = new Mutable<>(false);

    InteractiveInit interactiveInit = null;


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
        timingMonitor = new TimingMonitor(AutoOpmode.this);
        controller = new Controller(gamepad1);
//        thread = new Thread(new VisionLoader());
//        thread.start();
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
        robotStateMachine = new RobotStateMachine(this, robotColor, robotStartPos);
        telemetry.addData("Initialization:", "Successful!");

        interactiveInit = new InteractiveInit(this);
        interactiveInit.addBoolean(Simple, "Simple Mode", false, true);
    }

    @Override
    public void init_loop() {
        super.init_loop();
        controller.update();

//        if (simpleVision == null) {
//            telemetry.addData("Vision:", "LOADING...");
//        } else {
//            telemetry.addData("Vision:", "INITIALIZED");
//        }

        interactiveInit.update();
    }

    @Override
    public void start() {
        super.init();
        // Ensure starting position at origin, even if wheels turned since initialize.
        mecanumNavigation.update();
        mecanumNavigation.setCurrentPosition(new MecanumNavigation.Navigation2D(0,0,0));
        robotStateMachine.init();

        interactiveInit.lock();
    }

    @Override
    public void loop() {
        timingMonitor.loopStart();
        if(controller.start()) { timingMonitor.reset();} // Clear with start button
        super.loop();
        timingMonitor.checkpoint("POST super.loop()");
        controller.update();
        timingMonitor.checkpoint("POST controller.update()");
        mecanumNavigation.update();
        timingMonitor.checkpoint("POST mecanumNavigation.update()");
        robotStateMachine.update();
        timingMonitor.checkpoint("POST robotStateMachine.update()");
        mecanumNavigation.displayPosition();
        telemetry.addData("Current State", robotStateMachine.state.toString());
        telemetry.addLine();
        telemetry.addData("Period Average (sec)", df_prec.format(getAveragePeriodSec()));
        telemetry.addData("Period Max (sec)", df_prec.format(getMaxPeriodSec()));
        timingMonitor.displayMaxTimes();
        timingMonitor.checkpoint("POST TELEMETRY");
//        try {
//            simpleVision.updateTensorFlow(true);
//            simpleVision.displayTensorFlowDetections();
//        } catch(Exception e) {
//            telemetry.addData("Vision Not Loaded", "");
//        }
        timingMonitor.checkpoint("END");
    }




    // Initialize vuforia in a separate thread to avoid init() hangups.
    class VisionLoader implements Runnable {
        public void run() {
            simpleVision = new SimpleVision(getVuforiaLicenseKey(), AutoOpmode.this,
                    false, true,false,
                    true, false);
        }
    }

}
