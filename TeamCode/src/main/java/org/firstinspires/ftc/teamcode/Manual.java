package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Utilities.AutoDrive;
import org.firstinspires.ftc.teamcode.Utilities.Constants;
import org.firstinspires.ftc.teamcode.Utilities.Controller;
import org.firstinspires.ftc.teamcode.Utilities.InteractiveInit;
import org.firstinspires.ftc.teamcode.Utilities.Mecanum;
import org.firstinspires.ftc.teamcode.Utilities.MecanumNavigation;
import org.firstinspires.ftc.teamcode.Utilities.Mutable;

@TeleOp (name = "Manual", group = "Standard")
public class Manual extends RobotHardware {

    Controller controller1 = null;
    Controller controller2 = null;
    Mutable<Double> ArmSpeed = new Mutable<>(0.5);
    Mutable<Double> WristSpeed = new Mutable<>(0.2);
    Mutable<Double> FeederSpeed = new Mutable<>(.8);
    Mutable<Boolean> CoPilot = new Mutable<>(false);
    InteractiveInit interactiveInit = null;
    MecanumNavigation mecanumNavigation;
    AutoDrive autoDrive;



    @Override
    public void init() {
        super.init();


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



        controller1 = new Controller (gamepad1);
        controller2 = new Controller (gamepad2);

        interactiveInit = new InteractiveInit(this);
        interactiveInit.addDouble(ArmSpeed, "Arm speed", 0.1, 0.2, .03, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0);
        interactiveInit.addDouble(WristSpeed, "Wrist speed", 0.1, 0.2, .03, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0);
        interactiveInit.addDouble(FeederSpeed, "Feeder speed", 0.1, 0.2, .03, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0);
        interactiveInit.addBoolean(CoPilot, "Copilot Enable", false, true);
    }

    @Override
    public void init_loop() {
        super.init_loop();

        interactiveInit.update();
    }

    @Override
    public void start() {
        super.start();

        interactiveInit.lock();
    }

    @Override
    public void loop() {
        super.loop();

        controller1.update();
        controller2.update();
        mecanumNavigation.update();
        mecanumNavigation.displayPosition();

        double triggerThreshhold = 0.1;
        double armSpeed = ArmSpeed.get();
        double wristSpeed = WristSpeed.get();
        double feederSpeed = FeederSpeed.get();
        boolean copilotEnabled = CoPilot.get();
        Controller armController = null;


        if (copilotEnabled) {
            armController = controller2;
        } else {
            armController = controller1;
        }




        setDriveForSimpleMecanum(controller1.left_stick_x, controller1.left_stick_y,
                                 controller1.right_stick_x, controller1.right_stick_y);
        // Feeder control
        if (armController.right_trigger > triggerThreshhold) {
            setPower(MotorName.FEEDER, armController.right_trigger * feederSpeed);
        } else if (armController.left_trigger > triggerThreshhold){
            setPower(MotorName.FEEDER, -armController.left_trigger * feederSpeed);
        } else {
            setPower(MotorName.FEEDER, 0);
        }

        // Arm control and speed
        if (armController.rightBumper()) {
            setPower(MotorName.ARM, armSpeed);
        } else if (armController.leftBumper()){
            setPower(MotorName.ARM, -armSpeed);
        } else {
            setPower(MotorName.ARM, 0);
        }

        // Wrist control and speed
        if (armController.dpadUp()) {
            setPower(MotorName.WRIST, wristSpeed);
        } else if (armController.dpadDown()){
            setPower(MotorName.WRIST, -wristSpeed);
        } else {
            setPower(MotorName.WRIST, 0);
        }

        if (controller1.YOnce()) {
            mecanumNavigation.setCurrentPosition(new MecanumNavigation.Navigation2D(0, 0, 0));
        }


    }


}
