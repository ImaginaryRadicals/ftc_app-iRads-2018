package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Utilities.AutoDrive;
import org.firstinspires.ftc.teamcode.Utilities.Constants;
import org.firstinspires.ftc.teamcode.Utilities.Controller;
import org.firstinspires.ftc.teamcode.Utilities.InteractiveInit;
import org.firstinspires.ftc.teamcode.Utilities.MecanumNavigation;
import org.firstinspires.ftc.teamcode.Utilities.Mutable;

@TeleOp (name = "Manual", group = "Standard")
public class Manual extends RobotHardware {

    //Setting controller variables
    Controller controller1 = null;
    Controller controller2 = null;

    //Adding interactive init variables
    Mutable<Double> ArmSpeed = new Mutable<>(0.5);
    Mutable<Double> WristSpeed = new Mutable<>(0.2);
    Mutable<Double> FeederSpeed = new Mutable<>(.8);
    Mutable<Boolean> CoPilot = new Mutable<>(false);
    Mutable<Double> Exponential = new Mutable<>(1.0);
    Mutable<Double> GoToPosPower = new Mutable<>(1.0);

    //Enum for arm states
    public enum ArmStates{
        ARM_START,
        WRIST_UP,
        ARM_LEVEL,
        ARM_VERTICAL,
        ARM_SCORE,
        MANUAL,
    }

    ArmStates armState = ArmStates.MANUAL;

    InteractiveInit interactiveInit = null;
    MecanumNavigation mecanumNavigation;
    AutoDrive autoDrive;

    Controller armController = null;
    double triggerThreshold = 0.1;



    @Override
    public void init() {
        super.init();

        //Mecanum navigation
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


        //Changing gamepads to controllers
        controller1 = new Controller (gamepad1);
        controller2 = new Controller (gamepad2);

        //Adding Interactive init options
        interactiveInit = new InteractiveInit(this);
        interactiveInit.addDouble(ArmSpeed, "Arm speed", 0.1, 0.2, .3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0);
        interactiveInit.addDouble(WristSpeed, "Wrist speed", 0.1, 0.2, .3, 0.4, 0.6, 0.7, 0.8, 0.9, 1.0, 0.5);
        interactiveInit.addDouble(FeederSpeed, "Feeder speed", 0.1, 0.2, .3, 0.4, 0.5, 0.6, 0.7, 0.9, 1.0, 0.8);
        interactiveInit.addDouble(GoToPosPower, "Power to goto position", 0.1, 0.2, .3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0);
        interactiveInit.addDouble(Exponential, "Exponential", 3.0, 1.0);
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

        //Updates variables
        controller1.update();
        controller2.update();

        mecanumNavigation.update();

        double armSpeed = ArmSpeed.get();
        double wristSpeed = WristSpeed.get();
        double feederSpeed = FeederSpeed.get();
        double exponential = Exponential.get();
        boolean copilotEnabled = CoPilot.get();

        // Copilot / Arm control selector.
        if (copilotEnabled) {
            armController = controller2;
        } else {
            armController = controller1;
        }

        // Telemetry
        mecanumNavigation.displayPosition();
        telemetry.addData("ArmStates", armState.toString());

        armStateMachine();

        // Cord controls
        if (armController.leftBumper() && armController.rightBumper()) {
            // Winch controls
            setPower(MotorName.LIFT_WINCH, -armController.left_stick_y);
            // Feeder lifter
            double servoTarget = -armController.right_stick_y > 0 ? 1 : 0;
            moveServoAtRate(ServoName.FEEDER_LIFTER, servoTarget, Math.abs(armController.right_stick_y));

            if (armController.YOnce()) {
                mecanumNavigation.setCurrentPosition(new MecanumNavigation.Navigation2D(0, 0, 0));
            }

            if (armController.A()) {
                autoDrive.rotateThenDriveToPosition(new MecanumNavigation.Navigation2D(0, 0, 0), 0.5);
            }
        }

        if (controller1.YOnce()) {
            mecanumNavigation.setCurrentPosition(new MecanumNavigation.Navigation2D(0, 0, 0));
        } else { // Else for Cord controls
            // Mecanum Drive Control
            setDriveForSimpleMecanum(Math.pow(controller1.left_stick_x, exponential), Math.pow(controller1.left_stick_y, exponential),
                    Math.pow(controller1.right_stick_x, exponential), Math.pow(controller1.right_stick_y, exponential));



            if (copilotEnabled) { // Arm control for 2 pilots


                // Feeder control
                if (armController.right_trigger > triggerThreshold) {
                    setPower(MotorName.FEEDER, Math.pow(armController.right_trigger, exponential) * feederSpeed);
                } else if (armController.left_trigger > triggerThreshold) {
                    setPower(MotorName.FEEDER, Math.pow(-armController.left_trigger, exponential) * feederSpeed);
                } else {
                    setPower(MotorName.FEEDER, 0);
                }

                if (armState == ArmStates.MANUAL) {

                    // Arm and Wrist Control
                    setPower(MotorName.ARM, Math.pow(-armController.left_stick_y, exponential) * armSpeed);
                    setPower(MotorName.WRIST, Math.pow(-armController.right_stick_y, exponential) * wristSpeed);
                }

            } else { // Arm control for 1 pilot

                if (armState == ArmStates.MANUAL) {

                    // Arm control and speed
                    if (armController.rightBumper()) {
                        setPower(MotorName.ARM, armSpeed);
                    } else if (armController.leftBumper()) {
                        setPower(MotorName.ARM, -armSpeed);
                    } else {
                        setPower(MotorName.ARM, 0);
                    }

                    // Wrist control and speed
                    if (armController.dpadUp()) {
                        setPower(MotorName.WRIST, wristSpeed);
                    } else if (armController.dpadDown()) {
                        setPower(MotorName.WRIST, -wristSpeed);
                    } else {
                        setPower(MotorName.WRIST, 0);
                    }
                }
            }

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


    //Arm and wrist state machine
    public void armStateMachine () {

        double goToPos = GoToPosPower.get();
        boolean wristArrived = false;
        boolean armArrived = false;
        double power = goToPos;

        if(
            CoPilot.get() == true && ((Math.abs(armController.right_stick_y) >= triggerThreshold) ||
            (Math.abs(armController.left_stick_y) >= triggerThreshold) ) ||
            CoPilot.get() == false &&
            (armController.dpadUp() || armController.dpadDown() ||
            armController.rightBumper() || armController.leftBumper()))
        {
                armState = ArmStates.MANUAL;
        }

        switch (armState) {
            case ARM_START:
                armArrived = driveMotorToPos(MotorName.ARM, Constants.ARM_BOTTOM_TICKS, power);
                wristArrived = driveMotorToPos(MotorName.WRIST, Constants.WRIST_START_TICKS, power);
                if (armArrived && wristArrived && armController.XOnce()) {
                    armState = ArmStates.WRIST_UP;
                }
                break;

            case WRIST_UP:
                armArrived = driveMotorToPos(MotorName.ARM, Constants.ARM_BOTTOM_TICKS, power);
                wristArrived = driveMotorToPos(MotorName.WRIST, Constants.WRIST_MAX_TICKS, power);
                if (armArrived && wristArrived && armController.XOnce()) {
                    armState = ArmStates.ARM_LEVEL;
                } else if (armArrived && wristArrived && armController.BOnce()) {
                    armState = ArmStates.ARM_START;
                }
                break;

            case ARM_LEVEL:
                armArrived = driveMotorToPos(MotorName.ARM, Constants.ARM_LEVEL_TICKS, power);
                wristArrived = driveMotorToPos(MotorName.WRIST, Constants.WRIST_MAX_TICKS, power);
                if (armArrived && wristArrived && armController.XOnce()) {
                    armState = ArmStates.ARM_VERTICAL;
                } else if (armArrived && wristArrived && armController.BOnce()) {
                    armState = ArmStates.WRIST_UP;
                }
                break;

            case ARM_VERTICAL:
                armArrived = driveMotorToPos(MotorName.ARM, Constants.ARM_VERTICAL_TICKS, power);
                wristArrived = driveMotorToPos(MotorName.WRIST, Constants.WRIST_STRAIGHT_TICKS, power);
                if (armArrived && wristArrived && armController.XOnce()) {
                    armState = ArmStates.ARM_SCORE;
                } else if (armArrived && wristArrived && armController.BOnce()) {
                    armState = ArmStates.ARM_LEVEL;
                }
                break;

            case ARM_SCORE:
                armArrived = driveMotorToPos(MotorName.ARM, Constants.ARM_VERTICAL_TICKS, power);
                wristArrived = driveMotorToPos(MotorName.WRIST, Constants.WRIST_MIN_TICKS, power);
                if (armArrived && wristArrived && armController.XOnce()) {
                    armState = ArmStates.ARM_START;
                } else if (armArrived && wristArrived && armController.BOnce()) {
                    armState = ArmStates.ARM_VERTICAL;
                }
                break;

            case MANUAL:
                if (armController.XOnce()) {
                    armState = ArmStates.ARM_START;
                } else if (armController.BOnce()) {
                    armState = ArmStates.ARM_VERTICAL;
                }
                break;

            default:
                telemetry.addData("Default arm state ", "if this ran you messed up...");
                break;

        }

    }

}
