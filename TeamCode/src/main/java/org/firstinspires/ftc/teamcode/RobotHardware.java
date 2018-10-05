package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import java.text.DecimalFormat;
import java.util.ArrayList;
import java.util.Vector;

/**
 * Created by ryderswan on 10/1/18.
 */

public class RobotHardware extends OpMode{



    //Initalize Arm Motors
    //Arm motors start with the base = number 1, and increasing towards the top of the arm (
    // ex. if there are 3 segments, the bottom = 1, the middle = 2, and the top = 3.

    // All motors on the robot, in order of MotorName.
    private ArrayList<DcMotor> allMotors;
    // All servos on the robot, in order of ServoName.
    private ArrayList<Servo> allServos;
    // All optical distance sensors on the robot, in order of DistanceSensorName.
    private ArrayList<OpticalDistanceSensor> allOpticalDistanceSensors;
    // All color sensors on the robot, in order of ColorSensorName.
    private ArrayList<ColorSensor> allColorSensors;

    // Names of only the drive motors.
    private ArrayList<MotorName> driveMotorNames;

    // Names of only the arm motors
    private ArrayList<MotorName> armMotorNames;

    // IMU reference
    public BNO055IMU imu;

    // Execution cycle period monitor.
    //private ElapsedTime period  = new ElapsedTime();
    private Vector<Double> pastPeriods  = new Vector();


    // The motors on the robot.
    public enum MotorName {
        DRIVE_FRONT_LEFT,
        DRIVE_FRONT_RIGHT,
        DRIVE_BACK_LEFT,
        DRIVE_BACK_RIGHT,
        ARM_MOTOR_1,
        ARM_MOTOR_2,
        ARM_MOTOR_3,
    }

    // Motor methods

    /**
     * Sets the power of the motor.
     * @param motor The motor to modify.
     * @param power The power to set [-1, 1].
     */
    public void setPower(MotorName motor, double power) {
        DcMotor m = allMotors.get(motor.ordinal());
        if (m == null) {
            telemetry.addData("Motor Missing", motor.name());
        } else {
            m.setPower(power);
        }
    }

    /**
     * Get motor power
     * @param motor MotorName
     * @return Motor Power, or zero if it cannot be found
     */
    public double getPower(MotorName motor) {
        DcMotor m = allMotors.get(motor.ordinal());
        if (m == null) {
            telemetry.addData("Motor Missing", motor.name());
            return 0;
        } else {
            return m.getPower();
        }
    }

    /**
     * Gets the encoder value of the motor.
     * @param motor MotorName enum value.
     * @return integer encoder position in ticks.
     */
    public int getEncoderValue(MotorName motor) {
        DcMotor m = allMotors.get(motor.ordinal());
        if (m == null) {
            telemetry.addData("Motor Missing", motor.name());
            return 0;
        } else {
            return m.getCurrentPosition();
        }
    }

    /**
     * Stops all motors.
     */
    public void stopAllMotors() {
        for (MotorName m : MotorName.values()) {
            setPower(m, 0);
        }
    }

    /**
     * Set all four drive motors to the same runMode.
     * Options are RUN_WITHOUT_ENCODER, RUN_USING_ENCODER,
     * RUN_TO_POSITION is used with setTargetPosition()
     * STOP_AND_RESET_ENCODER
     * @param runMode
     */
    protected void setDriveMotorsRunMode(DcMotor.RunMode runMode) {
        for (MotorName motor : driveMotorNames) {
            DcMotor m = allMotors.get(motor.ordinal());
            if (m == null) {
                telemetry.addData("Motor Missing", motor.name());
            } else {
                m.setMode(runMode);
            }
        }
    }

    protected void setArmMotorRunMode(DcMotor.RunMode runMode) {
        for (MotorName motor : armMotorNames) {
            DcMotor m = allMotors.get(motor.ordinal());
            if (m == null) {
                telemetry.addData("Motor Missing", motor.name());
            } else {
                m.setMode(runMode);
            }
        }
    }

    /**
     * Set zero power braking behavior for all drive wheels.
     * @param zeroPowerBraking boolean to activate or deactivate zero power braking
     */
    protected void setDriveMotorsZeroPowerBraking(boolean zeroPowerBraking) {
        DcMotor.ZeroPowerBehavior brakingMode = zeroPowerBraking ? DcMotor.ZeroPowerBehavior.BRAKE
                : DcMotor.ZeroPowerBehavior.FLOAT;
        for (MotorName motor : driveMotorNames) {
            DcMotor m = allMotors.get(motor.ordinal());
            if (m == null) {
                telemetry.addData("Motor Missing", motor.name());
            } else {
                m.setZeroPowerBehavior(brakingMode);
            }
        }
    }

    protected void setarmMotorsZeroPowerBraking(boolean zeroPowerBraking) {
        DcMotor.ZeroPowerBehavior brakingMode = zeroPowerBraking ? DcMotor.ZeroPowerBehavior.BRAKE
                : DcMotor.ZeroPowerBehavior.FLOAT;
        for (MotorName motor : armMotorNames) {
            DcMotor m = allMotors.get(motor.ordinal());
            if (m == null) {
                telemetry.addData("Motor Missing", motor.name());
            } else {
                m.setZeroPowerBehavior(brakingMode);
            }
        }
    }


    /**
     * Sets the drive chain power.
     * @param left The power for the left two motors.
     * @param right The power for the right two motors.
     */
    public void setDriveForTank(double left, double right) {
        setPower(MotorName.DRIVE_FRONT_LEFT, left);
        setPower(MotorName.DRIVE_BACK_LEFT, left);
        setPower(MotorName.DRIVE_FRONT_RIGHT, right);
        setPower(MotorName.DRIVE_BACK_RIGHT, right);
    }


    /**
     * Apply motor power matching the wheels object
     * @param wheels Provides all four mecanum wheel powers, [-1, 1]
     */
    public void setDriveForMecanumWheels(Mecanum.Wheels wheels) {
        setPower(MotorName.DRIVE_FRONT_LEFT, wheels.frontLeft);
        setPower(MotorName.DRIVE_BACK_LEFT, wheels.backLeft);
        setPower(MotorName.DRIVE_FRONT_RIGHT, wheels.frontRight);
        setPower(MotorName.DRIVE_BACK_RIGHT, wheels.backRight);
    }

    public void setRotationForArmMotors(armKinematics.Motors Arm_Motor) {
        setPower(MotorName.ARM_MOTOR_1, Arm_Motor.one);
        setPower(MotorName.ARM_MOTOR_2, Arm_Motor.two);
        setPower(MotorName.ARM_MOTOR_3, Arm_Motor.three);
    }

    /**
     * Sets mecanum drive chain power using simplistic calculations.
     * @param leftStickX Unmodified Gamepad leftStickX inputs.
     * @param leftStickY Unmodified Gamepad leftStickY inputs.
     * @param rightStickX Unmodified Gamepad rightStickX inputs.
     * @param rightStickY Unmodified Gamepad rightStickY inputs.
     */
    public void setDriveForSimpleMecanum(double leftStickX, double leftStickY,
                                         double rightStickX, double rightStickY) {
        Mecanum.Wheels wheels = Mecanum.simpleJoystickToWheels (leftStickX, leftStickY, rightStickX, rightStickY);
        setDriveForMecanumWheels(wheels);
    }

    /**
     * Sets the drive chain power from Mecanum motion.
     * Maintains relative speeds when changing angles.
     * @param motion The desired Mecanum motion.
     */
    protected void setDriveForMecanum(Mecanum.Motion motion) {
        Mecanum.Wheels wheels = Mecanum.motionToWheels(motion);
        setDriveForMecanumWheels(wheels);
    }

    /**
     * Sets the drive chain power from Mecanum motion.
     * Uses max power output while changing speeds at angle motions.
     * @param motion The desired Mecanum motion.
     */
    protected void setDriveForMecanumForSpeed(Mecanum.Motion motion) {
        Mecanum.Wheels wheels = Mecanum.motionToWheels(motion).scaleWheelPower(
                Math.sqrt(2));
        setDriveForMecanumWheels(wheels);
    }

    public boolean driveToPosition(MecanumNavigation mecanumNavigation,
                                   MecanumNavigation.Navigation2D targetPosition,
                                   double rate) {
        double distanceThresholdInches = 1;
        double angleThresholdRadians = 2 * (2*Math.PI/180);
        rate = Range.clip(rate,0,1);
        MecanumNavigation.Navigation2D currentPosition =
                (MecanumNavigation.Navigation2D)mecanumNavigation.currentPosition.clone();
        MecanumNavigation.Navigation2D deltaPosition = targetPosition.minusEquals(currentPosition);

        // Not Close enough to target, keep moving
        if ( Math.abs(deltaPosition.x) > distanceThresholdInches ||
                Math.abs(deltaPosition.y) > distanceThresholdInches ||
                Math.abs(deltaPosition.theta) > angleThresholdRadians) {

            Mecanum.Wheels wheels = mecanumNavigation.deltaWheelsFromPosition(targetPosition);
            wheels.scaleWheelPower(rate);
            setDriveForMecanumWheels(wheels);
            return false;
        } else {  // Close enough
            setDriveForMecanumWheels(new Mecanum.Wheels(0,0,0,0));
            return true;
        }
    }



    // Declare all variables

    int armSegmentsNumber = 3;
    int armSegmentsLength = 10;


    //Convert the encoder ticks to degrees
    int motorPPS = 7; //seven pulses per seconds for an andy mark motor
    int armMotor1GearRatio = 40;
    int armMotor2GearRatio = 40;
    int armMotor3GearRatio = 40;
    public int armMotor1PosDegrees = 0;
    public int armMotor2PosDegrees = 0;
    public int armMotor3PosDegrees = 0;



    public void getMotorPositionDegrees() {
       int armMotor1TicksPerRotation = motorPPS * armMotor1GearRatio;
       int armMotor2TicksPerRotation = motorPPS * armMotor2GearRatio;
       int armMotor3TicksPerRotation = motorPPS * armMotor3GearRatio;

       int armMotor1PosEncoder = armMotor1.getCurrentPosition();
       int armMotor2PosEncoder = armMotor2.getCurrentPosition();
       int armMotor3PosEncoder = armMotor3.getCurrentPosition();

       armMotor1PosDegrees = (360 / armMotor1TicksPerRotation) * armMotor1PosEncoder;
       armMotor2PosDegrees = (360 / armMotor2TicksPerRotation) * armMotor2PosEncoder;
       armMotor3PosDegrees = (360 / armMotor3TicksPerRotation) * armMotor3PosEncoder;
    }



    //Initialize all motors
    //Arm motors start with the base = number 1, and increasing towards the top of the arm (
    // ex. if there are 3 segments, the bottom = 1, the middle = 2, and the top = 3.
    public DcMotor  armMotor1   = null;
    public DcMotor  armMotor2   = null;
    public DcMotor  armMotor3   = null;

    HardwareMap hwMap           =  null;
    private ElapsedTime period  = new ElapsedTime();
    public RobotHardware(){
    }

    public void init(HardwareMap ahwMap) {
        hwMap = ahwMap;

        // Define and Initialize Motors
        armMotor1 = hwMap.get(DcMotor.class, "armMotor1");
        armMotor2 = hwMap.get(DcMotor.class, "armMotor2");
        armMotor3 = hwMap.get(DcMotor.class, "armMotor3");

        //Set Motor Directions
        armMotor1.setDirection(DcMotor.Direction.FORWARD);
        armMotor2.setDirection(DcMotor.Direction.FORWARD);
        armMotor3.setDirection(DcMotor.Direction.FORWARD);

        // Set all motors to zero power
        armMotor1.setPower(0);
        armMotor2.setPower(0);
        armMotor3.setPower(0);

        // Set all motors to run with encoders.
        armMotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armMotor3.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    @Override
    public void init() {

    }

    @Override
    public void loop() {

    }

    // Format for displaying decimals.
    public DecimalFormat df;
    public DecimalFormat df_prec;

}
