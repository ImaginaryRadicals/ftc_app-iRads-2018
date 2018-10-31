package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Utilities.Color;

import java.text.DecimalFormat;
import java.util.ArrayList;
import java.util.Vector;

public class RobotHardware extends OpMode {

    // All motors on the robot, in order of MotorName.
    private ArrayList<DcMotor> allMotors;

    // All servos on the robot, in order of ServoName.
    private ArrayList<Servo> allServos;

    // Names of only the drive motors.
    private ArrayList<MotorName> driveMotorNames;

    // All color sensors on the robot, in order of ColorSensorName.
    private ArrayList<ColorSensor> allColorSensors;

    // Format for displaying decimals.
    public DecimalFormat df;
    public DecimalFormat df_prec;


    // IMU reference
    public BNO055IMU imu;

    // Execution cycle period monitor.
    private ElapsedTime period = new ElapsedTime();
    private Vector<Double> pastPeriods = new Vector();


    // The motors on the robot.
    public enum MotorName {
        DRIVE_FRONT_LEFT,
        DRIVE_FRONT_RIGHT,
        DRIVE_BACK_LEFT,
        DRIVE_BACK_RIGHT,
        ARM,
        WRIST,
        FEEDER,
    }

    /**
     * Sets the power of the motor.
     *
     * @param motor The motor to modify.
     * @param power The power to set [-1, 1].
     */
    public void setPower(MotorName motor, double power) {
        DcMotor m = allMotors.get(motor.ordinal());
        if (m == null) {
            telemetry.addData("Motor Missing", motor.name() + ": " + df.format(power));
        } else {
            m.setPower(power);
        }
    }

    /**
     * Get motor power
     *
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
     *
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
     *
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

    /**
     * Set zero power braking behavior for all drive wheels.
     *
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


    /**
     * Sets the drive chain power.
     *
     * @param left  The power for the left two motors.
     * @param right The power for the right two motors.
     */
    public void setDriveForTank(double left, double right) {
        setPower(MotorName.DRIVE_FRONT_LEFT, left);
        setPower(MotorName.DRIVE_BACK_LEFT, left);
        setPower(MotorName.DRIVE_FRONT_RIGHT, right);
        setPower(MotorName.DRIVE_BACK_RIGHT, right);
    }

    // The servos on the robot.
    public enum ServoName {
        FLIPPER_RIGHT,
        FIPPER_LEFT,
    }

    // Servo methods

    /**
     * Sets the angle of the servo.
     *
     * @param servo    The servo to modify.
     * @param position The angle to set [0, 1].
     */
    public void setAngle(ServoName servo, double position) {
        Servo s = allServos.get(servo.ordinal());
        if (s == null) {
            telemetry.addData("Servo Missing", servo.name() + ": " + df.format(position));
        } else {
            s.setPosition(position);
        }
    }

    /**
     * Get the position of a servo.
     *
     * @param servo ServoName enum to check
     * @return double servo position [0,1]
     */
    public double getAngle(ServoName servo) {
        Servo s = allServos.get(servo.ordinal());
        if (s == null) {
            telemetry.addData("Servo Missing", servo.name());
            return -1;
        } else {
            return s.getPosition();
        }
    }

    // The color sensors on the robot.
    public enum ColorSensorName {
        JEWEL_COLOR,
    }

    /**
     * Gets the color value on the sensor.
     *
     * @param sensor The sensor to read.
     * @param color  The color channel to read intensity.
     */
    public int getColorSensor(ColorSensorName sensor, Color.Channel color) {
        ColorSensor s = allColorSensors.get(sensor.ordinal());
        if (s == null) {
            telemetry.addData("Color Sensor Missing", sensor.name());
            return 0;
        }

        switch (color) {
            case RED:
                return s.red();
            case GREEN:
                return s.green();
            case BLUE:
                return s.blue();
            case ALPHA:
                return s.alpha();
            default:
                return 0;
        }
    }

    /**
     * Sets the LED power for the color sensor.
     *
     * @param sensor  The sensor to set the LED power.
     * @param enabled Whether to turn the LED on.
     */
    public void setColorSensorLedEnabled(ColorSensorName sensor,
                                         boolean enabled) {
        ColorSensor s = allColorSensors.get(sensor.ordinal());
        if (s == null) {
            telemetry.addData("Color Sensor Missing", sensor.name());
        } else {
            s.enableLed(enabled);
        }
    }

    /**
     * Checks JEWEL_COLOR sensor and returns enum based on color detected.
     *
     * @return RED, BLUE, or null
     */
    public Color.Mineral getMineralColor() {
        int red = getColorSensor(ColorSensorName.JEWEL_COLOR, Color.Channel.RED);
        int blue = getColorSensor(ColorSensorName.JEWEL_COLOR, Color.Channel.BLUE);
        if (red > blue) {
            return Color.Mineral.GOLD;
        } else if (blue > red) {
            return Color.Mineral.SILVER;
        } else {
            return Color.Mineral.UNKNOWN;
        }
    }

    public void displayColorSensorTelemetry() {
        telemetry.addData("Color RED", getColorSensor(ColorSensorName.JEWEL_COLOR, Color.Channel.RED));
        telemetry.addData("Color BLUE", getColorSensor(ColorSensorName.JEWEL_COLOR, Color.Channel.BLUE));
        telemetry.addData("Jewel Color:", getMineralColor().toString());
    }

    // Possible starting positions.
    public enum StartPosition {
        FIELD_CRATER,
        FIELD_DEPOT,
    }

    /**
     * Gets the Vuforia license key.
     */
    protected String getVuforiaLicenseKey() {
        String vuforiaLicenseKey = "AUQ7leT/////AAAAGbE5ttrmO0iOg4xdJTnQehMYDMxLvRqCeEEhtqeZWJzzoNESAE9U6OUW7BmVwUSNmsVtZb1p6ALNdMJnozgpwyLM98L/E2+omz7xJqvSsDqnhlDqFUeoTd4xKyVjcKinMPzkkvFbJHrh9bHWXqvY3Z68QtNbJiiyLLvXuFmk/Y/ZnFBzUT7fZzuQsceQZJVbvmokgb+TRN8Wy+RHRYtOhHznJOVOdxTp2OEHY1nLWwq0trt4ozfzzpu/8Mk2Vym/gKaZk9cyAA0tyduKk5r+6Zx+o/mUPN7Ox5qjhXOaYxz1amH05ieZOPSu8MXSM47L+5WxD4riIfPBY2fjfrFtq4EXyhTo9VjHD0gd1N0cXbaw";
        return vuforiaLicenseKey;
    }

    // IMU Names (Could support multiple REV IMUs)
    public enum IMUNames {
        IMU,
    }

    public void init() {

        allMotors = new ArrayList<DcMotor>();
        for (MotorName m : MotorName.values()) {
            try {
                allMotors.add(hardwareMap.get(DcMotor.class, m.name()));
            } catch (Exception e) {
                telemetry.addData("Motor Missing", m.name());
                allMotors.add(null);
            }
        }

        // Collect a list of only the drive motors.
        driveMotorNames = new ArrayList<MotorName>();
        driveMotorNames.add(MotorName.DRIVE_FRONT_LEFT);
        driveMotorNames.add(MotorName.DRIVE_FRONT_RIGHT);
        driveMotorNames.add(MotorName.DRIVE_BACK_LEFT);
        driveMotorNames.add(MotorName.DRIVE_BACK_RIGHT);

        // Set motor directions.
        try {
            allMotors.get(MotorName.DRIVE_FRONT_RIGHT.ordinal()).setDirection(DcMotor.Direction.REVERSE);
            allMotors.get(MotorName.DRIVE_BACK_RIGHT.ordinal()).setDirection(DcMotor.Direction.REVERSE);
            allMotors.get(MotorName.ARM.ordinal()).setDirection(DcMotor.Direction.FORWARD);
            allMotors.get(MotorName.WRIST.ordinal()).setDirection(DcMotor.Direction.FORWARD);
            allMotors.get(MotorName.FEEDER.ordinal()).setDirection(DcMotor.Direction.FORWARD);
        } catch (Exception e) {
            telemetry.addData("Unable to set motor direction", "");
        }

        // Set drive motors to use encoders
        setDriveMotorsRunMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Set drive motors to float instead of brake when power is zero.
        setDriveMotorsZeroPowerBraking(false);

        // Set arm motor to brake
        try {
            allMotors.get(MotorName.ARM.ordinal()).setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            allMotors.get(MotorName.ARM.ordinal()).setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            allMotors.get(MotorName.WRIST.ordinal()).setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            allMotors.get(MotorName.WRIST.ordinal()).setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            allMotors.get(MotorName.FEEDER.ordinal()).setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            allMotors.get(MotorName.FEEDER.ordinal()).setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        } catch (Exception e) {
            telemetry.addData("Unable to set arm motor to zero power brake or encoder use", "");
        }

        allServos = new ArrayList<Servo>();
        for (ServoName s : ServoName.values()) {
            try {
                allServos.add(hardwareMap.get(Servo.class, s.name()));
            } catch (Exception e) {
                telemetry.addData("Servo Missing", s.name());
                allServos.add(null);
            }
        }
        // Set servo direction
        try {
            allServos.get(ServoName.FIPPER_LEFT.ordinal()).setDirection(Servo.Direction.REVERSE);
        } catch (Exception e) {
            telemetry.addData("Unable to set left servo direction", "");
        }

        allColorSensors = new ArrayList<ColorSensor>();
        for (ColorSensorName s : ColorSensorName.values()) {
            try {
                allColorSensors.add(hardwareMap.get(ColorSensor.class,
                        s.name()));
            } catch (Exception e) {
                telemetry.addData("Color Sensor Missing", s.name());
                allColorSensors.add(null);
            }
        }

        df = new DecimalFormat("0.00");
        df_prec = new DecimalFormat("0.0000");

        stopAllMotors();
        period.reset(); // Reset timer
    }

    public void start() {
        stopAllMotors();
        period.reset(); // Reset timer
    }

    public void loop() {
        //updatePeriodTime();
    }

    public void stop() {
        super.stop();

        for (MotorName m : MotorName.values()) {
            setPower(m, 0);
        }
        for (ColorSensorName s : ColorSensorName.values()) {
            setColorSensorLedEnabled(s, false);
        }
    }
}

