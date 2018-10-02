package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Created by ryderswan on 10/1/18.
 */

public class robotHardware {

    // Declare all variables

    int armSegmentsNumber = 3;
    int armSegmentsLength = 10;


    //Convert the encoder ticks to degrees
    int motorPPS = 7; //seven pulses per seconds for an andy mark motor
    int armMotor1GearRatio = 40;
    int armMotor2GearRatio = 40;
    int armMotor3GearRatio = 40;
    public int armMotor1PosDegrees;
    public int armMotor2PosDegrees;
    public int armMotor3PosDegrees;



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
    public robotHardware(){
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

}
