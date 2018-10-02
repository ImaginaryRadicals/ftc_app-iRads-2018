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




    //Initialize all motors
    //Arm motors start with the base = number 1, and increasing towards the top of the arm (
    // ex. if there are 3 segments, the bottom = 1, the middle = 2, and the top = 3.
    public DcMotor  armMotor1   = null;
    public DcMotor  armMotor2   = null;
    public DcMotor  armMotor3   = null;

    /* local OpMode members. */
    HardwareMap hwMap           =  null;
    private ElapsedTime period  = new ElapsedTime();
    /* Constructor */
    public robotHardware(){
    }

    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
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
