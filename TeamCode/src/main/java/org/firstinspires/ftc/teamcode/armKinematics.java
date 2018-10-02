package org.firstinspires.ftc.teamcode;

/**
 * Created by ryderswan on 10/1/18.
 *
 * Calculate the arm position in three dimensional space
 */

import java.lang.Math;

public class armKinematics extends robotHardware {

    //Declare all variables

    //Hardware variables
    int armSegmentsNumber = 3;
    double armSegmentsLength = 10;

    //Final Arm Position
    double armPosX = 0;
    double armPosY = 0;
    double armPosZ = 0;

    //Change this after writing the robot tracking code
    double robotAngle = 0;




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


    //Code
    public void armTracking() {

        double seg1x = armSegmentsLength * Math.sin(armMotor1PosDegrees);
        double seg1y = armSegmentsLength * Math.cos(armMotor1PosDegrees);

        double seg2x = armSegmentsLength * Math.sin(armMotor2PosDegrees);
        double seg2y = armSegmentsLength * Math.cos(armMotor2PosDegrees);

        double seg3x = armSegmentsLength * Math.sin(armMotor3PosDegrees);
        double seg3y = armSegmentsLength * Math.cos(armMotor3PosDegrees);

        armPosX = 0;
        armPosY = seg1x + seg2x + seg3x;
        armPosZ = seg1y + seg2y + seg3y;
    }



}
