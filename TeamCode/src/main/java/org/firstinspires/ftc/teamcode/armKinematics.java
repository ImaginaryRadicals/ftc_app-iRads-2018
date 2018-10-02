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
    int armPosX = 0;
    int armPosy = 0;
    int armPosz = 0;
    



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
        double seg1x = armSegmentsLength * Math.cos(armMotor1PosDegrees);
        double seg1y = armSegmentsLength * Math.sin(armMotor1PosDegrees);

        double seg2x = armSegmentsLength * Math.cos(armMotor2PosDegrees);
        double seg2y = armSegmentsLength * Math.sin(armMotor2PosDegrees);

        double seg3x = armSegmentsLength * Math.cos(armMotor3PosDegrees);
        double seg3y = armSegmentsLength * Math.sin(armMotor3PosDegrees);
    }



}
