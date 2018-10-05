package org.firstinspires.ftc.teamcode;

/**
 * Created by ryderswan on 10/1/18.
 *
 * Calculate the arm position in three dimensional space
 */

import java.lang.Math;

public class armKinematics extends RobotHardware {



    public static class Motors {
        // The arm Motors.
        public double one;
        public double two;
        public double three;
    }

    //Final Arm Position
    double armPosX = 0;
    double armPosY = 0;
    double armPosZ = 0;

    double robotAngle = 0;

    public void getMotorPositionDegrees() {

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
