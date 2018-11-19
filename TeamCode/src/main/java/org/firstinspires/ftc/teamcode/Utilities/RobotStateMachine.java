package org.firstinspires.ftc.teamcode.Utilities;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.teamcode.AutoOpmode;
import org.firstinspires.ftc.teamcode.RobotHardware;

import java.lang.reflect.Array;
import java.util.ArrayList;
import java.util.Arrays;

/**
 * Created by Ashley on 12/16/2017.
 */

public class RobotStateMachine {

    public enum AutoState {
        START,
        LAND,
        DISMOUNT,
        IDENTIFY_CENTER,
        IDENTIFY_LEFT,
        IDENTIFY_RIGHT,
        UNKNOWN,
        KNOCK_GOLD_CENTER,
        KNOCK_GOLD_LEFT,
        KNOCK_GOLD_RIGHT,
        DRIVE_DEPOT,
        CLAIM_DEPOT,
        DRIVE_CRATER,
        ENTER_CRATER,
        STOP,
        SIMPLE_CRATER,
        SIMPLE_DEPOT,
        SIMPLE_START,
    }

    public AutoState state = AutoState.START;
    private AutoOpmode opMode;
    private ElapsedTime stateLoopTimer = new ElapsedTime();
    private double lastStateLoopPeriod = 0;
    private ElapsedTime stateTimer = new ElapsedTime();

    // Colors
    public Color.Ftc teamColor = Color.Ftc.UNKNOWN;
    public Color.Mineral centerMineral = Color.Mineral.UNKNOWN;
    public Color.Mineral leftMineral = Color.Mineral.UNKNOWN;
    public Color.Mineral rightMineral = Color.Mineral.UNKNOWN;
    public SimpleVision.GoldMineralPosition goldMineralPosition = SimpleVision.GoldMineralPosition.UNKNOWN;

    // Kinematics
    private ArrayList<MecanumNavigation.Navigation2D> waypointArrayGlobal;

    public RobotHardware.StartPosition startPosition;
    private int currentDriveWaypoint = 0;
    private double driveRate = 0.5;
    public double speed = 0.5;
    public boolean foundMineral = false;
    public boolean centerGold = false;

    private ArrayList<MecanumNavigation.Navigation2D> simpleWaypointArray;

    DcMotor armMotor;


    public RobotStateMachine(AutoOpmode opMode, Color.Ftc teamColor, RobotHardware.StartPosition startPosition) {
        this.opMode = opMode;
        this.teamColor = teamColor;
        this.startPosition = startPosition;
    }


    public void init() {
        stateLoopTimer.reset();
        stateTimer.reset();
    }


    public void update() {

        lastStateLoopPeriod = stateLoopTimer.seconds();
        stateLoopTimer.reset();

        if (state == AutoState.START) {
            if (opMode.Simple.get()) {
                stateTimer.reset();
                state = AutoState.SIMPLE_START;

            } else {
                stateTimer.reset();
                state = AutoState.LAND;
            }
        } else if (state == AutoState.LAND) {
            stateTimer.reset();
            state = AutoState.DISMOUNT;
        } else if (state == AutoState.DISMOUNT) {

            boolean arrived = opMode.autoDrive.rotateThenDriveToPosition(new MecanumNavigation.Navigation2D(5, 0, degreesToRadians(0)), speed);

            if (stateTimer.seconds() >= 10 || arrived) {
                stateTimer.reset();

                state = AutoState.IDENTIFY_CENTER;

            }
        } else if (state == AutoState.IDENTIFY_CENTER) {
            if (goldMineralPosition == SimpleVision.GoldMineralPosition.CENTER) {
                stateTimer.reset();

                foundMineral = true;
                centerGold = true;

                state = AutoState.KNOCK_GOLD_CENTER;


            } else {
                state = AutoState.IDENTIFY_LEFT;
            }

        } else if (state == AutoState.IDENTIFY_LEFT) {
                if (goldMineralPosition == SimpleVision.GoldMineralPosition.LEFT) {
                    stateTimer.reset();

                    foundMineral = true;

                    state = AutoState.KNOCK_GOLD_LEFT;

                } else {
                    state = AutoState.IDENTIFY_RIGHT;
                }


        } else if (state == AutoState.IDENTIFY_RIGHT) {
            if (goldMineralPosition == SimpleVision.GoldMineralPosition.RIGHT) {
                stateTimer.reset();

                foundMineral = true;

                state = AutoState.KNOCK_GOLD_RIGHT;

            } else {
                state = AutoState.UNKNOWN;
            }

        } else if (state == AutoState.UNKNOWN) {
            if (goldMineralPosition == SimpleVision.GoldMineralPosition.UNKNOWN) {

                foundMineral = false;

                state = AutoState.UNKNOWN;
                
            } else {
                state = AutoState.IDENTIFY_CENTER;
            }


        } else if (state == AutoState.KNOCK_GOLD_CENTER || state == AutoState.KNOCK_GOLD_RIGHT || state == AutoState.KNOCK_GOLD_LEFT) {
            if (foundMineral && goldMineralPosition == SimpleVision.GoldMineralPosition.CENTER) {
                stateTimer.reset();

                opMode.autoDrive.rotateThenDriveToPosition(new MecanumNavigation.Navigation2D(8, 8, degreesToRadians(0)), speed);

            } else if (foundMineral && goldMineralPosition == SimpleVision.GoldMineralPosition.RIGHT) {
                stateTimer.reset();

                opMode.autoDrive.rotateThenDriveToPosition(new MecanumNavigation.Navigation2D(0, 0, degreesToRadians(0)), speed);

            } else if (foundMineral && goldMineralPosition == SimpleVision.GoldMineralPosition.LEFT) {
                stateTimer.reset();

                opMode.autoDrive.rotateThenDriveToPosition(new MecanumNavigation.Navigation2D(0, 0, degreesToRadians(0)), speed);

            } else {

                state = AutoState.IDENTIFY_CENTER;
            }
        } else if (state == AutoState.DRIVE_DEPOT) {
            stateTimer.reset();

            opMode.autoDrive.rotateThenDriveToPosition(new MecanumNavigation.Navigation2D(0, 0, degreesToRadians(0)), speed);

        } else if (state == AutoState.SIMPLE_START) {

            if (startPosition == RobotHardware.StartPosition.FIELD_CRATER) {
                simpleWaypointArray = new ArrayList<>(Arrays.asList(
                    // DISMOUNT
                    new MecanumNavigation.Navigation2D(19, 19, degreesToRadians(-135)),

                    new MecanumNavigation.Navigation2D(24, 24, degreesToRadians(-135)),

                    new MecanumNavigation.Navigation2D(24, 24, degreesToRadians(-45)),

                    new MecanumNavigation.Navigation2D(48, 48, degreesToRadians(-45))
                ));

            } else {
                simpleWaypointArray = new ArrayList<>(Arrays.asList(

                        new MecanumNavigation.Navigation2D(-19, 19, degreesToRadians(-225)),

                        new MecanumNavigation.Navigation2D(-24, 24, degreesToRadians(-255)),

                        new MecanumNavigation.Navigation2D(-24, 24, degreesToRadians(-270)),

                        new MecanumNavigation.Navigation2D(-48, 24, degreesToRadians(-270)),

                        new MecanumNavigation.Navigation2D(-60, 24, degreesToRadians(-270)),

                        new MecanumNavigation.Navigation2D(-60, -48, degreesToRadians(-270))

                ));
            }

            currentDriveWaypoint = 1;
            opMode.mecanumNavigation.setCurrentPosition( simpleWaypointArray.get(0));
            stateTimer.reset();
            if (startPosition == RobotHardware.StartPosition.FIELD_CRATER) {
                state = AutoState.SIMPLE_CRATER;
            } else {
                state = AutoState.SIMPLE_DEPOT;
            }


        } else if (state == AutoState.SIMPLE_CRATER ) {
            boolean done = false;

            if (!opMode.controller.X()) { // Pause Robot while X button is held.
                done = simpleWaypointDrive(simpleWaypointArray);
            } else {
                opMode.stopAllMotors();
            }

            if(done) {
                stateTimer.reset();
                state = AutoState.STOP;
                opMode.stop();
            }


        } else if (state == AutoState.SIMPLE_DEPOT) {
            state = AutoState.SIMPLE_CRATER; // They are the same right now.
            
        } else if (state == AutoState.STOP) {
            opMode.stopAllMotors();
            opMode.stop();
        } else {
            // error
            opMode.stopAllMotors();
        }
    }
    
    private double degreesToRadians(double degrees) {
        return degrees * Math.PI / 180;
    }

    private double radiansToDegrees(double radians) {
        return radians * 180 / Math.PI;
    }

    private boolean simpleWaypointDrive(ArrayList<MecanumNavigation.Navigation2D> waypointList) {
        boolean arrived = false;
        boolean finalArrived = false;
        // Skip first point, used for setting navigation start in SIMPLE_START
        if (currentDriveWaypoint < 1) {
            currentDriveWaypoint = 1;
        }
        // So we can ramp up speed on LAST waypoint to get into CRATER
        int lastIndex = waypointList.size() - 1;
        int secondToLastIndex = waypointList.size() - 2;
        if (currentDriveWaypoint <= secondToLastIndex){
            arrived = opMode.autoDrive.rotateThenDriveToPosition(waypointList.get(currentDriveWaypoint),0.5);
            if(arrived) {
                currentDriveWaypoint++;
            }
        }
        if (currentDriveWaypoint == lastIndex) {
            // Ramming SPEED! Get over crater.
            finalArrived = opMode.autoDrive.rotateThenDriveToPosition(waypointList.get(lastIndex),1.0);
        }
        if (finalArrived) {
            opMode.stopAllMotors();
        }
        opMode.telemetry.addData("Simple Waypoint Drive","");
        opMode.telemetry.addData("Current Waypoint: ", currentDriveWaypoint);
        opMode.telemetry.addData("Target", waypointList.get(currentDriveWaypoint).toString());
        return finalArrived;
    }


    private ArrayList<MecanumNavigation.Navigation2D>
            generateWaypoints(Color.Ftc teamColor, RobotHardware.StartPosition startPosition,
                                RelicRecoveryVuMark positionVumark, double insertionSkewRadiansCCW) {
        ArrayList<MecanumNavigation.Navigation2D> waypointArray = new ArrayList<>(Arrays.asList(new MecanumNavigation.Navigation2D(0,0,0)));

        double dismountBlueDistance = 24;
        double dismountRedDistance = 25;
        double alignmentStrafeCorner = 12;
        double alignmentDriveCenter = 22 - 6 + 2 - 6.25;
        double approachCorner = 0;
        double approachCenter = 0;
        double insertCorner = 12;
        double insertCenter = 12;
        double backupDistance = 4;

        // Select Driving waypoints
        if (teamColor == Color.Ftc.BLUE) {
            if (startPosition == RobotHardware.StartPosition.FIELD_CRATER) {
                /** Blue Crater */
                waypointArray = new ArrayList<>(Arrays.asList(
                    // DISMOUNT
                    new MecanumNavigation.Navigation2D(0,
                            0, degreesToRadians(0))
                ));
            } else if (startPosition == RobotHardware.StartPosition.FIELD_DEPOT) {
                /** Blue Depot */
                waypointArray = new ArrayList<>(Arrays.asList(
                        // DISMOUNT
                        new MecanumNavigation.Navigation2D(0,
                                0, degreesToRadians(0))
                ));
            }
        } else if (teamColor == Color.Ftc.RED) {
            if (startPosition == RobotHardware.StartPosition.FIELD_CRATER) {
                /** Red Corner */
                waypointArray = new ArrayList<>(Arrays.asList(
                        // DISMOUNT
                        new MecanumNavigation.Navigation2D(0,
                                0, degreesToRadians(0))
                ));
            } else if (startPosition == RobotHardware.StartPosition.FIELD_DEPOT) {
                /** Red Center */
                waypointArray = new ArrayList<>(Arrays.asList(
                        // DISMOUNT
                        new MecanumNavigation.Navigation2D(0,
                                0, degreesToRadians(0))
                ));
            }
        } else {
            opMode.stopAllMotors();
            stateTimer.reset();
            state = AutoState.STOP;
        }

        return waypointArray;
    }




    private MecanumNavigation.Navigation2D getGlyphOffsetFromRotation(double rotationRadians) {
        double distanceToGlyphCenter = 12+6-7+1.5; // Added half of glyph width to lever arm.
        return new MecanumNavigation.Navigation2D( distanceToGlyphCenter * ( Math.cos(rotationRadians) - 1), distanceToGlyphCenter * Math.sin(rotationRadians), rotationRadians);
    }

    /**
     * Get the number of inches the robot needs to slide toward the right Cryptobox column
     * in order to line the glyph up with the desired column, at the desired skew angle.
     * @param vumarkPosition
     * @param skewAngleRadiansCCW
     * @return
     */
    private double getGlyphboxOffsetTowardRight( RelicRecoveryVuMark vumarkPosition, double skewAngleRadiansCCW) {
        MecanumNavigation.Navigation2D glyphOffsetFromRotation = getGlyphOffsetFromRotation(skewAngleRadiansCCW);
        double columnWidth = 7.63;
        double offsetRightTotal = 0;
        if (vumarkPosition == RelicRecoveryVuMark.LEFT) {
            offsetRightTotal -= columnWidth;
        } else if (vumarkPosition == RelicRecoveryVuMark.RIGHT) {
            offsetRightTotal += columnWidth;
        }
        offsetRightTotal += glyphOffsetFromRotation.y;

        return offsetRightTotal;
    }

    private boolean driveToWaypointAtRate(int waypointNumber, double driveRate) {
        // Show Target Status and debug info
        opMode.telemetry.addData("Current Waypoint: ", currentDriveWaypoint);
        opMode.telemetry.addData("Target", waypointArrayGlobal.get(waypointNumber).toString());
        return opMode.autoDrive.rotateThenDriveToPosition(waypointArrayGlobal.get(waypointNumber),driveRate);
    }

}
