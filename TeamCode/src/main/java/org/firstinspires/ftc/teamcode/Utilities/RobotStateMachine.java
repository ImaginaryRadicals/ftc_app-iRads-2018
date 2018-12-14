package org.firstinspires.ftc.teamcode.Utilities;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.teamcode.AutoOpmode;
import org.firstinspires.ftc.teamcode.RobotHardware;

import java.util.ArrayList;
import java.util.Arrays;

/**
 * Created by Ashley on 12/16/2017.
 */

public class RobotStateMachine {

    public enum AutoState {
        START,
        LAND,
        UNHOOK,
        DISMOUNT,
        IDENTIFY_CENTER,
        IDENTIFY_LEFT,
        IDENTIFY_RIGHT,
        ALIGN_CENTER_MINERAL,
        ALIGN_LEFT_MINERAL,
        ALIGN_RIGHT_MINERAL,
        UNKNOWN,
        KNOCK_GOLD_CENTER,
        KNOCK_GOLD_LEFT,
        KNOCK_GOLD_RIGHT,
        ALIGN_CENTER_DEPOT,
        ALIGN_LEFT_DEPOT,
        ALIGN_RIGHT_DEPOT,
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
    private Waypoints waypoints;

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

    public double speed = 1;
    public boolean foundMineral = false;
    public boolean centerGold = false;
    public boolean arrived = false;

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
        waypoints = new Waypoints(teamColor, startPosition, opMode.doPartnerMinerals.get());
    }

    public boolean driveMotorToPos (RobotHardware.MotorName motorName, int targetTicks, double power, double rampThreshold) {
        power = Range.clip(Math.abs(power), 0, 1);
        int poweredDistance = 0;
        int arrivedDistance = 50;
        //int rampThreshold = 400;
        double maxRampPower = 1.0;
        double minRampPower = 0.0;
        int errorSignal = opMode.getEncoderValue(motorName) - targetTicks;
        double direction = errorSignal > 0 ? -1.0: 1.0;
        double rampDownRatio = AutoDrive.rampDown(Math.abs(errorSignal), rampThreshold, maxRampPower, minRampPower);

        if (Math.abs(errorSignal) >= poweredDistance) {
            opMode.setPower(motorName, direction * power * rampDownRatio);
        } else {
            opMode.setPower(motorName, 0);
        }

        if(Math.abs(errorSignal) <= arrivedDistance) {
            return true;
        }else {
            return false;
        }
    }

    public boolean driveMotorToPos (RobotHardware.MotorName motorName, int targetTicks, double power) {
        int rampDistanceTicks = 400;
        return driveMotorToPos (motorName,targetTicks,power,rampDistanceTicks);
    }


    public void update() {

        double timeout = 6; // Identification timeout.
        lastStateLoopPeriod = stateLoopTimer.seconds();
        stateLoopTimer.reset();

        if (state != AutoState.CLAIM_DEPOT) {
            driveMotorToPos(RobotHardware.MotorName.ARM, 250, 1.0);
        }

        speed = opMode.AutoDriveSpeed.get();

        if (state == AutoState.START) {
            if (opMode.Simple.get()) {
                stateTimer.reset();
                state = AutoState.SIMPLE_START;

            } else {
                // This needs to be set based on our starting location. DEBUG
                opMode.mecanumNavigation.setCurrentPosition(waypoints.initialPosition);
                stateTimer.reset();
                state = AutoState.LAND;
            }
        } else if (state == AutoState.LAND) {
            arrived = driveMotorToPos(RobotHardware.MotorName.LIFT_WINCH, Constants.LIFTER_MAX_TICKS, 1.0);

            if (arrived || stateTimer.seconds() >= 8) {
                opMode.stopAllMotors();
                stateTimer.reset();
                state = AutoState.UNHOOK;
            }
        } else if (state == AutoState.UNHOOK) {
            arrived = opMode.autoDrive.rotateThenDriveToPosition(waypoints.unhookPosition, speed);
            if (arrived) {
                state = AutoState.DISMOUNT;
                stateTimer.reset();
            }
        } else if (state == AutoState.DISMOUNT) {

            arrived = opMode.autoDrive.rotateThenDriveToPosition(waypoints.dismountPosition, speed);

            // attempt to shake lift arm loose.
            // arrived = opMode.autoDrive.multiWaypointState("DISMOUNT",1.0, new ArrayList<>(Arrays.asList(
            //        new MecanumNavigation.Navigation2D(12.43+1,12.43+1,degreesToRadians(-45)),
            //        new MecanumNavigation.Navigation2D(12.43-1,12.43-1,degreesToRadians(-45)),
            //        new MecanumNavigation.Navigation2D(12.43,12.43,degreesToRadians(-30))
            // )));

            if (arrived) {
                stateTimer.reset();

                state = AutoState.ALIGN_CENTER_MINERAL;

            }
        } else if (state == AutoState.ALIGN_CENTER_MINERAL) {
            arrived = opMode.autoDrive.rotateThenDriveToPosition(waypoints.alignMineral_center, speed);
            if (arrived) {
                stateTimer.reset();
                state = AutoState.IDENTIFY_CENTER;
            }

        } else if (state == AutoState.IDENTIFY_CENTER) {
            arrived = opMode.autoDrive.rotateThenDriveToPosition(waypoints.scanMineral_center, speed);

            if (arrived && stateTimer.seconds() > 1 && stateTimer.seconds() < timeout) {
                // Detect mineral at image center
                centerMineral = opMode.simpleVision.identifyMineral(SimpleVision.MineralIdentificationLocation.BOTTOM);

                if (centerMineral == Color.Mineral.GOLD) {
                    stateTimer.reset();
                    state = AutoState.KNOCK_GOLD_CENTER;
                } else if(centerMineral == Color.Mineral.SILVER){
                    stateTimer.reset();
                    state = AutoState.IDENTIFY_LEFT;
                }

            } else if (stateTimer.seconds() >= timeout) {
                // Timed out: assume detection wasn't possible, act as if it were gold.
                stateTimer.reset();
                state = AutoState.STOP;
            }
        } else if (state == AutoState.IDENTIFY_LEFT) {
            // First rotate robot to point camera toward the left mineral.
            arrived = opMode.autoDrive.rotateThenDriveToPosition(waypoints.scanMineral_left, speed);
            // Detect mineral at image center
            leftMineral = opMode.simpleVision.identifyMineral(SimpleVision.MineralIdentificationLocation.BOTTOM);
            if (stateTimer.seconds() > 1 && stateTimer.seconds() < timeout) {
                if (leftMineral == Color.Mineral.GOLD) {
                    if (arrived) {
                        foundMineral = true;
                        stateTimer.reset();
                        state = AutoState.KNOCK_GOLD_LEFT;
                    }
                } else if (leftMineral == Color.Mineral.SILVER) {
                    stateTimer.reset();
                    state = AutoState.ALIGN_RIGHT_MINERAL;
                }
            } else if (stateTimer.seconds() >= timeout) {
                stateTimer.reset();
                state = AutoState.STOP;
            }
        } else if (state == AutoState.ALIGN_RIGHT_MINERAL) {

            // Detect mineral at image center
            arrived = opMode.autoDrive.rotateThenDriveToPosition(waypoints.alignMineral_right, speed);
            if(arrived && stateTimer.seconds() > 1 ) {
                    stateTimer.reset();
                    state = AutoState.KNOCK_GOLD_RIGHT;
            }

        } else if (state == AutoState.KNOCK_GOLD_CENTER)  {

            arrived = opMode.autoDrive.rotateThenDriveToPosition(waypoints.knockMineral_center, speed);
            if (arrived) {
                stateTimer.reset();
                state = AutoState.ALIGN_CENTER_DEPOT;
            }
        } else if (state == AutoState.KNOCK_GOLD_LEFT)  {
            arrived = opMode.autoDrive.rotateThenDriveToPosition(waypoints.knockMineral_left, speed);
            if (arrived) {
                stateTimer.reset();
                state = AutoState.ALIGN_LEFT_DEPOT;
            }
        } else if (state == AutoState.KNOCK_GOLD_RIGHT)  {
            arrived = opMode.autoDrive.rotateThenDriveToPosition(waypoints.knockMineral_right, speed);
            if (arrived) {
                stateTimer.reset();
                state = AutoState.ALIGN_RIGHT_DEPOT;
            }
        } else if (state == AutoState.ALIGN_CENTER_DEPOT)  {
            arrived = opMode.autoDrive.rotateThenDriveToPosition(waypoints.alignMineral_center, speed);
            if (arrived) {
                stateTimer.reset();
                state = AutoState.DRIVE_DEPOT;
            }
        } else if (state == AutoState.ALIGN_RIGHT_DEPOT)  {
            arrived = opMode.autoDrive.rotateThenDriveToPosition(waypoints.alignMineral_right, speed);
            if (arrived) {
                stateTimer.reset();
                state = AutoState.DRIVE_DEPOT;
            }
        } else if (state == AutoState.ALIGN_LEFT_DEPOT)  {
            arrived = opMode.autoDrive.rotateThenDriveToPosition(waypoints.alignMineral_left, speed);
            if (arrived) {
                stateTimer.reset();
                state = AutoState.DRIVE_DEPOT;
            }
        } else if (state == AutoState.DRIVE_DEPOT) {
            arrived = opMode.autoDrive.rotateThenDriveToPosition(waypoints.photoPosition, speed);
            if (arrived) {
                state = AutoState.CLAIM_DEPOT;
                stateTimer.reset();
            }

        } else if (state == AutoState.CLAIM_DEPOT) {
            arrived = opMode.autoDrive.rotateThenDriveToPosition(waypoints.flagDrop, speed);
            if (arrived) {
                arrived = driveMotorToPos(RobotHardware.MotorName.ARM, 8787, speed);
                if (arrived) {
                    stateTimer.reset();
                    state = AutoState.DRIVE_CRATER;
                }
            }

        } else if (state == AutoState.DRIVE_CRATER) {
            arrived = opMode.autoDrive.rotateThenDriveToPosition(waypoints.craterPark, speed);
                if (arrived) {
                    state = AutoState.STOP;
                    stateTimer.reset();
                }
        } else if (state == AutoState.SIMPLE_START) {
            // In these waypoint lists, the first entry is only used to set the
            // initial position for navigating.
            // The last position is moved to at full speed to get into the crater.
            // The intermediate positions are moved to at a reduced speed.
            // Motions first rotate, then translate.

            if (startPosition == RobotHardware.StartPosition.FIELD_CRATER) {
                simpleWaypointArray = new ArrayList<>(Arrays.asList(
                    // START POSITION (used to set mecanumNavigation initial position)
                    new MecanumNavigation.Navigation2D(19, 19, degreesToRadians(-45)),
                    // DISMOUNT (make space to turn)
                    new MecanumNavigation.Navigation2D(24, 24, degreesToRadians(-45)),
                    // ROTATE IN PLACE
                    new MecanumNavigation.Navigation2D(24, 24, degreesToRadians(45)),
                    // JUMP INTO CRATER (full speed waypoint)
                    new MecanumNavigation.Navigation2D(48, 48, degreesToRadians(45))
                ));

            } else {
                simpleWaypointArray = new ArrayList<>(Arrays.asList(
                    // START POSITION
                    new MecanumNavigation.Navigation2D(-19, 19, degreesToRadians(45)),
                    // DISMOUNT
                    new MecanumNavigation.Navigation2D(-24, 24, degreesToRadians(45)),
                    // ROTATE IN PLACE
                    new MecanumNavigation.Navigation2D(-24, 24, degreesToRadians(270)),
                    // HIT LEFT MINERAL
                    new MecanumNavigation.Navigation2D(-48, 24, degreesToRadians(270)),
                    // LINE UP WITH CRATER
                    new MecanumNavigation.Navigation2D(-60, 24, degreesToRadians(270)),
                    // JUMP INTO CRATER (full speed)
                    new MecanumNavigation.Navigation2D(-60, -48, degreesToRadians(270))

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
