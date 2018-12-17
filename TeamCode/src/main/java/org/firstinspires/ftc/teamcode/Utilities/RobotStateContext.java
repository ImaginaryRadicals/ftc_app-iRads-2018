package org.firstinspires.ftc.teamcode.Utilities;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.AutoOpmode;
import org.firstinspires.ftc.teamcode.RobotHardware;

import static org.firstinspires.ftc.teamcode.Utilities.Executive.StateMachine.StateType.DRIVE;

public class RobotStateContext implements Executive.RobotStateMachineContextInterface {

    AutoOpmode opMode;
    Executive.StateMachine stateMachine;
    Color.Ftc teamColor;
    RobotHardware.StartPosition startPosition;
    Waypoints waypoints;
    double driveSpeed = 0.8;


    public RobotStateContext(AutoOpmode opMode, Color.Ftc teamColor, RobotHardware.StartPosition startPosition) {
        this.opMode = opMode;
        this.teamColor = teamColor;
        this.startPosition = startPosition;
        stateMachine = new Executive.StateMachine(opMode);
        waypoints = new Waypoints(teamColor, startPosition, true);
    }

    public void init() {
        stateMachine.changeState(DRIVE, new Start_State());
        stateMachine.changeState(Executive.StateMachine.StateType.ARM, new ArmLevelState());
        stateMachine.init();
    }

    public void update() {
        stateMachine.update();
    }

    public String getCurrentState() {
        return stateMachine.getCurrentState();
    }

    /**
     * Define Concrete State Classes
     */

    class ArmLevelState extends Executive.StateBase {
        @Override
        public void update() {
            super.update();
            arrived = opMode.autoDrive.driveMotorToPos(RobotHardware.MotorName.ARM, 250, 1.0, 100);
        }
    }


    class Start_State extends Executive.StateBase {
        @Override
        public void update() {
            super.update();
            if(stateTimer.seconds() > 1) {
                opMode.mecanumNavigation.setCurrentPosition(waypoints.initialPosition);
                opMode.imuUtilities.updateNow();
                opMode.imuUtilities.setCompensatedHeading(180/Math.PI*waypoints.initialPosition.theta);
                nextState(DRIVE, new DescendLander());
            }
        }
    }

    class DescendLander extends Executive.StateBase {
        @Override
        public void update() {
            super.update();
            arrived = opMode.autoDrive.driveMotorToPos(RobotHardware.MotorName.LIFT_WINCH, 3000, 1.0, 100);
            if (arrived) {
                opMode.imuUtilities.updateMecanumHeadingFromGyro(opMode.mecanumNavigation);
                nextState(DRIVE, new Unhook());
            }
        }
    }

    class Unhook extends Executive.StateBase {
        @Override
        public void update() {
            super.update();
            arrived = opMode.autoDrive.rotateThenDriveToPosition(waypoints.unhookPosition, driveSpeed);
            if (arrived) {
                opMode.imuUtilities.updateMecanumHeadingFromGyro(opMode.mecanumNavigation);
                nextState(DRIVE, new Dismount());
            }
        }
    }
    class Dismount extends Executive.StateBase {
        @Override
        public void update() {
            super.update();
            arrived = opMode.autoDrive.rotateThenDriveToPosition(waypoints.dismountPosition, driveSpeed);
            if (arrived) {
                nextState(DRIVE, new ScanCenter());
            }
        }
    }

    class ScanCenter extends Executive.StateBase {
        ElapsedTime identificationTimer = new ElapsedTime();
        double identificationTime = 2;
        double timeout = 6;
        Color.Mineral mineralColor = Color.Mineral.UNKNOWN;

        @Override
        public void init(Executive.StateMachine stateMachine) {
            super.init(stateMachine);
            identificationTimer.reset();
        }

        @Override
        public void update() {
            super.update();
            arrived = opMode.autoDrive.rotateThenDriveToPosition(waypoints.scanMineral_center, driveSpeed);
            if (!arrived) {
                identificationTimer.reset();
            }
            mineralColor = opMode.simpleVision.identifyMineral(SimpleVision.MineralIdentificationLocation.BOTTOM);
            if (arrived && identificationTimer.seconds() > identificationTime && stateTimer.seconds() < timeout) {
                if(mineralColor == Color.Mineral.GOLD) {
                    nextState(DRIVE, new HitCenter());
                } else if (mineralColor == Color.Mineral.SILVER) {
                    nextState(DRIVE, new ScanLeft());
                } else {
                    // UNKNOWN
                    nextState(DRIVE, new HitCenter()); // Guess, keep moving.
                }
            } else if (stateTimer.seconds() >= timeout) {
                // Next state - failed identification
                nextState(DRIVE, new HitCenter()); // Guess, keep moving.
            }
        }
    }

    class HitCenter extends Executive.StateBase {
        @Override
        public void update() {
            super.update();
            arrived = opMode.autoDrive.rotateThenDriveToPosition(waypoints.knockMineral_center, driveSpeed);
            if (arrived) {
                nextState(DRIVE, new ScanCenter());
            }
        }
    }

    class ScanLeft extends Executive.StateBase {
        ElapsedTime identificationTimer = new ElapsedTime();
        double identificationTime = 2;
        double timeout = 6;
        Color.Mineral mineralColor = Color.Mineral.UNKNOWN;

        @Override
        public void init(Executive.StateMachine stateMachine) {
            super.init(stateMachine);
            identificationTimer.reset();
        }

        @Override
        public void update() {
            super.update();
            arrived = opMode.autoDrive.rotateThenDriveToPosition(waypoints.alignMineral_left, driveSpeed);
            if (!arrived) {
                identificationTimer.reset();
            }
            mineralColor = opMode.simpleVision.identifyMineral(SimpleVision.MineralIdentificationLocation.BOTTOM);
            if (arrived && identificationTimer.seconds() > identificationTime && stateTimer.seconds() < timeout) {
                if(mineralColor == Color.Mineral.GOLD) {
                    nextState(DRIVE, new AlignLeft());
                } else if (mineralColor == Color.Mineral.SILVER) {
                    nextState(DRIVE, new AlignRight());
                } else {
                    // UNKNOWN
                    nextState(DRIVE, new AlignLeft()); // Guess, keep moving.
                }
            } else if (stateTimer.seconds() >= timeout) {
                // Next state - failed identification
                nextState(DRIVE, new AlignLeft()); // Guess, keep moving.
            }
        }
    }

    class AlignLeft extends Executive.StateBase {
        @Override
        public void update() {
            super.update();
            arrived = opMode.autoDrive.rotateThenDriveToPosition(waypoints.alignMineral_left, driveSpeed);
            if (arrived) {
                nextState(DRIVE, new HitLeft());
            }
        }
    }

    class HitLeft extends Executive.StateBase {
        @Override
        public void update() {
            super.update();
            arrived = opMode.autoDrive.rotateThenDriveToPosition(waypoints.knockMineral_left, driveSpeed);
            if (arrived) {
                nextState(DRIVE, new FlagPrepState()); // FIX
            }
        }
    }

    class AlignRight extends Executive.StateBase {
        @Override
        public void update() {
            super.update();
            arrived = opMode.autoDrive.rotateThenDriveToPosition(waypoints.alignMineral_right, driveSpeed);
            if (arrived) {
                nextState(DRIVE, new KnockRight());
            }
        }
    }

    class KnockRight extends Executive.StateBase {
        @Override
        public void update() {
            super.update();
            arrived = opMode.autoDrive.rotateThenDriveToPosition(waypoints.knockMineral_right, driveSpeed);
            if (arrived) {
                nextState(DRIVE, new ScanCenter());
            }
        }
    }



    class FlagPrepState extends Executive.StateBase {
        // Move arm to a high position, preparing to drop flag, but not dropping it yet.
        boolean driveArrived;
        boolean armArrived;

        @Override
        public void init(Executive.StateMachine stateMachine) {
            super.init(stateMachine);
            stateMachine.removeStateType(Executive.StateMachine.StateType.ARM); // Stop arm hold state.
        }

        @Override
        public void update() {
            super.update();
            // Hold same position
            driveArrived = opMode.autoDrive.rotateThenDriveToPosition(new MecanumNavigation.Navigation2D(10,10,45), 0.8);
            armArrived = opMode.autoDrive.driveMotorToPos(RobotHardware.MotorName.ARM, 2000, 1.0, 400);
            if (driveArrived && armArrived) {
                nextState(DRIVE, new FlagDropState());
            }
        }
    }


    class FlagDropState extends Executive.StateBase {
        boolean armArrived;

        @Override
        public void update() {
            super.update();
            // Hold same position
//            arrived = opMode.autoDrive.rotateThenDriveToPosition(new MecanumNavigation.Navigation2D(10,10,45), 0.8);
            armArrived = opMode.autoDrive.driveMotorToPos(RobotHardware.MotorName.ARM, 5000, 1.0);
            if (armArrived) {
                nextState(DRIVE, new Stop_State());
                nextState(Executive.StateMachine.StateType.ARM, new ArmLevelState());
            }
        }
    }

    class ReturnHome extends Executive.StateBase {
        @Override
        public void update() {
            super.update();
            arrived = opMode.autoDrive.rotateThenDriveToPosition(new MecanumNavigation.Navigation2D(0,0,0),1.0);
            if(arrived) {
                nextState(DRIVE, new Stop_State());
            }
        }
    }



    class Stop_State extends Executive.StateBase {
        @Override
        public void update() {
            super.update();
            opMode.stopAllMotors();
        }
    }




}
