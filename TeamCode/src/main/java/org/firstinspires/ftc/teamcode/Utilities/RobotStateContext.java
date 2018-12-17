package org.firstinspires.ftc.teamcode.Utilities;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.AutoOpmode;
import org.firstinspires.ftc.teamcode.RobotHardware;

public class RobotStateContext implements Executive.RobotStateMachineContextInterface {

    AutoOpmode opMode;
    Executive.StateMachine stateMachine;
    Waypoints waypoints;



    RobotStateContext(AutoOpmode opMode) {
        this.opMode = opMode;
        stateMachine = new Executive.StateMachine(opMode);

    }

    public void init() {
        stateMachine.changeState(Executive.StateMachine.StateType.DRIVE, new Start_State());
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
                nextState(Executive.StateMachine.StateType.DRIVE, new Drive_WaypointA_State());
            }
        }
    }

    class Drive_WaypointA_State extends Executive.StateBase {
        @Override
        public void update() {
            super.update();
            arrived = opMode.autoDrive.rotateThenDriveToPosition(new MecanumNavigation.Navigation2D(10,10,45), 0.8);
            if (arrived) {
                nextState(Executive.StateMachine.StateType.DRIVE, new FlagPrepState());
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
                nextState(Executive.StateMachine.StateType.DRIVE, new FlagDropState());
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
                nextState(Executive.StateMachine.StateType.DRIVE, new Stop_State());
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
                nextState(Executive.StateMachine.StateType.DRIVE, new Stop_State());
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
