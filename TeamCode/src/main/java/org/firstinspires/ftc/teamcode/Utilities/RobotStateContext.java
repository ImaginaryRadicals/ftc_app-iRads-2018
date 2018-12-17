package org.firstinspires.ftc.teamcode.Utilities;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.AutoOpmode;

public class RobotStateContext implements Executive.RobotStateMachineContextInterface {

    AutoOpmode opMode;
    Executive.StateMachine stateMachine;



    RobotStateContext(AutoOpmode opMode) {
        this.opMode = opMode;
        stateMachine = new Executive.StateMachine(opMode);

    }

    public void init() {
        Executive.StateBase initialState = new Start_State();
        stateMachine.changeState(Executive.StateMachine.StateType.DRIVE, initialState);
        stateMachine.init();
    }

    public void update() {
        stateMachine.update();
    }


    /**
     * Define Concrete State Classes
     */

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
