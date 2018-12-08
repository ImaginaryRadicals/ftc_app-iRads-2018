package org.firstinspires.ftc.teamcode.Utilities;

import org.firstinspires.ftc.teamcode.AutoOpmode;

import java.util.ArrayList;

public class Path {
    private ArrayList<MecanumNavigation.Navigation2D> waypoints = new ArrayList<>();
    private ArrayList<RobotStateMachine.AutoState> states = new ArrayList<>();
    private int index = 0;
    private AutoDrive autoDrive;

    Path(AutoDrive autoDrive)
    {
        this.autoDrive = autoDrive;
    }

    public void addWaypoint(RobotStateMachine.AutoState state, MecanumNavigation.Navigation2D waypoint) {
        states.add(state);
        waypoints.add(waypoint);
    }

    public RobotStateMachine.AutoState getState()
    {
        return states.get(index);
    }

    public boolean run(double speed)
    {
        boolean arrived = autoDrive.rotateThenDriveToPosition(waypoints.get(index), speed);

        if (arrived) {
            ++index;
        }

        if (index >= waypoints.size())
        {
            return true;
        }

        return false;
    }

    private double degreesToRadians(double degrees) {
        return degrees * Math.PI / 180;
    }
}
