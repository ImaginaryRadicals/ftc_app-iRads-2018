package org.firstinspires.ftc.teamcode.Utilities;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.RobotHardware;

import java.util.ArrayList;

/**
 *  Timing Monitor is a debug tool for discovering the source of
 *  slowdowns in code.
 */
public class TimingMonitor {
    private RobotHardware opMode; // Reference to opmode
    private ElapsedTime timer;
    private boolean initialized;
    // All checkpoints hit once, names established and arrays populated.
    private boolean firstLoopCompleted;
    private ArrayList<String> checkpointNames;
    private ArrayList<Double> checkpointTimes;
    private int checkpointIndex;
    private int totalCheckpoints;
    private boolean checkpointOrderError;
    private final int MAX_CHECKPOINTS = 100;

    public TimingMonitor(RobotHardware opMode) {
        this.opMode = opMode;
        timer = new ElapsedTime();
        this.reset();
    }

    public void reset() {
        initialized = true; // What does this represent?
        firstLoopCompleted = false; // Required to begin timing
        checkpointIndex = 0;
        totalCheckpoints = 0;
        checkpointOrderError = false;
        checkpointNames = new ArrayList<>();
        checkpointTimes = new ArrayList<>();
    }

    public void loopStart() {
        double previousTime;
        double currentTime;
        // Determine if the first loop has been completed
        if (!firstLoopCompleted){ // Initial loop underway.
            if(checkpointIndex > 0) {
                // We've already completed the first loop, so get to work.
                firstLoopCompleted = true;
                totalCheckpoints = checkpointNames.size();
            } else {
                // This is the first time the loop start has been run.
                //checkpointIndex <= 0
                checkpointIndex = 0;
                checkpointNames.add("START");
                currentTime = timer.seconds(); // Read and trash time for consistency.
                timer.reset();
                checkpointTimes.add(0.0); // Initialize blank on first loop.

            }
        }

        if (firstLoopCompleted){
            // Loop initialized and surveyed once.
            // Begin normal operation.
            // Looking for longest delay
            checkpointIndex = 0; // Only place the checkpoint index is reset.
            currentTime = timer.seconds();
            timer.reset();
            previousTime = checkpointTimes.get(checkpointIndex);
            if(currentTime > previousTime) {
                // Store new time if it is longer.
                checkpointTimes.set(checkpointIndex,currentTime);
            }
        }
        // Increment the checkpoint index
        checkpointIndex++;
    }

    public void checkpoint(String checkpointName){
        double currentTime;
        double previousTime;
        if(checkpointIndex > 0 && checkpointIndex <= MAX_CHECKPOINTS) {
            // If checkpointIndex wasn't incremented >0, then we haven't run loopStart().
            // If loopStart() is NEVER called, then checkpoint will cause the arrays to grow
            // without bound, so a limit was added
            if (firstLoopCompleted) {
                currentTime = timer.seconds();
                timer.reset();
                previousTime = checkpointTimes.get(checkpointIndex);
                if (currentTime > previousTime) {
                    // Store new time if it is longer.
                    checkpointTimes.set(checkpointIndex, currentTime);
                }

                //Error check the name index
                if (checkpointName != checkpointNames.get(checkpointIndex)) {
                    // If we miss a checkpoint, our times will be out of order and invalid.
                    checkpointOrderError = true;
                }


            } else {
                // firstLoopCompleted = false
                // Add checkpoint name to list
                checkpointNames.add(checkpointName);
                // Add checkpoint time to the list.
                currentTime = timer.seconds(); // Read and trash time for consistency.
                timer.reset();
                checkpointTimes.add(0.0); // Initialize blank
            }
            // Increment the checkpoint index
            checkpointIndex++;
        } else { // If checkpointIndex wasn't incremented, then we haven't run loopStart().
            checkpointOrderError = true;
        }
    }

    public void displayMaxTimes() {
        opMode.telemetry.addData("TimingMonitor","Max Time Results:");
        if(checkpointOrderError) {
            opMode.telemetry.addData("ERROR", "checkpoint order inconsistent.");
            opMode.telemetry.addData("ERROR", "Make sure loopStart() runs before checkpoint(), and that no calls are conditional.");
        }
        if(firstLoopCompleted) {
            int index = 0;
            for (String checkpoint : checkpointNames) {
                opMode.telemetry.addData(checkpoint, opMode.df_prec.format(checkpointTimes.get(index)));
                index++;
            }
        }
    }
}
