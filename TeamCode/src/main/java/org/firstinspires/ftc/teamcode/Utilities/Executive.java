package org.firstinspires.ftc.teamcode.Utilities;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.Map;
import java.util.Set;

/**
 * Provides Executive robot Control through class based state machine.
 */
public class Executive {

    /**
     * Interface to be used as t he interface for the class which hosts the
     * state machine object as well as the concrete implementations of the required states.
     */
    public interface RobotStateMachineContextInterface {
        void init();
        void update();
    }


    /**
     * State Machine will support one additional concurrent state for each possible StateType.
     * Note that StateMachine.remove(StateType.ARM) would remove an existing ARM state, for
     * example.
     */
    public enum StateType {
        DRIVE,
        ARM,
        LIFT,
    }

    /**
     * Robot state machine, supporting multiple named simultaneous states, expanded by adding to an enum.
     */
    public class StateMachine {

        private Map<StateType, StateBase> stateMap = new HashMap<>();


        public void changeState(StateType stateType, StateBase state) {
            stateMap.put(stateType, state);
        }

        public void removeStateType(StateType stateType) {
            stateMap.remove(stateType);
        }


        public void clearDeletedStates() {
            for (StateType stateType : StateType.values()) {
                if (stateMap.get(stateType).isDeleteRequested()) {
                    stateMap.remove(stateType);
                }
            }
        }

        public void init() {
            Set<StateType> stateTypeSet = stateMap.keySet();
            StateType[] stateTypeKeyArray = stateTypeSet.toArray(new StateType[stateTypeSet.size()]);
            for (StateType type : stateTypeKeyArray) {
                 stateMap.get(type).init();
            }
        }

        /**
         * Check if state is initialized, and throw exception if not.
         * Then, if nextStates has values, StateMachine.add() them and clear new states.
         * Then, if state.isDeleteRequested(), StateMachine.delete(state)
         * otherwise, run state.update()
         */
        public void update() {
            Set<StateType> stateTypeSet = stateMap.keySet();
            StateType[] stateTypeKeyArray = stateTypeSet.toArray(new StateType[stateTypeSet.size()]);
            for (StateType type : stateTypeKeyArray) {
                StateBase state = stateMap.get(type);

                if (!state.isInitialized()) {
                    throw new RuntimeException("ERROR: state called in StateMachine update() was never initialized") ;
                }
                // Delete or update state
                if (state.isDeleteRequested() == false) {
                    state.update();
                }
            }
            clearDeletedStates();
        }


        public void reset() {
            Set<StateType> stateTypeSet = stateMap.keySet();
            StateType[] stateTypeKeyArray = stateTypeSet.toArray(new StateType[stateTypeSet.size()]);
            for (StateType type : stateTypeKeyArray) {
                StateBase state = stateMap.get(type);
                state.reset();
            }
        }


    }



    // The class is abstract, but the internal methods are not abstract so that they can be optionally implemented
    public abstract class StateBase {

        StateMachine stateMachine;
        private boolean initialized = false;
        private boolean deleteRequested = false;

        public StateBase(StateMachine stateMachine) {
            this.stateMachine = stateMachine;
        }

        public void init() {
            initialized = true;
        }

        public void update() {

        }

        public void reset() {
            initialized = false;
        }

        boolean isInitialized() {
            return initialized;
        }

        public void requestDelete() {
            deleteRequested = true;
        }

        public boolean isDeleteRequested() {
            return deleteRequested;
        }



    }


}
