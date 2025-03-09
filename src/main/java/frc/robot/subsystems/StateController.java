package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.*;

// this is used for the the second xbox controller so we can manage what the robot will do on the next step and will
// return the aprorate command so the robot will excuted it

public class StateController extends SubsystemBase {
    
    public enum StateCategory {
        CORTAL_PUT,
        CORTAL_GET,
        BALL_GET,
        BALL_PUT
    }
    
    public enum RobotState {
        // CORTAL states
        CORTAL_GROUND(StateCategory.CORTAL_GET),    
        CORTAL_FEEDER(StateCategory.CORTAL_GET),    
        CORTAL_L1(StateCategory.CORTAL_PUT),        
        CORTAL_L2(StateCategory.CORTAL_PUT),        
        CORTAL_L3(StateCategory.CORTAL_PUT),        
        CORTAL_L4(StateCategory.CORTAL_PUT),        

        // BALL states
        BALL_L2_TO_L3(StateCategory.BALL_GET),    
        BALL_L3_TO_L4(StateCategory.BALL_GET),    
        BALL_PROCESSOR(StateCategory.BALL_PUT),   
        BALL_NET(StateCategory.BALL_PUT);         

        private final StateCategory category;
        
        RobotState(StateCategory category) {
            this.category = category;
        }
        
        public StateCategory getCategory() {
            return category;
        }
    }
    
    private static final Map<RobotState, RobotState> NEXT_LEVEL = new HashMap<>();
    private static final Map<RobotState, RobotState> PREV_LEVEL = new HashMap<>();
    
    static {
        // Initialize CORTAL level transitions
        NEXT_LEVEL.put(RobotState.CORTAL_L1, RobotState.CORTAL_L2);
        NEXT_LEVEL.put(RobotState.CORTAL_L2, RobotState.CORTAL_L3);
        NEXT_LEVEL.put(RobotState.CORTAL_L3, RobotState.CORTAL_L4);
        NEXT_LEVEL.put(RobotState.CORTAL_L4, RobotState.CORTAL_L1);
        
        PREV_LEVEL.put(RobotState.CORTAL_L1, RobotState.CORTAL_L4);
        PREV_LEVEL.put(RobotState.CORTAL_L2, RobotState.CORTAL_L1);
        PREV_LEVEL.put(RobotState.CORTAL_L3, RobotState.CORTAL_L2);
        PREV_LEVEL.put(RobotState.CORTAL_L4, RobotState.CORTAL_L3);
        
        // Initialize BALL level transitions
        NEXT_LEVEL.put(RobotState.BALL_L2_TO_L3, RobotState.BALL_L3_TO_L4);
        NEXT_LEVEL.put(RobotState.BALL_L3_TO_L4, RobotState.BALL_L2_TO_L3);
        
        PREV_LEVEL.put(RobotState.BALL_L2_TO_L3, RobotState.BALL_L3_TO_L4);
        PREV_LEVEL.put(RobotState.BALL_L3_TO_L4, RobotState.BALL_L2_TO_L3);
    }
    
    private RobotState currentState;
    private final Map<StateCategory, RobotState> savedStates;
    
    public StateController() {
        currentState = RobotState.CORTAL_L1;
        savedStates = new EnumMap<>(StateCategory.class);
        
        // Initialize default states for each category
        savedStates.put(StateCategory.CORTAL_PUT, RobotState.CORTAL_L1);
        savedStates.put(StateCategory.CORTAL_GET, RobotState.CORTAL_GROUND);
        savedStates.put(StateCategory.BALL_GET, RobotState.BALL_L2_TO_L3);
        savedStates.put(StateCategory.BALL_PUT, RobotState.BALL_PROCESSOR);
    }
    
    public void setState(RobotState newState) {
        if (currentState != null) {
            savedStates.put(currentState.getCategory(), currentState);
        }
        currentState = newState;
    }
    
    public RobotState getCurrentState() {
        return currentState;
    }
    
    public boolean isInState(RobotState state) {
        return currentState == state;
    }
    
    public boolean isCortalState() {
        return currentState.name().startsWith("CORTAL");
    }

    public boolean isBallState() {
        return currentState.name().startsWith("BALL");
    }

    public boolean isGet() {
        return currentState.getCategory() == StateCategory.CORTAL_GET 
            || currentState.getCategory() == StateCategory.BALL_GET;
    }

    public boolean isPut() {
        return currentState.getCategory() == StateCategory.CORTAL_PUT 
            || currentState.getCategory() == StateCategory.BALL_PUT;
    }

    public void incrementLevel() {
        RobotState nextState = NEXT_LEVEL.get(currentState);
        if (nextState != null) {
            setState(nextState);
        }
    }

    public void decrementLevel() {
        RobotState prevState = PREV_LEVEL.get(currentState);
        if (prevState != null) {
            setState(prevState);
        }
    }

    public void toggleGetPut() {
        if (isCortalState()) {
            StateCategory targetCategory = isGet() ? StateCategory.CORTAL_PUT : StateCategory.CORTAL_GET;
            setState(savedStates.get(targetCategory));
        } else if (isBallState()) {
            StateCategory targetCategory = isGet() ? StateCategory.BALL_PUT : StateCategory.BALL_GET;
            setState(savedStates.get(targetCategory));
        }
    }

    public void toggleBallCortal() {
        if (isCortalState()) {
            StateCategory targetCategory = isGet() ? StateCategory.BALL_GET : StateCategory.BALL_PUT;
            setState(savedStates.get(targetCategory));
        } else if (isBallState()) {
            StateCategory targetCategory = isGet() ? StateCategory.CORTAL_GET : StateCategory.CORTAL_PUT;
            setState(savedStates.get(targetCategory));
        }
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }
}
