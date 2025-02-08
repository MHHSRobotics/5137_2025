package frc.robot.constants;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.ArmSystem;

import java.util.HashMap;
import java.util.Map;

/**
 * Constants for coordinated positions of the arm, elevator, and wrist mechanisms.
 * This class centralizes all the goal positions for the entire arm system.
 */
public final class ArmSystemConstants {
    private static final double wristStraight = Units.degreesToRadians(90);
    private static final double wristDown = Units.degreesToRadians(0);

    public static final double armSystemTimeout = 5; // seconds

    // Map of named states
    public static final Map<String, ArmSystem.ArmSystemState> states = new HashMap<>();
    
    

    static {
        // Define all states
        ArmSystem.ArmSystemState groundIntakeState = new ArmSystem.ArmSystemState(
            Units.degreesToRadians(0),  // From ArmConstants.groundIntakeGoal
            0.26,                         // From ElevatorConstants.groundIntakeGoal
            wristDown     // From WristConstants.pos1 (down)
        );


        ArmSystem.ArmSystemState defaultState = new ArmSystem.ArmSystemState(
            Units.degreesToRadians(90),   // From ArmConstants.defaultGoal
            0.0,                          // From ElevatorConstants.defaultGoal
            wristDown     // From WristConstants.pos1 (down)
        );


        ArmSystem.ArmSystemState sourceState = new ArmSystem.ArmSystemState(
            Units.degreesToRadians(55),  // From ArmConstants.sourceGoal
            0.76,                         // From ElevatorConstants.sourceGoal
            wristStraight    // From WristConstants.pos2 (straight)
        );


        ArmSystem.ArmSystemState algaeLowState = new ArmSystem.ArmSystemState(
            Units.degreesToRadians(120),   // From ArmConstants.algaeGoal
            0.35,                         // From ElevatorConstants.algaeGoal
            wristStraight    // From WristConstants.pos2 (straight)
        );

        ArmSystem.ArmSystemState algaeHighState = new ArmSystem.ArmSystemState(
            Units.degreesToRadians(105),   // From ArmConstants.algaeGoal
            0.35,                         // From ElevatorConstants.algaeGoal
            wristStraight    // From WristConstants.pos2 (straight)
        );


        ArmSystem.ArmSystemState l1State = new ArmSystem.ArmSystemState(
            Units.degreesToRadians(135),   // From ArmConstants.goals[0]
            0.06,                        // From ElevatorConstants.goals[0]
            wristStraight   // Scoring wrist position (straight)
        );


        ArmSystem.ArmSystemState l2State = new ArmSystem.ArmSystemState(
            Units.degreesToRadians(135),   // From ArmConstants.goals[1]
            0.26,                        // From ElevatorConstants.goals[1]
            wristStraight   // Scoring wrist position (straight)
        );


        ArmSystem.ArmSystemState l3State = new ArmSystem.ArmSystemState(
            Units.degreesToRadians(135),   // From ArmConstants.goals[2]
            0.56,                        // From ElevatorConstants.goals[2]
            wristStraight   // Scoring wrist position (straight)
        );


        ArmSystem.ArmSystemState l4State = new ArmSystem.ArmSystemState(
            Units.degreesToRadians(120),   // From ArmConstants.goals[3]
            1.26,                        // From ElevatorConstants.goals[3]
            wristStraight   // Scoring wrist position (straight)
        );

        ArmSystem.ArmSystemState processor = new ArmSystem.ArmSystemState(
            Units.degreesToRadians(15),
            1.26,
            wristStraight
        );


        // Populate the states map
        states.put("groundIntake", groundIntakeState);
        states.put("default", defaultState);
        states.put("source", sourceState);
        states.put("algaeLow", algaeLowState);        
        states.put("algaeHigh", algaeHighState);
        states.put("L1", l1State);
        states.put("L2", l2State);
        states.put("L3", l3State);
        states.put("L4", l4State);
        states.put("processor", processor);
    }


    // Default state for subsystem initialization
    public static final ArmSystem.ArmSystemState defaultState = states.get("default");

    // Default state for subsystem initialization
    public static final Translation3d armTransOffset=new Translation3d(0.11,-0.18,0.26);
} 