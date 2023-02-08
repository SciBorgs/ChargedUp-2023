package org.sciborgs1155.robot.subsystems;

import org.junit.jupiter.api.*;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj2.command.Command;

import static org.junit.jupiter.api.Assertions.*;

import org.sciborgs1155.robot.Constants;
import org.sciborgs1155.robot.Constants.Dimensions;

public class ElevatorTest {
    
    Elevator elevator = new Elevator();

    @BeforeEach
    void setup() {
        assert HAL.initialize(500, 0);
    }

    @Test
    void setGoal() {
        elevator.setGoal(9).ignoringDisable(true).schedule();
        assertEquals(9, elevator.getGoal());
        
        elevator.setGoal(Dimensions.ELEVATOR_MAX_HEIGHT + 5).ignoringDisable(true).schedule();
        assertEquals(Dimensions.ELEVATOR_MAX_HEIGHT, elevator.getGoal());

        elevator.setGoal(-3).ignoringDisable(true).schedule();
        assertEquals(Dimensions.ELEVATOR_MIN_HEIGHT, elevator.getGoal());
    }
}
