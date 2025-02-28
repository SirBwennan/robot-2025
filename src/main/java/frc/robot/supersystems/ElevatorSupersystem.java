package frc.robot.supersystems;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.CoralArm;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.CoralArm.CoralArmPreset;
import frc.robot.subsystems.Elevator.ElevatorHeightPreset;

public class ElevatorSupersystem extends SubsystemBase {
    private static Elevator elevator;
    private static CoralArm coral_arm;

    public ElevatorSupersystem() {
        elevator = Elevator.getInstance();
        coral_arm = CoralArm.getInstance();
    }

    public void fullIntakeSequence() {
        elevator.setHeightFromPreset(ElevatorHeightPreset.Intake);
        coral_arm.setPivotAngleFromPreset(CoralArmPreset.Down);
        
        coral_arm.setGripperVoltage(Volts.of(40));
    }
}
