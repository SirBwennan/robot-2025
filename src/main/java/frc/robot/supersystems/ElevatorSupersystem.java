package frc.robot.supersystems;

import static frc.robot.Constants.ElevatorSupersystemConstants.beamBreakSensorDIO;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.Preset;
import frc.robot.subsystems.CoralArmGripper;
import frc.robot.subsystems.CoralArmPivot;
import frc.robot.subsystems.Elevator;

public class ElevatorSupersystem {
    private static ElevatorSupersystem instance;

    private static final Elevator elevator = Elevator.getInstance();
    private static final CoralArmGripper coral_arm_gripper = CoralArmGripper.getInstance();
    private static final CoralArmPivot coral_arm_pivot = CoralArmPivot.getInstance();

    private double cur_elevator_height = 0.0;
    private double cur_arm_angle = 0.0;
    private double cur_gripper_voltage = 0.0;

    private static final DigitalInput beam_break_sensor = new DigitalInput(beamBreakSensorDIO);
    private static final Trigger beam_broken = new Trigger(() -> beam_break_sensor.get());

    private ElevatorSupersystem() {
    }

    public Command setState(double elevator_height, double arm_angle, double gripper_voltage) {
        cur_elevator_height = elevator_height;
        cur_arm_angle = arm_angle;
        cur_gripper_voltage = gripper_voltage;
        return Commands.parallel(
                elevator.setHeight(elevator_height),
                coral_arm_pivot.setAngle(arm_angle),
                coral_arm_gripper.setGripperVoltage(gripper_voltage));
    }

    public Command setState(double elevator_height, double arm_angle) {
        cur_elevator_height = elevator_height;
        cur_arm_angle = arm_angle;
        return setState(elevator_height, arm_angle, cur_gripper_voltage);
    }

    public Command setStateElevator(double elevator_height) {
        cur_elevator_height = elevator_height;
        return setState(elevator_height, cur_arm_angle, cur_gripper_voltage);
    }

    public Command setStateGripper(double gripper_voltage) {
        cur_gripper_voltage = gripper_voltage;
        return setState(cur_elevator_height, cur_arm_angle, gripper_voltage);
    }

    public Command setStatePivot(double arm_angle) {
        cur_arm_angle = arm_angle;
        return setState(cur_elevator_height, arm_angle, cur_gripper_voltage);
    }

    public Command setStatePreset(Preset preset) {
        cur_elevator_height = preset.getHeight();
        cur_arm_angle = preset.getAngle();
        return setState(preset.getHeight(), preset.getAngle(), cur_gripper_voltage);
    }

    public Command setStatePreset(Preset preset, double gripper_voltage) {
        cur_elevator_height = preset.getHeight();
        cur_arm_angle = preset.getAngle();
        cur_gripper_voltage = gripper_voltage;
        return setState(preset.getHeight(), preset.getAngle(), gripper_voltage);
    }

    public Command setStateFromDashboard() {
        return Commands.parallel(
                elevator.setHeightFromDashboard(),
                coral_arm_pivot.setAngleFromDashboard(),
                coral_arm_gripper.setVoltageFromDashboard());
    }

    public static synchronized ElevatorSupersystem getInstance() {
        if (instance == null) {
            instance = new ElevatorSupersystem();
        }

        return instance;
    }

    public Trigger hasCoral() {
        return beam_broken;
    }

    // Intake
    public Command intakeSetupIntake() {
        return setStatePivot(0.15)
                .until(coral_arm_pivot.isGreaterThan(0))
                .andThen(setStateElevator(Preset.IntakeCatch.getHeight()))
                .until(elevator.isAtHeight(Preset.IntakeCatch.getHeight()))
                .andThen(setStatePivot(Preset.IntakeGrip.getAngle()));
    }

    public Command intakeLoadIntake() {
        return setStatePreset(Preset.IntakeGrip, 2)
                .until(beam_broken)
                .withTimeout(3)
                .andThen(setStatePreset(Preset.IntakeCatch, 1))
                .onlyIf(elevator.isGreaterThanHeight(Preset.IntakeCatch.getHeight()));
    }

    public Command testTriggers() {
        return Commands.print("elevator above").onlyIf(elevator.isGreaterThanHeight(Preset.IntakeCatch.getHeight()));
    }

    public Command intakePostIntake() {
        return setState(Preset.IntakeCatch.getHeight(),Preset.ScoreL4.getAngle())
                .until(elevator.isAtHeight(Preset.IntakeCatch.getHeight()).and(coral_arm_pivot.isGreaterThan(0.1)))
                .andThen(setState(
                        Preset.ScoreL1.getHeight(),
                        Preset.ScoreL4.getAngle()));
    }

    public Command storagePosition() {
        return intakePostIntake();
    }

    // Score Coral
    public static enum CoralLayer {
        L1, L2, L3, L4;

        public Preset toPreset() {
            return switch (this) {
                case L1 -> Preset.ScoreL1;
                case L2 -> Preset.ScoreL2;
                case L3 -> Preset.ScoreL3;
                case L4 -> Preset.ScoreL4;
            };
        }
    };

    public Command coralPrepareElevator(CoralLayer selected_layer) {
        return setStateElevator(selected_layer.toPreset().getHeight());
    }

    public Command coralPrepareArm(CoralLayer selected_layer) {
        return coral_arm_pivot.setAngle(selected_layer.toPreset().getAngle());
    }

    public Command coralScoreCoral(CoralLayer selected_layer) {
        if (selected_layer == CoralLayer.L1) {
            // it's at 90deg, driver drives forward while we spin gripper motors negative
            return setStateGripper(-3);
        } else if (selected_layer == CoralLayer.L2) {
            // it's at 60deg. needs to rotate down, then driver drives away
            return setState(
                    Preset.ScoreL2.getHeight(),
                    Preset.ScoreL2.getAngle() - 0.0277,
                    0);
        } else if (selected_layer == CoralLayer.L3) {
            return setState(
                    Preset.ScoreL3.getHeight(),
                    Preset.ScoreL4.getAngle() - 0.0277,
                    0);
        } else if (selected_layer == CoralLayer.L4) {
            return setState(
                    Preset.ScoreL4.getHeight(),
                    0,
                    0);
        } else {
            return Commands.none();
        }
    }

    // Extract Algae
    public static enum AlgaeExtractionLayer {
        High, Low;

        private Preset toPreset() {
            return switch (this) {
                case High -> Preset.ExtractAlgaeLow;
                case Low -> Preset.ExtractAlgaeHigh;
            };
        }
    };

    public Command algaeExtractionPrepareElevator(AlgaeExtractionLayer selected_layer) {
        return setStateElevator(selected_layer.toPreset().getHeight());
    }

    public Command algaeExtractionPrepareArm() {
        return setStatePivot(Preset.ExtractAlgaeLow.getAngle());
    }

    public Command algaeExtractionExtractAlgae() {
        return setStateGripper(-3);
    }
}
