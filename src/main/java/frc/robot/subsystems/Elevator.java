package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Millimeters;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Rotations;
import static frc.robot.Constants.ElevatorConstants.*;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase {
    private static Elevator instance;

    private static TalonFX leftMotor;
    private static TalonFX rightMotor;

    private static final PositionVoltage position_voltage = new PositionVoltage(0);

    private static CANrange heightSensor;

    private Elevator() {
        rightMotor = new TalonFX(rightMotorID, "canivore");
        leftMotor = new TalonFX(leftMotorID, "canivore");
        heightSensor = new CANrange(heightSensorID, "canivore");

        configureMotors();
    }

    public static synchronized Elevator getInstance() {
        if (instance == null) {
            instance = new Elevator();
        }

        return instance;
    }

    private void configureMotors() {
        var cfg = new TalonFXConfiguration();
        cfg.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        // feedback sensor
        cfg.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        cfg.Feedback.SensorToMechanismRatio = motorGearRatio;

        // MotionMagic
        cfg.MotionMagic.MotionMagicAcceleration = motorMaxAcceleration;
        cfg.MotionMagic.MotionMagicAcceleration = motorCruiseVelocity;

        // PID + motionmagic constants
        cfg.Slot0.GravityType = GravityTypeValue.Elevator_Static;
        cfg.Slot0.kA = 0.01;
        cfg.Slot0.kG = 0.067;
        cfg.Slot0.kP = 60;
        cfg.Slot0.kI = 0;
        cfg.Slot0.kD = 0.5;

        rightMotor.getConfigurator().apply(cfg);
        leftMotor.getConfigurator().apply(cfg);
        rightMotor.setControl(leftMotor.getAppliedControl());
    }

    // Commands to nudge elevator (doesn't rely on setHeight)
    public Command startNudgeElevator(Voltage volts) {
        return run(() -> {
            rightMotor.setVoltage(volts.magnitude());
            leftMotor.setVoltage(volts.magnitude());
        });
    }

    public Command stopNudgeElevator() {
        return runOnce(() -> {
            rightMotor.setVoltage(0);
            leftMotor.setVoltage(0);
        });
    }

    private Distance getHeight() {
        return heightSensor.getDistance().getValue().minus(canrangeOffset);
    }

    // setHeight-related methods
    private boolean isAtHeight(Distance goalHeight) {
        return getHeight().isNear(goalHeight, heightTolerance);
    }

    // TODO: measure elevator height presets
    public enum ElevatorHeightPreset {
        Initial(Inches.of(0)), // not zero!!!
        Intake(Inches.of(0)),
        L1(Inches.of(0)),
        L2(Inches.of(0)),
        L3(Inches.of(0)),
        L4(Inches.of(0));

        private final Distance distance;

        ElevatorHeightPreset(Distance d) {
            distance = d;
        }

        public Distance getHeight() {
            return distance;
        }
    }

    private static Angle translateHeightToRotations(Distance goalHeight) {
        return Radians.of(goalHeight.in(Inches) / (2 * Math.PI * sprocketRadius.in(Inches)));
    }

    public Command setHeight(Distance goalHeight) {
        Angle goalRotations = translateHeightToRotations(goalHeight);
        position_voltage.Position = goalRotations.in(Rotations);

        return runOnce(() -> {
            leftMotor.setControl(position_voltage);
            rightMotor.setControl(position_voltage);
        })
            .until(() -> isAtHeight(goalHeight))
            .andThen(() -> {
                leftMotor.setVoltage(0);
                rightMotor.setVoltage(0);
            });
    }

    public Command setHeightFromPreset(ElevatorHeightPreset preset) {
        return setHeight(preset.getHeight());
    }

    @Override
    public void periodic() {
        if (leftMotor.getPosition().getValue().isNear(translateHeightToRotations(getHeight()), angleTolerance)) {
            leftMotor.setPosition(translateHeightToRotations(getHeight()));
        }

        SmartDashboard.putString("CANrange Reading", String.format("%.9f", heightSensor.getDistance().getValue().in(Millimeters)));
    }
}
