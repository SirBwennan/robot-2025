package frc.robot.subsystems;

import static frc.robot.Constants.ClimberConstants.*;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {
    private static Climber instance;

    private static ClimbState climber_state;
    private static SparkMax motor;

    private enum ClimbState {
        Initial,
        Phase1Active,
        Phase1Complete,
        Phase2Active,
        Phase2Complete,
        Phase3Active,
        ClimbComplete;
    };

    private Climber() {
        climber_state = ClimbState.Initial;

        motor = new SparkMax(climberMotorID, MotorType.kBrushless);
        configureMotors();
    }

    public static synchronized Climber getInstance() {
        if (instance == null) {
            instance = new Climber();
        }

        return instance;
    }

    private void configureMotors() {
        // do I need to reset motor here?
        var cfg = new SparkMaxConfig();
        cfg.idleMode(IdleMode.kBrake);
        cfg.smartCurrentLimit(climberMotorCurrentLimit);
        cfg.closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .p(0.5)
                .i(0)
                .d(0.05);

        motor.configure(cfg, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    private boolean isAtSetpoint(double rotations) {
        return Math.abs(motor.getEncoder().getPosition() - rotations) > climberMotorTolerance;
    }

    private Command addRotations(double rotations) {
        double currentPosition = motor.getEncoder().getPosition();
        double targetPosition = (currentPosition + rotations);
        return run(() -> motor.getClosedLoopController().setReference(targetPosition, ControlType.kPosition));
    }

    /**
     * Moves the climber arm into position to catch the cage
     * Runs in 0.6 sec
     */
    public Command deployIntake() {
        if (climber_state == ClimbState.Initial) {
            return run(() -> {
                climber_state = ClimbState.Phase1Active;
                addRotations(50).until(() -> isAtSetpoint(50));
                climber_state = ClimbState.Phase1Complete;
            });
        } else {
            return Commands.none();
        }
    }

    /**
     * Bring the cage into the center of the robot, not supporting the weight of the
     * robot yet.
     * Runs in 1.2 sec
     */
    public Command retrieveCage() {
        if (climber_state == ClimbState.Phase1Complete) {
            return run(() -> {
                climber_state = ClimbState.Phase2Active;
                addRotations(79).until(() -> isAtSetpoint(50+79));
                climber_state = ClimbState.Phase2Complete;
            });
        } else {
            return Commands.none();
        }
    }

    /**
     * Complete the climb by pivoting off the cage and supporting the robot from the
     * cage.
     * Runs in 3 sec
     */
    public Command finishClimb() {
        if (climber_state == ClimbState.Phase2Complete) {
            return run(() -> {
                climber_state = ClimbState.Phase3Active;
                addRotations(81).until(() -> isAtSetpoint(50+79+81));
                climber_state = ClimbState.ClimbComplete;
            });
        } else {
            return Commands.none();
        }
    }

    @Override
    public void periodic() {
        SmartDashboard.putString("Climber State", climber_state.toString());
    }

    // TODO: SmartDashboard camera feed
}
