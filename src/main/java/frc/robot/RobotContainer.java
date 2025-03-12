// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.photonvision.EstimatedRobotPose;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import choreo.auto.AutoChooser;
import choreo.auto.AutoFactory;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants.Preset;
import frc.robot.control.Launchpad;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.CoralArmGripper;
import frc.robot.subsystems.CoralArmPivot;
import frc.robot.subsystems.Elevator;
import frc.robot.supersystems.ElevatorSupersystem;
import frc.robot.supersystems.ElevatorSupersystem.CoralLayer;

public class RobotContainer {
    // initialize subsystems
    Elevator elevator = Elevator.getInstance();
    ElevatorSupersystem supersystem = ElevatorSupersystem.getInstance();
    CoralArmPivot coral_arm_pivot = CoralArmPivot.getInstance();
    CoralArmGripper coral_arm_gripper = CoralArmGripper.getInstance();
    Climber climber = Climber.getInstance();

    Launchpad launchpad = new Launchpad(1, 2, 3, new Color8Bit(255, 255, 255));
    Vision vision = Vision.getInstance();

    private double max_speed = TunerConstants.kSpeedAt12Volts.magnitude(); // kSpeedAt12Volts desired top speed
    private double max_robot_centric_speed = TunerConstants.kSpeedAt12Volts.magnitude() * 0.2; // kSpeedAt12Volts
                                                                                               // desired top speed
    private double max_angular_rate = 1.5 * Math.PI; // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(max_speed * 0.1)
            .withRotationalDeadband(max_angular_rate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors

    private final SwerveRequest.RobotCentric robot_centric = new SwerveRequest.RobotCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(max_speed);
    private final CommandXboxController controller = new CommandXboxController(0);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    // Choreo stuff
    private final AutoFactory auto_factory;
    private final AutoRoutines auto_routines;
    private final AutoChooser auto_chooser = new AutoChooser();

    public RobotContainer() {
        // Add Autos
        auto_factory = drivetrain.createAutoFactory()
                .bind("set_l4_height", supersystem.coralPrepareElevator(CoralLayer.L4))
                .bind("score_l4", supersystem.coralScoreCoral(CoralLayer.L4))
                .bind("prepare_accept_coral", supersystem.intakeSetupIntake())
                .bind("accept_coral", supersystem.intakeLoadIntake());

        auto_routines = new AutoRoutines(auto_factory);
        auto_chooser.addRoutine("Center Cage 2L4", auto_routines::CenterCage2l4);

        SmartDashboard.putData("Auto Chooser", auto_chooser);

        configureBindings();
    }

    private boolean manual_turtle_mode = false;
    private double turtle_mode = 1.0;

    public void updateTurtleMode() {
        turtle_mode = ((elevator.getHeight() >= Preset.ScoreL3.getHeight()) || manual_turtle_mode) ? 0.357 : 1;
    }

    private Trigger turtle_trigger = new Trigger(
            () -> ((elevator.getHeight() >= Preset.ScoreL3.getHeight()) || manual_turtle_mode));

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.

        // Drivetrain will execute this command periodically
        drivetrain.setDefaultCommand(
                drivetrain.applyRequest(
                        () -> drive.withVelocityX(-controller.getLeftY() * max_speed) // Drive forward with negative Y
                                                                                      // (forward)
                                .withVelocityY(-controller.getLeftX() * max_speed) // Drive left with negative X (left)
                                .withRotationalRate(-controller.getRightX() * max_angular_rate) // Drive
                                                                                                // counterclockwise with
                                                                                                // negative X (left)
                ));

        turtle_trigger.whileTrue(drivetrain.applyRequest(
                () -> drive.withVelocityX(-controller.getLeftY() * max_speed * turtle_mode) // Drive forward with
                                                                                            // negative Y (forward)
                        .withVelocityY(-controller.getLeftX() * max_speed * turtle_mode) // Drive left with negative X
                                                                                         // (left)
                        .withRotationalRate(-controller.getRightX() * max_angular_rate * turtle_mode) // Drive
                                                                                                      // counterclockwise
                                                                                                      // with negative X
                                                                                                      // (left)
        ));

        controller.a().whileTrue(drivetrain.applyRequest(() -> brake));
        controller.b().whileTrue(drivetrain.applyRequest(
                () -> point.withModuleDirection(new Rotation2d(-controller.getLeftY(), -controller.getLeftX()))));

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        controller.back().and(controller.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        controller.back().and(controller.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        controller.start().and(controller.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        controller.start().and(controller.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // reset the field-centric heading on left bumper press
        controller.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));
        
        controller.povUp().whileTrue(
                drivetrain.applyRequest(() -> robot_centric.withVelocityX(0).withVelocityY(max_robot_centric_speed)));
        controller.povDown().whileTrue(
                drivetrain.applyRequest(() -> robot_centric.withVelocityX(0).withVelocityY(-max_robot_centric_speed)));
        controller.povLeft().whileTrue(
                drivetrain.applyRequest(() -> robot_centric.withVelocityX(max_robot_centric_speed).withVelocityY(0)));
        controller.povRight().whileTrue(
                drivetrain.applyRequest(() -> robot_centric.withVelocityX(max_robot_centric_speed).withVelocityY(0)));

        // Elevator/coral arm controls
        launchpad.getButton(8, 1).onTrue(supersystem.coralPrepareElevator(CoralLayer.L4));
        launchpad.getButton(8, 2).onTrue(supersystem.coralPrepareElevator(CoralLayer.L3));
        launchpad.getButton(8, 3).onTrue(supersystem.coralPrepareElevator(CoralLayer.L2));
        launchpad.getButton(8, 4).onTrue(supersystem.coralPrepareElevator(CoralLayer.L1));

        launchpad.getButton(7, 1).onTrue(supersystem.coralPrepareArm(CoralLayer.L4));
        launchpad.getButton(7, 2).onTrue(supersystem.coralPrepareArm(CoralLayer.L3));
        launchpad.getButton(7, 3).onTrue(supersystem.coralPrepareArm(CoralLayer.L2));
        launchpad.getButton(7, 4).onTrue(supersystem.coralPrepareArm(CoralLayer.L1));

        launchpad.getButton(6, 1).onTrue(supersystem.coralScoreCoral(CoralLayer.L4));
        launchpad.getButton(6, 2).onTrue(supersystem.coralScoreCoral(CoralLayer.L3));
        launchpad.getButton(6, 3).onTrue(supersystem.coralScoreCoral(CoralLayer.L2));
        launchpad.getButton(6, 4).onTrue(supersystem.coralScoreCoral(CoralLayer.L1));

        /// manual testing voltage sets
        controller.leftBumper().onTrue(climber.setVoltage(6)).onFalse(climber.setVoltage(0));
        controller.leftTrigger().onTrue(climber.setVoltage(-6)).onFalse(climber.setVoltage(0));

        // Telemetrize our drive train
        drivetrain.registerTelemetry(logger::telemeterize);

        // Vision commands
        Commands.run(() -> {
            for (EstimatedRobotPose estimated_pose : vision.getCameraPoseEstimations()) {
                drivetrain.addVisionMeasurement(estimated_pose.estimatedPose.toPose2d(),
                        estimated_pose.timestampSeconds);

            }
        });
    }

    public Command getAutonomousCommand() {
        return auto_chooser.selectedCommand();
    }
}
