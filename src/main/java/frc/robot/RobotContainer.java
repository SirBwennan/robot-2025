// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.Supplier;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import choreo.auto.AutoChooser;
import choreo.auto.AutoFactory;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.commands.DriveToPose;
import frc.robot.control.Launchpad;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.CoralArmGripper;
import frc.robot.subsystems.CoralArmPivot;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.StatusLED;
import frc.robot.supersystems.ElevatorSupersystem;
import frc.robot.util.AllianceFlipUtil;

public class RobotContainer {
        // initialize subsystems
        Elevator elevator = Elevator.getInstance();
        ElevatorSupersystem supersystem = ElevatorSupersystem.getInstance();
        CoralArmPivot coral_arm_pivot = CoralArmPivot.getInstance();
        CoralArmGripper coral_arm_gripper = CoralArmGripper.getInstance();
        StatusLED status_led = StatusLED.getInstance();
        // Climber climber = Climber.getInstance();

        Launchpad launchpad = new Launchpad(1, 2, 3, new Color8Bit(255, 255, 255));

        private double max_speed = TunerConstants.kSpeedAt12Volts.magnitude(); // kSpeedAt12Volts desired top speed
                                                                               // desired top speed
        private double max_angular_rate = 1.5 * Math.PI; // 3/4 of a rotation per second max angular velocity

        /* Setting up bindings for necessary control of the swerve drive platform */
        private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
                        .withDeadband(max_speed * 0.1)
                        .withRotationalDeadband(max_angular_rate * 0.1) // Add a 10% deadband
                        .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use closed-loop control for drive motors

        private final SwerveRequest.RobotCentric robot_centric = new SwerveRequest.RobotCentric()
                        .withDriveRequestType(DriveRequestType.Velocity);

        private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
        private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

        private final Telemetry logger = new Telemetry(max_speed);
        private final CommandXboxController controller = new CommandXboxController(0);

        public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

        // Choreo stuff
        private final AutoFactory auto_factory;
        private final AutoRoutines auto_routines;
        private final AutoChooser auto_chooser = new AutoChooser();
        
        private final SlewRateLimiter slew_rate_limiter_x = new SlewRateLimiter(1);
        private final SlewRateLimiter slew_rate_limiter_y = new SlewRateLimiter(1);

        private double getControlX() {
                return slew_rate_limiter_x.calculate(-controller.getLeftX());
        }

        private double getControlY() {
                return slew_rate_limiter_y.calculate(-controller.getLeftY());
        }

        public RobotContainer() {
                if (Robot.isSimulation()) {
                        DriverStation.silenceJoystickConnectionWarning(true);
                }
                // Add Autos
                auto_factory = drivetrain.createAutoFactory();

                auto_routines = new AutoRoutines(auto_factory);
                auto_chooser.addRoutine("Center 1L4", auto_routines::Center1l4);
                auto_chooser.addRoutine("Drive Forward", auto_routines::DriveForward);

                SmartDashboard.putData("Auto Chooser", auto_chooser);

                configureBindings();
        }

        private final double turtle_mode = 0.15;
        private Trigger turtle_trigger = new Trigger(() -> (elevator.getHeight() >= Elevator.ElevatorHeight.scoreL2));

        private final double slower_turtle_mode = 0.035;

        // Auto align bindings
        private Supplier<Pose2d> nearestLeftCoral() {
                return () -> {
                        return drivetrain.getState().Pose.nearest(
                                        FieldConstants.Reef.lefts.stream().map(AllianceFlipUtil::apply).toList());
                };
        }

        private Supplier<Pose2d> nearestRightCoral() {
                return () -> {
                        return drivetrain.getState().Pose.nearest(
                                        FieldConstants.Reef.rights.stream().map(AllianceFlipUtil::apply).toList());
                };

        }

        public void configureBindings() {
                // Note that X is defined as forward according to WPILib convention,
                // and Y is defined as to the left according to WPILib convention.

                // Drivetrain will execute this command periodically
                drivetrain.setDefaultCommand(
                                drivetrain.applyRequest(
                                                () -> drive.withVelocityX(getControlY() * max_speed) // Drive
                                                                                                              // forward
                                                                                                              // with
                                                                                                              // negative
                                                                                                              // Y
                                                                                                              // (forward)
                                                                .withVelocityY(getControlX() * max_speed) // Drive
                                                                                                                   // left
                                                                                                                   // with
                                                                                                                   // negative
                                                                                                                   // X
                                                                                                                   // (left)
                                                                                                                   // Drive
                                                                                                                   // counterclockwise
                                                                                                                   // with
                                                                                                                   // negative
                                                                                                                   // X
                                                                                                                   // (left)
                                                                .withRotationalRate(-controller.getRightX()
                                                                                * max_angular_rate) // Drive
                                                                                                    // counterclockwise
                                                                                                    // with
                                                                                                    // negative X (left)

                                ));

                // TODO: turtle trigger was a problem in auto so we commented it out, will fix it later
                turtle_trigger
                        .and(DriverStation::isTeleopEnabled)
                        .whileTrue(drivetrain.applyRequest(
                                () -> drive.withVelocityX(getControlY() * max_speed * turtle_mode) // Drive
                                                                                                            // forward
                                                                                                            // with
                                                                                                            // negative
                                                                                                            // Y
                                                                                                            // (forward)
                                                .withVelocityY(getControlY() * max_speed * turtle_mode) // Drive
                                                                                                                 // left
                                                                                                                 // with
                                                                                                                 // negative
                                                                                                                 // X
                                                                                                                 // (left)
                                                .withRotationalRate(-controller.getRightX() * max_angular_rate
                                                                * turtle_mode) // Drive
                                                                               // counterclockwise
                                                                               // with negative X
                                                                               // (left)
                ));

                controller.a().whileTrue(drivetrain.applyRequest(() -> brake));
                controller.b().whileTrue(drivetrain.applyRequest(
                                () -> point.withModuleDirection(
                                                new Rotation2d(-controller.getLeftY(), -controller.getLeftX()))));

                // Run SysId routines when holding back/start and X/Y.
                // Note that each routine should be run exactly once in a single log.
                controller.back().and(controller.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
                controller.back().and(controller.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
                controller.start().and(controller.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
                controller.start().and(controller.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

                // reset the field-centric heading on left bumper press
                controller.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

                controller.povLeft().whileTrue(
                                drivetrain.applyRequest(
                                                () -> robot_centric.withVelocityX(0)
                                                                .withVelocityY(max_speed * slower_turtle_mode)));
                controller.povRight().whileTrue(

                                drivetrain.applyRequest(
                                                () -> robot_centric.withVelocityX(0)
                                                                .withVelocityY(-max_speed * slower_turtle_mode)));
                controller.povUp().whileTrue(
                                drivetrain.applyRequest(
                                                () -> robot_centric.withVelocityX(max_speed * slower_turtle_mode)
                                                                .withVelocityY(0)));
                controller.povDown().whileTrue(
                                drivetrain.applyRequest(
                                                () -> robot_centric.withVelocityX(-max_speed * slower_turtle_mode)
                                                                .withVelocityY(0)));

                DriveToPose left_go_to_pose = new DriveToPose(drivetrain, nearestLeftCoral());
                DriveToPose right_go_to_pose = new DriveToPose(drivetrain, nearestRightCoral());

                (new Trigger(left_go_to_pose::atGoal)).or(right_go_to_pose::atGoal).whileTrue(status_led.flashColor(Color.kGreen, 1));

                // left/right reef align
                launchpad.getButton(2, 0).whileTrue(left_go_to_pose);
                launchpad.getButton(3, 0).whileTrue(right_go_to_pose);

                // Elevator/coral arm controls
                launchpad.getButton(8, 1).onTrue(supersystem.coralPrepareL1());
                launchpad.getButton(8, 2).onTrue(supersystem.coralPrepareL2());
                launchpad.getButton(8, 3).onTrue(supersystem.coralPrepareL3());
                launchpad.getButton(8, 4).onTrue(supersystem.coralPrepareL4());

                // coral scoring
                launchpad.getButton(7, 1).onTrue(supersystem.coralScoreL1());
                launchpad.getButton(7, 2).onTrue(supersystem.coralScoreL2());
                launchpad.getButton(7, 3).onTrue(supersystem.coralScoreL3());
                launchpad.getButton(7, 4).onTrue(supersystem.coralScoreL4());

                // intake
                launchpad.getButton(8, 5).onTrue(supersystem.intakePrepare());
                launchpad.getButton(7, 5).onTrue(supersystem.intakeLoad());
                launchpad.getButton(6, 5).onTrue(supersystem.intakePost());

                // high extract / low extract
                launchpad.getButton(8, 6).onTrue(supersystem.extractionPrepareHigh());
                launchpad.getButton(7, 6).onTrue(supersystem.extractionExtractHigh());

                // low extract
                launchpad.getButton(8, 7).onTrue(supersystem.extractionPrepareLow());
                launchpad.getButton(7, 7).onTrue(supersystem.extractionExtractLow());

                // TODO: barge scoring

                // processor scoring
                launchpad.getButton(6, 7).onTrue(supersystem.algaePrepareProcessor());
                launchpad.getButton(5, 7).onTrue(supersystem.algaeScoreProcessor());

                // barge scoring

                launchpad.getButton(6, 6).onTrue(supersystem.algaePrepareBarge());
                launchpad.getButton(5, 6).onTrue(supersystem.algaeScoreBarge());

                // storage positions

                launchpad.getButton(0, 7).onTrue(supersystem.storagePositionAlgae());
                launchpad.getButton(0, 8).onTrue(supersystem.storagePosition());

                // launchpad.getButton(2, 2).onTrue(supersystem.setStateFromDashboard());

                launchpad.getButton(8, 8).onTrue(supersystem.extractionStop());
                launchpad.getButton(7, 0).onTrue(Commands.runOnce(() -> ElevatorSupersystem.beam_break_override = true))
                                .onFalse(Commands.runOnce(() -> ElevatorSupersystem.beam_break_override = false));

                /// manual testing voltage sets
                // controller.rightTrigger().onTrue(climber.setVoltage(6)).onFalse(climber.setVoltage(0));
                // controller.leftTrigger().onTrue(climber.setVoltage(-6)).onFalse(climber.setVoltage(0));

                // Telemetrize our drive train
                drivetrain.registerTelemetry(logger::telemeterize);
        }

        // public Command getDriveToPose(Supplier<Pose2d> supplySide) {
        // DriveToPose leftSide = new DriveToPose(drivetrain, supplySide.get());

        // return leftSide;
        // }

        public Command getAutonomousCommand() {
                return auto_chooser.selectedCommand();
        }
}
