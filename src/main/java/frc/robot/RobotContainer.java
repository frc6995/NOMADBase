// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import java.lang.Thread.State;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.therekrab.autopilot.APTarget;
import com.ctre.phoenix6.hardware.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import choreo.auto.AutoChooser;
import choreo.auto.AutoFactory;
import frc.robot.Autos;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.ArmS;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.ElevatorS;
import frc.robot.subsystems.HandS;

import frc.robot.subsystems.HandS.HandConstants;
import frc.robot.subsystems.YAMSIntakePivot;
import frc.robot.subsystems.YAMSIntakeRollerS;
import frc.robot.StateMachine;

public class RobotContainer {
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second
                                                                                      // max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    public static final CommandXboxController joystick = new CommandXboxController(0);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    // public final IntakePivotS intakePivot = new IntakePivotS();

    public final YAMSIntakeRollerS intakeRoller = new YAMSIntakeRollerS();

    public final HandS handRoller = new HandS();

    public final ArmS arm = new ArmS();

    public final ElevatorS elevator = new ElevatorS();

    public final YAMSIntakePivot yIntakePivot = new YAMSIntakePivot();

    private final AutoFactory autoFactory;
    private Mechanism2d VISUALIZER;
    private final Autos autoRoutines;
    private final AutoChooser m_chooser = new AutoChooser();
    private final StateMachine stateMachine = new StateMachine();

    public RobotContainer() {

        drivetrain.resetOdometry(new Pose2d());
        VISUALIZER = logger.MECH_VISUALIZER;

        configureBindings();
        SmartDashboard.putData("Visualzer", VISUALIZER);

        autoFactory = drivetrain.createAutoFactory();
        autoRoutines = new Autos(drivetrain, arm, yIntakePivot, intakeRoller, elevator, null, autoFactory);
        m_chooser.addRoutine("FourCoralRight", autoRoutines::FourCoralRight);
        m_chooser.addRoutine("FourCoralLeft", autoRoutines::FourCoralLeft);
        m_chooser.addRoutine("BacksideAuto", autoRoutines::BacksideAuto);
        SmartDashboard.putData("Auto Mode", m_chooser);

    }

    public double xButtonPressedTime = 0;

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
                // Drivetrain will execute this command periodically
                drivetrain.applyRequest(() -> drive.withVelocityX(-joystick.getLeftY() * MaxSpeed) // Drive forward with
                                                                                                   // negative Y
                                                                                                   // (forward)
                        .withVelocityY(-joystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                        .withRotationalRate(-joystick.getRightX() * MaxAngularRate) // Drive counterclockwise with
                                                                                    // negative X (left)
                ));

        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
                drivetrain.applyRequest(() -> idle).ignoringDisable(true));

        joystick.a().onTrue(
                stateMachine.prepL1());
        joystick.x().onTrue(
                stateMachine.prepL2());
        joystick.y().onTrue(
                stateMachine.prepL3());
        
        joystick.b().onTrue(stateMachine.prepL4()); 

        joystick.rightBumper().onTrue(
                stateMachine.prepGroundAlgae());
        joystick.leftBumper().onTrue(stateMachine.prepReefAlgae());

        joystick.rightTrigger().onTrue(stateMachine.Score());

        joystick.leftTrigger().onTrue(stateMachine.intakeCoral());
        
     
     

        drivetrain.registerTelemetry(logger::telemeterize);

    }

    public Command getAutonomousCommand() {
        return m_chooser.selectedCommand();

    }

    // TODO: add to state machine, delete
    public Command Stow() {
        return yIntakePivot.setAngle(YAMSIntakePivot.L1_ANGLE);
    }

    public Command L1Score() {
        return intakeRoller.outTakeRollers();
    }

    public Command Handoff() {
        return yIntakePivot.setAngle(YAMSIntakePivot.HANDOFF_ANGLE).until(
                () -> Math.abs(yIntakePivot.getAngle().in(Degrees) - YAMSIntakePivot.HANDOFF_ANGLE.in(Degrees)) < 2.0);

    }

}
