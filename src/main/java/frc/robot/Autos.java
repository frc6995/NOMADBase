package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.wpilibj2.command.Commands.deadline;
import static edu.wpi.first.wpilibj2.command.Commands.defer;
import static edu.wpi.first.wpilibj2.command.Commands.either;
import static edu.wpi.first.wpilibj2.command.Commands.none;
import static edu.wpi.first.wpilibj2.command.Commands.parallel;
import static edu.wpi.first.wpilibj2.command.Commands.print;
import static edu.wpi.first.wpilibj2.command.Commands.race;
import static edu.wpi.first.wpilibj2.command.Commands.runOnce;
import static edu.wpi.first.wpilibj2.command.Commands.select;
import static edu.wpi.first.wpilibj2.command.Commands.sequence;
import static edu.wpi.first.wpilibj2.command.Commands.waitSeconds;
import static edu.wpi.first.wpilibj2.command.Commands.waitUntil;

import java.util.HashMap;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;
import java.util.Map.Entry;
import java.util.NoSuchElementException;
import java.util.Optional;
import java.util.Set;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Function;
import java.util.function.Supplier;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.therekrab.autopilot.APTarget;
import com.ctre.phoenix6.swerve.SwerveRequest;

import choreo.Choreo.TrajectoryLogger;
import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import choreo.auto.AutoChooser;
import choreo.trajectory.SwerveSample;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Logged.Strategy;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.YAMSIntakePivot;

public class Autos {
    private final AutoFactory m_factory;
    private final RobotContainer m_container;
    protected final CommandSwerveDrivetrain m_drivebase;
    protected final YAMSIntakePivot m_intakepiv;
    private final StateMachine stateMachine = new StateMachine();
    private final double SCORE_WAIT = 0.875;

    public Autos(CommandSwerveDrivetrain drivebase, YAMSIntakePivot intakepiv,
            AutoFactory factory, RobotContainer container, StateMachine stateMachine) {
        m_drivebase = drivebase; // need
        m_intakepiv = intakepiv;
        m_factory = factory;
        m_container = container;

        container.m_chooser.addRoutine(simpleAutoName, this::simpleAuto);
        container.m_chooser.addRoutine(backsideL1Name, this::backsideL1);
    }

    //Example auto
    String simpleAutoName = "Simple Auto";
    public AutoRoutine simpleAuto() {
        final AutoRoutine routine = m_factory.newRoutine(simpleAutoName);
        final AutoTrajectory traj = routine.trajectory("1");
        routine.active().onTrue(
                traj.resetOdometry()
                        .andThen(traj.cmd())
                        .andThen(createAutoAlignCommand(new Pose2d(ChoreoVariables.getPose("Lolipop1").getX(), ChoreoVariables.getPose("Lolipop1").getY(), ChoreoVariables.getPose("Lolipop1").getRotation()))));
        return routine;
    }

    //Example auto
    String backsideL1Name = "Backside L1";
    public AutoRoutine backsideL1() {
        final AutoRoutine routine = m_factory.newRoutine(backsideL1Name);
        var firstScore = routine.trajectory("BacksideL1(1)");
        var postScoreIntake = routine.trajectory("BacksideL1(2)");
        routine.active().onTrue(
                firstScore.resetOdometry()
                        .andThen(firstScore.cmd()
                                .andThen(waitSeconds(SCORE_WAIT))
                                .andThen(postScoreIntake.cmd())));

        firstScore.atTimeBeforeEnd(0.2).onTrue(stateMachine.prepL1());
        // firstScore.doneFor(0.1).onTrue(null/*stateMachine.scoreL1());
        // firstScore.atTimeBeforeEnd(0.1).onTrue(stateMachine.intakeCoral());
        return routine;
    }

    /**
     * Creates a new Command using the Autopilot AutoAlign to navigate to the targetPose. 
     * 
     * @param targetPose The desired ending Pose2d
     * @return The Command to navigate to the given Pose2d.
     */
    public Command createAutoAlignCommand(Pose2d targetPose) {
        return new AutoAlign(new APTarget(targetPose), m_drivebase);
    }

    /**
     * Creates a new Command using the Autopilot AutoAlign to navigate to the targetPose. Takes a desired entry angle
     * when approaching the targetPose.
     * 
     * @param targetPose The desired ending Pose2d
     * @param entryAngle The desired angle to approach the targetPose with.
     * @return The Command to navigate to the given Pose2d.
     */
    public Command createAutoAlignCommand(Pose2d targetPose, Rotation2d entryAngle) {
        return new AutoAlign(new APTarget(targetPose).withEntryAngle(entryAngle), m_drivebase);
    }
}
