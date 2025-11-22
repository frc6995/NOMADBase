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
    protected final CommandSwerveDrivetrain m_drivebase;
    protected final YAMSIntakePivot m_intakepiv;
    private final double SCORE_WAIT = 0.875;

    public Autos(CommandSwerveDrivetrain drivebase, YAMSIntakePivot intakepiv,
            AutoFactory factory) {
        m_drivebase = drivebase; // need
        m_intakepiv = intakepiv;
        m_factory = factory;

    }

    public void resetOdometry() {

    }

    public AutoRoutine simpleAuto() {
        final AutoRoutine routine = m_factory.newRoutine("Simple Auto");
        final AutoTrajectory traj = routine.trajectory("1");
        routine.active().onTrue(
                traj.resetOdometry()
                        .andThen(traj.cmd()));  
        return routine;
    }
    /*
     * var traj = routine.trajectory("1");
          traj.atTime(0).onTrue(m_arm.goToPosition(Arm.Positions.PIVOT_SIDE_LOW_ALGAE)).onTrue(m_hand.inAlgae());

          var toScore = routine.trajectory("3");
          traj.chain(toScore);
          var moveback = routine.trajectory("4");
          toScore.atTime(0.3).onTrue(m_arm.goToPosition(Arm.Positions.STOW));
          toScore.done().onTrue(bargeUpAndOutVoltage()).onTrue(waitSeconds(1.2).andThen(moveback.spawnCmd()));
     */

}
