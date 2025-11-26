package frc.robot;

import static edu.wpi.first.units.Units.*;

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
import frc.robot.subsystems.CommandSwerveDrivetrain;
import edu.wpi.first.units.measure.Distance;

import frc.robot.subsystems.YAMSIntakePivot;
import frc.robot.subsystems.YAMSIntakePivot.intakeConstants;

public class StateMachine {
    // TODO: add logging/simulation for states

    public final YAMSIntakePivot yIntakePivot = new YAMSIntakePivot();

    public enum RobotState {
        // Todo: add all states as in button mapping doc
        CORAL_INTAKING,
        HANDOFF,
        L1_PRE_SCORE,
        L2_PRE_SCORE,
        L3_PRE_SCORE,
        L4_PRE_SCORE,
        INTAKING_ALGAE_GROUND,
        INTAKING_ALGAE_REEF,
        ALGAE_STOW,
        BARGE_PREP

    }

    public RobotState currentState = RobotState.HANDOFF;

    public Command setState(RobotState newState) {
        return new InstantCommand(() -> currentState = newState);
    }

    // Functions below:
    // Todo: add command that combines intakeCoral and stowCoral, update states

    public Command stowCoral() {
        return Commands.sequence(setState(RobotState.HANDOFF),
                yIntakePivot.setAngle(intakeConstants.HANDOFF_ANGLE));
    }

    // Commands below:
    // TODO: add handoff sequence

    public Command prepL1() {
        return Commands.sequence(setState(RobotState.L1_PRE_SCORE),
                yIntakePivot.setAngle(intakeConstants.L1_ANGLE)

        );
    }
}
