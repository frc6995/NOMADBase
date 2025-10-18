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
import edu.wpi.first.units.measure.Distance;

import frc.robot.subsystems.HandS.HandConstants;
import frc.robot.subsystems.YAMSIntakePivot;
import frc.robot.subsystems.YAMSIntakeRollerS;

public class StateMachine {
    // TODO: add logging/simulation for states

    public final YAMSIntakeRollerS intakeRoller = new YAMSIntakeRollerS();

    public final HandS handRoller = new HandS();

    // public final HandConstants handConstants = new HandConstants();

    public final ArmS arm = new ArmS();

    public final ElevatorS elevator = new ElevatorS();
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
        BARGE_PREP,
        NO_GAME_PIECE

    }

    private RobotState currentState = RobotState.HANDOFF;

    public Command setState(RobotState newState) {
        return new InstantCommand(() -> currentState = newState);
    }

    // Functions below:
    // Todo: add command that combines intakeCoral and stowCoral, update states
    public Command intakeCoral() {
        return Commands.sequence(setState(RobotState.CORAL_INTAKING),
                Commands.race(yIntakePivot.setAngle(YAMSIntakePivot.DOWN_ANGLE), intakeRoller.coralIntake()));
    }

    public Command stowCoral() {
        return Commands.sequence(setState(RobotState.HANDOFF),
                yIntakePivot.setAngle(YAMSIntakePivot.HANDOFF_ANGLE));
    }

    public Command setUpperMechanism(Angle armAngle, Distance elevHeight) {
        return Commands.parallel(
                arm.setAngle(armAngle),
                elevator.setHeight(elevHeight));

    }

    // Commands below:
    // TODO: add handoff sequence

    public Command prepL4() {
        if (currentState == RobotState.HANDOFF) {
            return Commands.sequence(setState(RobotState.L4_PRE_SCORE),
                    setUpperMechanism(ArmS.HANDOFF_ANGLE, elevator.ELEVATOR_HANDOFF_HEIGHT),

                    Commands.parallel(handRoller.HandCoralIntake(),
                            intakeRoller.handoffOutTake()),
                    Commands.sequence(
                            setUpperMechanism(ArmS.L4_ANGLE, elevator.L4_HEIGHT)));

        } else {
            return Commands.none();

        }
    }

    public Command prepL3() {
        if (currentState == RobotState.HANDOFF) {
            return Commands.sequence(setState(RobotState.L3_PRE_SCORE),
                    setUpperMechanism(ArmS.HANDOFF_ANGLE, elevator.ELEVATOR_HANDOFF_HEIGHT),

                    Commands.parallel(handRoller.HandCoralIntake(),
                            intakeRoller.handoffOutTake()),
                    Commands.sequence(
                            setUpperMechanism(ArmS.L3_ANGLE, elevator.L3_HEIGHT)));

        } else {
            return Commands.none();

        }
    }

    public Command prepL2() {
        if (currentState == RobotState.HANDOFF) {
            return Commands.sequence(setState(RobotState.L2_PRE_SCORE),
                    setUpperMechanism(ArmS.HANDOFF_ANGLE, elevator.ELEVATOR_HANDOFF_HEIGHT),

                    Commands.parallel(handRoller.HandCoralIntake(),
                            intakeRoller.handoffOutTake()),
                    Commands.sequence(
                            setUpperMechanism(ArmS.L2_ANGLE, elevator.L2_HEIGHT)));

        } else {
            return Commands.none();

        }
    }

    public Command prepL1() {
        if (currentState == RobotState.HANDOFF) {
            return Commands.sequence(setState(RobotState.L1_PRE_SCORE),
                    yIntakePivot.setAngle(yIntakePivot.L1_ANGLE)

            );

        } else {
            return Commands.none();

        }
    }

    public Command prepGroundAlgae() {
        if (currentState == RobotState.NO_GAME_PIECE) {
            return Commands.sequence(setState(RobotState.INTAKING_ALGAE_REEF),

                    Commands.parallel(yIntakePivot.setAngle(yIntakePivot.L1_ANGLE),
                            setUpperMechanism(ArmS.GROUND_INTAKE_ANGLE, elevator.kElevatorMinHeight),
                            handRoller.setHandRollerVoltage(HandConstants.INTAKING_ALGAE_GROUND_VOlTAGE)

                    ));

        } else {
            return Commands.none();
        }

    }

    public Command prepReefAlgae() {
        if (currentState == RobotState.NO_GAME_PIECE) {
            return Commands.sequence(setState(RobotState.INTAKING_ALGAE_REEF),

                    Commands.parallel(yIntakePivot.setAngle(yIntakePivot.L1_ANGLE),
                            setUpperMechanism(ArmS.REEF_INTAKE_ALGAE_HIGH, elevator.kElevatorMinHeight),
                            handRoller.setHandRollerVoltage(HandConstants.INTAKING_ALGAE_GROUND_VOlTAGE)

                    ));

        } else {
            return Commands.none();
        }

    }

    public Command prepBarge() {
        if (currentState == RobotState.INTAKING_ALGAE_GROUND || currentState == RobotState.INTAKING_ALGAE_REEF) {
            return Commands.sequence(setState(RobotState.BARGE_PREP),
                    Commands.parallel(yIntakePivot.setAngle(yIntakePivot.L1_ANGLE),
                            setUpperMechanism(ArmS.REEF_INTAKE_ALGAE_HIGH, elevator.kElevatorMaxHeight),
                            handRoller.setHandRollerVoltage(HandConstants.INTAKING_ALGAE_GROUND_VOlTAGE)

                    ));

        } else {
            return Commands.none();
        }

    }

    public Command Score() {
        if (currentState == RobotState.L1_PRE_SCORE) {
            return Commands.sequence(intakeRoller.L1OutTake(), yIntakePivot.setAngle(yIntakePivot.HANDOFF_ANGLE),
                    setState(RobotState.NO_GAME_PIECE));

        }

        else if (currentState == RobotState.L2_PRE_SCORE) {
            return Commands.sequence(setUpperMechanism(Degrees.of(-30), Inches.of(36)),
            handRoller.handOutTakeRollers(),
            setState(RobotState.NO_GAME_PIECE)
            );

        }
        else if (currentState == RobotState.L3_PRE_SCORE) {
            return Commands.sequence(setUpperMechanism(Degrees.of(-30), Inches.of(48)),
            handRoller.handOutTakeRollers(),
            setState(RobotState.NO_GAME_PIECE)
            );
        }//Change the inches, not exact right now

        else if (currentState == RobotState.L4_PRE_SCORE){
            return Commands.sequence(setUpperMechanism(Degrees.of(-30), Inches.of(60)),
            handRoller.handOutTakeRollers(),
            setState(RobotState.NO_GAME_PIECE)
            );
        }
        else if (currentState == RobotState.BARGE_PREP){
            return Commands.sequence(setUpperMechanism(Degrees.of(-15), Inches.of(60)),handRoller.handOutTakeRollers(),
            setState(RobotState.NO_GAME_PIECE)
            );
        }
        else 
            return Commands.none();

    }

}
