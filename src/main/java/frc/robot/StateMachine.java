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
import frc.robot.subsystems.ArmS;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.ElevatorS;
import frc.robot.subsystems.HandS;
import edu.wpi.first.units.measure.Distance;

import frc.robot.subsystems.HandS.HandConstants;
import frc.robot.subsystems.YAMSIntakePivot;
import frc.robot.subsystems.YAMSIntakeRollerS;


public class StateMachine {
    //TODO: add logging/simulation for states

    public final YAMSIntakeRollerS intakeRoller = new YAMSIntakeRollerS();

    public final HandS handRoller = new HandS();

    public final ArmS arm = new ArmS();

    public final ElevatorS elevator = new ElevatorS();
    public final YAMSIntakePivot yIntakePivot = new YAMSIntakePivot();

    public enum RobotState{
        //Todo: add all states as in button mapping doc
        CORAL_INTAKING,
        INTAKE_STOW,
        PREP_L2

    }


    private RobotState currentState = RobotState.INTAKE_STOW;

    public Command setState(RobotState newState) {
        return new InstantCommand(() -> currentState = newState);
    }



    //Todo: add command that combines intakeCoral and stowCoral, update states
    public Command intakeCoral() {
        return Commands.sequence(setState(RobotState.CORAL_INTAKING),
        Commands.race(yIntakePivot.setAngle(YAMSIntakePivot.DOWN_ANGLE), intakeRoller.coralIntake())
        )
        ;
    }
    
     public Command stowCoral() {
        return Commands.sequence(setState(RobotState.INTAKE_STOW),
            yIntakePivot.setAngle(YAMSIntakePivot.SOME_ANGLE)
        );
    }

    public Command prepL2() {
        if (currentState == RobotState.PREP_L2) {
            return Commands.none();
        }
        else {
            return Commands.sequence(setState(RobotState.PREP_L2),
                setUpperMechanism(ArmS.L2_ANGLE, elevator.L2_HEIGHT)
                )
            ;
        }
    }



    public Command setUpperMechanism(Angle armAngle, Distance elevHeight) {
        return Commands.parallel(
            arm.setAngle(armAngle),
            elevator.setHeight(elevHeight)
        );


    }


}


