package frc.robot;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.Matrix;

import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.Pigeon2;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.generated.TunerConstants;

public class Vision {

    private SwerveDrivePoseEstimator m_poseEstimator;
    private Pigeon2 m_gyro;

    // Constructor to initialize members
    public Vision(SwerveDrivePoseEstimator poseEstimator, Pigeon2 gyro) {
        this.m_poseEstimator = poseEstimator;
        this.m_gyro = gyro;
    }

    LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight");

    public void update() {
        LimelightHelpers.SetRobotOrientation("limelight",
                m_poseEstimator.getEstimatedPosition().getRotation().getDegrees(), 0, 0, 0, 0, 0);

        LimelightHelpers.SetIMUMode("limelight", 0);
        boolean doRejectUpdate = false;
        // if our angular velocity is greater than 360 degrees per second, ignore vision
        // updates
        if (Math.abs(m_gyro.getRate()) > 360) {
            doRejectUpdate = true;
        }
        if (mt2.tagCount == 0) {
            doRejectUpdate = true;
        }
        if (!doRejectUpdate) {
            m_poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(.7, .7, 9999999));
            m_poseEstimator.addVisionMeasurement(
                    mt2.pose,
                    mt2.timestampSeconds);

        }
    }

    public Pose2d getMegaTag2Pose() {
        // Return the MegaTag 2 pose
        return mt2.pose;
    }

    public double getMegaTag2Timestamp() {
        // Return the MegaTag 2 timestamp
        return mt2.timestampSeconds;
    }

    public Matrix<N3, N1> getMegaTag2StdDevs() {
        // Return nothing
        return getMegaTag2StdDevs();
    }
}
