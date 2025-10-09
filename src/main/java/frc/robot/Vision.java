package frc.robot;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;

import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.Pigeon2;
import frc.robot.LimelightHelpers;
import frc.robot.generated.TunerConstants;

public class Vision {

    private SwerveDrivePoseEstimator m_poseEstimator;
    private Pigeon2 m_gyro = new Pigeon2(75, TunerConstants.kCANBus2); // Assuming m_gyro is a member of this class

    // Constructor to initialize members
    public Vision(SwerveDrivePoseEstimator poseEstimator, Pigeon2 gyro) {
        this.m_poseEstimator = poseEstimator;
        this.m_gyro = gyro;
    }

    public void update() {
        // The following logic seems to be for a periodic update
        LimelightHelpers.SetRobotOrientation("limelight",
                m_poseEstimator.getEstimatedPosition().getRotation().getDegrees(), 0, 0, 0, 0, 0);
        LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight");

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
}
