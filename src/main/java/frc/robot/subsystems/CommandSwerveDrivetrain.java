package frc.robot.subsystems;

import java.util.function.Supplier;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.LimelightHelpers;

import java.util.concurrent.locks.ReadWriteLock;
import java.util.concurrent.locks.ReentrantReadWriteLock;

/**
 * Class that extends the Phoenix SwerveDrivetrain class and implements
 * subsystem so it can be used in command-based projects easily.
 */
public class CommandSwerveDrivetrain extends SwerveDrivetrain implements Subsystem {
  private static final double kSimLoopPeriod = 0.005; // 5 ms
  private Notifier m_simNotifier = null;
  private double m_lastSimTime;

  private final ReadWriteLock m_visionLock = new ReentrantReadWriteLock();

  /*
   * private final SwerveDrivePoseEstimator m_poseEstimator =
   * new SwerveDrivePoseEstimator(
   * m_kinematics,
   * m_pigeon2.getRotation2d(),
   * m_modulePositions,
   * new Pose2d(),
   * VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(5)),
   * VecBuilder.fill(0.5, 0.5, Units.degreesToRadians(30)));
   */
  /* Blue alliance sees forward as 0 degrees (toward red alliance wall) */
  private final Rotation2d BlueAlliancePerspectiveRotation = Rotation2d.fromDegrees(0);
  /* Red alliance sees forward as 180 degrees (toward blue alliance wall) */
  private final Rotation2d RedAlliancePerspectiveRotation = Rotation2d.fromDegrees(180);
  /* Keep track if we've ever applied the operator perspective before or not */
  private boolean hasAppliedOperatorPerspective = false;

  public CommandSwerveDrivetrain(SwerveDrivetrainConstants driveTrainConstants, double OdometryUpdateFrequency,
      SwerveModuleConstants... modules) {
    super(driveTrainConstants, OdometryUpdateFrequency, modules);
    if (Utils.isSimulation()) {
      startSimThread();
    }
  }

  public CommandSwerveDrivetrain(SwerveDrivetrainConstants driveTrainConstants, SwerveModuleConstants... modules) {
    super(driveTrainConstants, modules);
    if (Utils.isSimulation()) {
      startSimThread();
    }
  }

  public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
    return run(() -> this.setControl(requestSupplier.get()));
  }

  private void startSimThread() {
    m_lastSimTime = Utils.getCurrentTimeSeconds();

    /* Run simulation at a faster rate so PID gains behave more reasonably */
    m_simNotifier = new Notifier(() -> {
      final double currentTime = Utils.getCurrentTimeSeconds();
      double deltaTime = currentTime - m_lastSimTime;
      m_lastSimTime = currentTime;

      /* use the measured time delta, get battery voltage from WPILib */
      updateSimState(deltaTime, RobotController.getBatteryVoltage());
    });
    m_simNotifier.startPeriodic(kSimLoopPeriod);
  }

  @Override
  public void periodic() {
    /* Periodically try to apply the operator perspective */
    /*
     * If we haven't applied the operator perspective before, then we should apply
     * it regardless of DS state
     */
    /*
     * This allows us to correct the perspective in case the robot code restarts
     * mid-match
     */
    /*
     * Otherwise, only check and apply the operator perspective if the DS is
     * disabled
     */
    /*
     * This ensures driving behavior doesn't change until an explicit disable event
     * occurs during testing
     */
    if (!hasAppliedOperatorPerspective || DriverStation.isDisabled()) {
      DriverStation.getAlliance().ifPresent((allianceColor) -> {
        this.setOperatorPerspectiveForward(
            allianceColor == Alliance.Red ? RedAlliancePerspectiveRotation
                : BlueAlliancePerspectiveRotation);
        hasAppliedOperatorPerspective = true;
      });
    }

    updateOdometry();

    SmartDashboard.putString("Pose", m_odometry.getEstimatedPosition().toString());
    SmartDashboard.putString("Yaw", m_pigeon2.getYaw().toString());
    
  }

  public void updateOdometry() {
    try {
      m_stateLock.writeLock().lock(); //lock access to the odometry (also being locked by SwerveDrivetrain)

      m_odometry.update(Rotation2d.fromDegrees(BaseStatusSignal.getLatencyCompensatedValue(
          m_yawGetter, m_angularVelocity)), m_modulePositions);

      boolean useMegaTag2 = true; // set to false to use MegaTag1
      boolean doRejectUpdate = false;
      if (useMegaTag2 == false) {
        LimelightHelpers.PoseEstimate mt1 = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight");

        if (mt1.tagCount == 1 && mt1.rawFiducials.length == 1) {
          if (mt1.rawFiducials[0].ambiguity > .7) {
            doRejectUpdate = true;
          }
          if (mt1.rawFiducials[0].distToCamera > 3) {
            doRejectUpdate = true;
          }
        }
        if (mt1.tagCount == 0) {
          doRejectUpdate = true;
        }

        if (!doRejectUpdate) {
          m_odometry.setVisionMeasurementStdDevs(VecBuilder.fill(.5, .5, 9999999));
          m_odometry.addVisionMeasurement(
              mt1.pose,
              mt1.timestampSeconds);
        }
      } else if (useMegaTag2 == true) {
        LimelightHelpers.SetRobotOrientation("limelight", m_odometry.getEstimatedPosition().getRotation().getDegrees(),
            0, 0, 0, 0, 0);
        LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight");
        if (mt2 == null) {
        } else {
          if (Math.abs(m_pigeon2.getRate()) > 720) // if our angular velocity is greater than 720 degrees per second,
                                                   // ignore vision updates
          {
            doRejectUpdate = true;
          }
          if (mt2.tagCount == 0) {
            doRejectUpdate = true;
          }
        }

        if (!doRejectUpdate) {
          //update odometry
          m_odometry.setVisionMeasurementStdDevs(VecBuilder.fill(.7, .7, 9999999));
          m_odometry.addVisionMeasurement(
              mt2.pose,
              mt2.timestampSeconds);
        }
      }
    } finally {
      //release the lock
      m_stateLock.writeLock().unlock();
    }

  }

}
