package frc.robot;

import java.util.ArrayList;
import java.util.Optional;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.PhotonvisionConstants;
import frc.robot.Constants.SwerveConstants.ChassisKinematics;
import frc.robot.subsystems.swerve.SubsystemSwerveDrivetrain;
import frc.robot.utility.AHRS_IMU;
import frc.robot.utility.IMU;
import frc.robot.utility.PhotonCameraGroup;
import frc.robot.utility.PhotonCameraGroup.Measurement;

public class DataManager {
  /** Singleton instance */
  private static DataManager instance;

  // PROPOSAL: Deprecate or remove this method.
  // REASON: Maintaining Dependency Inversion (the D in SOLID) to avoid
  // DataManager being used before it is configured, or used in places
  // another system would be better. The DataManager instance should always
  // be accessed through dependency injection (passing a thing into a constructor)
  // to this end.
  // H!
  public static DataManager instance() {
    return instance;
  }

  /**
   * An {@link Entry} that is automatically updated upon a call to
   * {@link #update DataManager.instance().update}.
   */
  public abstract class DataManagerEntry<T> extends Entry<T> {
    public DataManagerEntry() {
      entries.add(this);
    }
  }

  public class RobotPosition extends DataManagerEntry<Pose2d> {
    /** used to combine vision and odometry data */
    private SwerveDrivePoseEstimator poseEstimator;

    private SubsystemSwerveDrivetrain subsystemSwerveDrivetrain;
    private PhotonCameraGroup photonGroup = PhotonvisionConstants.cameraGroup;
    private IMU imu = new AHRS_IMU();

    private Field2d field2d = new Field2d();

    public RobotPosition(RobotContainer robotContainer) {
      subsystemSwerveDrivetrain = robotContainer.subsystemSwerveDrivetrain;
      poseEstimator = new SwerveDrivePoseEstimator(ChassisKinematics.kDriveKinematics, imu.getRotation(),
          subsystemSwerveDrivetrain.getModulePositions(), new Pose2d());
    }

    public void update() {
      poseEstimator.update(imu.getRotation(), subsystemSwerveDrivetrain.getModulePositions());
      Optional<Measurement> measurement = photonGroup.getMeasurement(get().getRotation(), Timer.getFPGATimestamp());

      if (measurement.isPresent()) {
        double ambiguity = measurement.get().ambiguity;
        poseEstimator.addVisionMeasurement(measurement.get().pose.estimatedPose.toPose2d(),
            measurement.get().pose.timestampSeconds, VecBuilder.fill(ambiguity + 0.5, ambiguity + 0.5, ambiguity + 0.5));
      }

      field2d.setRobotPose(get());
      SmartDashboard.putData(field2d);
    }

    public void set(Pose2d newPose) {
      poseEstimator.resetPosition(imu.getRotationModulus(), subsystemSwerveDrivetrain.getModulePositions(), newPose);
    }

    public Pose2d get() {
      return poseEstimator.getEstimatedPosition();
    }
  }

  public static enum ElevatorSetpoint {
  
  }

  public static class ElevatorPositionData {
 
    }
  

  public class ElevatorPosition extends DataManagerEntry<ElevatorPositionData> {

    public ElevatorPosition(RobotContainer robotContainer) {
    }

    @Override
    public ElevatorPositionData get() {
      //TODO
      return null; //return null for now
    }
  }

  @SuppressWarnings("rawtypes")
  private ArrayList<DataManagerEntry> entries = new ArrayList<>();

  public DataManagerEntry<Pose2d> robotPosition;

  public void update() {
    entries.forEach(entry -> entry.update());
  }

  /**
   * Constructs the singleton from member variables of a RobotContainer.
   * Should be called ASAP, preferably in the constructor for RobotContainer.
   */
  public DataManager(RobotContainer robotContainer) {
    robotPosition = new RobotPosition(robotContainer);

    instance = this;
  }
}
