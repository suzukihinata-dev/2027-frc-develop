package frc.robot.subsystems.swervedrive;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathPlannerPath;
import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.io.File;
import java.util.function.DoubleSupplier;
import swervelib.parser.SwerveParser;
import yams.mechanisms.config.SwerveDriveConfig;
import yams.mechanisms.swerve.SwerveDrive;
import yams.mechanisms.swerve.utility.SwerveInputStream;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.telemetry.SwerveDriveTelemetryConfig;

/**
 * Swerve drive subsystem built from the YAGSL JSON configuration in
 * src/main/deploy/swerve/base. All hardware (motors, encoders, gyro, gearing, PID) is defined
 * there — edit the JSON to match the robot, not this file.
 */
public class SwerveSubsystem extends SubsystemBase
{

  private final SwerveDrive drive;

  /** NavX2 (YAGSL's JSON has no NavX2 option, so the gyro type is "custom" and is passed in below). */
  private final AHRS navx = new AHRS(NavXComType.kMXP_SPI);

  public SwerveSubsystem()
  {
    var cfg = new SwerveDriveConfig()
        .withStartingPose(new Pose2d(3, 3, Rotation2d.kZero))
        .withSubsystem(this)
        // NavX reports clockwise-positive; WPILib is counter-clockwise-positive.
        .withGyro(() -> Degrees.of(-navx.getAngle()))
        .withGyroVelocity(() -> DegreesPerSecond.of(-navx.getRate()))
        .withTranslationController(new PIDController(4, 0, 0))
        .withRotationController(new PIDController(1, 0, 0))
        .withTelemetry("swerve", new SwerveDriveTelemetryConfig(TelemetryVerbosity.HIGH));

    SwerveParser.parse(new File(Filesystem.getDeployDirectory(), "swerve/base"));
    drive = SwerveParser.createSwerveDriveDevices(cfg).swerveDrive();

    configurePathPlanner();
  }

  /**
   * Configure PathPlanner's {@link AutoBuilder}. The robot's mass, MOI and module properties are
   * read from deploy/pathplanner/settings.json, which the PathPlanner GUI writes when its robot
   * config is saved. If that file is missing the robot still starts, but {@link #followPath}
   * does nothing.
   */
  private void configurePathPlanner()
  {
    try
    {
      RobotConfig config = RobotConfig.fromGUISettings();
      AutoBuilder.configure(
          this::getPose,
          this::resetOdometry,
          drive::getRobotRelativeSpeed,
          (speeds, feedforwards) -> drive.setRobotRelativeChassisSpeeds(speeds),
          new PPHolonomicDriveController(new PIDConstants(5, 0, 0), new PIDConstants(5, 0, 0)),
          config,
          this::isRedAlliance,
          this);
    } catch (Exception e)
    {
      DriverStation.reportError("PathPlanner config failed: " + e.getMessage(), e.getStackTrace());
    }
  }

  private boolean isRedAlliance()
  {
    return DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red;
  }

  /**
   * Follow a path drawn in the PathPlanner GUI. Odometry is first reset to the path's starting
   * pose, so the robot must be placed there. The path is flipped on the red alliance.
   *
   * @param pathName Name of the file under deploy/pathplanner/paths, without ".path".
   * @return Command that resets odometry and follows the path, or one that does nothing if the
   *         path can't be loaded.
   */
  public Command followPath(String pathName)
  {
    try
    {
      PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);
      Command resetToStart = Commands.runOnce(() -> {
        PathPlannerPath startPath = isRedAlliance() ? path.flipPath() : path;
        startPath.getStartingHolonomicPose().ifPresent(this::resetOdometry);
      });
      return resetToStart.andThen(AutoBuilder.followPath(path)).withName("Follow " + pathName);
    } catch (Exception e)
    {
      DriverStation.reportError("Failed to load path '" + pathName + "': " + e.getMessage(), e.getStackTrace());
      return Commands.none();
    }
  }

  /**
   * Create a {@link SwerveInputStream} from driver inputs, used as the input to {@link #drive}.
   *
   * @param x   X translation supplier, in [-1, 1].
   * @param y   Y translation supplier, in [-1, 1].
   * @param rot Angular velocity supplier, in [-1, 1].
   * @return Stream converting the inputs into chassis speeds.
   */
  public SwerveInputStream getAngularVelocityStream(DoubleSupplier x, DoubleSupplier y, DoubleSupplier rot)
  {
    return new SwerveInputStream(drive, x, y, rot);
  }

  /**
   * Drive the robot using the given stream.
   *
   * @param stream Driver input stream, see {@link #getAngularVelocityStream}.
   * @return Command that drives the robot until interrupted.
   */
  public Command drive(SwerveInputStream stream)
  {
    return drive.drive(
        () -> ChassisSpeeds.fromFieldRelativeSpeeds(stream.get(), new Rotation2d(drive.getGyroAngle())));
  }

  /**
   * Zero the gyro, resetting the robot's heading.
   *
   * @return Command that zeroes the gyro.
   */
  public Command zeroGyro()
  {
    return Commands.runOnce(drive::zeroGyro, this).withName("Zero Gyro");
  }

  /**
   * Gets the measured pose (position and rotation) of the robot, as reported by odometry.
   *
   * @return The robot's pose.
   */
  public Pose2d getPose()
  {
    return drive.getPose();
  }

  /**
   * Reset the pose estimator to the given pose.
   *
   * @param pose Pose to reset to. Field relative, blue-origin.
   */
  public void resetOdometry(Pose2d pose)
  {
    drive.resetOdometry(pose);
  }

  @Override
  public void periodic()
  {
    drive.updateTelemetry();
  }

  @Override
  public void simulationPeriodic()
  {
    drive.simIterate();
  }
}
