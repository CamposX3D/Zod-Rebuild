package frc.robot.subsystems;

import java.io.File;
import java.util.function.Supplier;

import com.ctre.phoenix6.hardware. Pigeon2;
import com. ctre.phoenix6.hardware. core.CorePigeon2;
import com. pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib. commands.PathfindingCommand;
import com. pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib. controllers.PPHolonomicDriveController;
import com. pathplanner.lib.path.PathConstraints;

import edu.wpi.first. math.VecBuilder;
import edu.wpi.first.math.geometry. Pose2d;
import edu.wpi.first.math. geometry.Rotation2d;
import edu.wpi.first. math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj. DriverStation;
import edu. wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first. wpilibj2.command.SubsystemBase;
import frc.robot.constants.SwerveConstants. Tracao;
import frc.robot.utils.LimelightHelpers;
import swervelib.SwerveController;
import swervelib. SwerveDrive;
import swervelib.parser.SwerveDriveConfiguration;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;


/**
 * Classe de subsistema onde fazemos a ponte do nosso código para YAGSL
 */
public class SwerveSubsystem extends SubsystemBase {
  
  private final SwerveDrive swerveDrive;
  public boolean correctionPID = false;
  private final CorePigeon2 pigeon;
  private final Pigeon2 m_gyro;

  private final Field2d field = new Field2d();

  public SwerveSubsystem(File directory) {
    SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;

    try {
      swerveDrive = new SwerveParser(directory).createSwerveDrive(Tracao.MAX_SPEED);
    } catch (Exception e) {
      throw new RuntimeException(e);
    }

    pigeon = new CorePigeon2(13);
    m_gyro = new Pigeon2(13);

    LimelightHelpers.setCameraPose_RobotSpace("limelight", 0.355, 0, 0.18, 0.35, 0.5, 0);
    LimelightHelpers.setPipelineIndex("limelight", 6);

    // LimelightHelpers.SetFieldLayout("limelight", "2026-rebuild");

    int[] validTagIDs = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 
                         17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32};
    LimelightHelpers.SetFiducialIDFiltersOverride("limelight", validTagIDs);

    swerveDrive.setHeadingCorrection(true);

    setupPathPlanner();
  }

  /**
   * Setup AutoBuilder for PathPlanner. 
   */
  public void setupPathPlanner() {
    RobotConfig config;
    try {
      config = RobotConfig.fromGUISettings();

      final boolean enableFeedforward = true;
      AutoBuilder. configure(
          this::getPose,
          this::resetOdometry,
          this::getRobotVelocity,
          (speedsRobotRelative, moduleFeedForwards) -> {
            if (enableFeedforward) {
              swerveDrive.drive(
                  speedsRobotRelative,
                  swerveDrive.kinematics. toSwerveModuleStates(speedsRobotRelative),
                  moduleFeedForwards. linearForces());
            } else {
              swerveDrive.setChassisSpeeds(speedsRobotRelative);
            }
          },
          new PPHolonomicDriveController(
              new PIDConstants(0.8, 0.0, 0.035),
              new PIDConstants(1.75, 0, 0.015)
          ),
          config,
          () -> {
            var alliance = DriverStation.getAlliance();
            if (alliance.isPresent()) {
              return alliance.get() == DriverStation.Alliance.Red;
            }
            return false;
          },
          this
      );

    } catch (Exception e) {
      e.printStackTrace();
    }
    PathfindingCommand.warmupCommand().schedule();
  }

  @Override
  public void periodic() {
    swerveDrive.updateOdometry();

    LimelightHelpers.SetRobotOrientation(
        "limelight",
        swerveDrive.getYaw().getDegrees(),
        getYawRate(),
        m_gyro.getPitch().getValueAsDouble(),
        0,
        m_gyro.getRoll().getValueAsDouble(),
        0
    );

    updateVisionOdometry();

    field.setRobotPose(swerveDrive.getPose());
    SmartDashboard.putData("Field", field);
    SmartDashboard.putNumber("Odometry X", swerveDrive. getPose().getX());
    SmartDashboard.putNumber("Odometry Y", swerveDrive.getPose().getY());
    SmartDashboard.putNumber("Odometry Theta", swerveDrive.getPose().getRotation().getDegrees());
  }

  private void updateVisionOdometry() {
    boolean useMegaTag2 = true;

    LimelightHelpers.PoseEstimate poseEstimate;

    if (useMegaTag2) {
      var alliance = DriverStation.getAlliance();
      if (alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red) {
        poseEstimate = LimelightHelpers.getBotPoseEstimate_wpiRed_MegaTag2("limelight");
      } else {
        poseEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight");
      }
    } else {
      var alliance = DriverStation.getAlliance();
      if (alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red) {
        poseEstimate = LimelightHelpers. getBotPoseEstimate_wpiRed("limelight");
      } else {
        poseEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight");
      }
    }

    if (poseEstimate == null || poseEstimate.tagCount == 0) {
      return;
    }

    if (poseEstimate.pose.getX() == 0 && poseEstimate.pose. getY() == 0) {
      return;
    }

    if (Math.abs(getYawRate()) > 720) {
      return;
    }

    if (! useMegaTag2 && poseEstimate.rawFiducials != null && poseEstimate.rawFiducials.length > 0) {
      for (var fiducial : poseEstimate.rawFiducials) {
        if (fiducial.ambiguity > 0.7) {
          return;
        }
        if (fiducial.distToCamera > 5.0) {
          return;
        }
      }
    }

    double xyStdDev = 0.7;
    double thetaStdDev = 9999999;

    if (poseEstimate.tagCount >= 2) {
      xyStdDev = 0.5;
    }
    if (poseEstimate. avgTagDist > 3.5) {
      xyStdDev = 1.0;
    }

    swerveDrive.addVisionMeasurement(
        poseEstimate.pose,
        poseEstimate.timestampSeconds,
        VecBuilder.fill(xyStdDev, xyStdDev, thetaStdDev)
    );
  }

  public void drive(Translation2d translation, double rotation, boolean fieldRelative){
    swerveDrive.drive(translation,
      rotation,
      fieldRelative,
      (false));
  }

  public void driveField(Supplier<ChassisSpeeds> velocity) {
    swerveDrive. driveFieldOriented(velocity. get());
  }

  public Command driveFieldOriented(Supplier<ChassisSpeeds> velocity) {
    return run(() -> {
      swerveDrive.driveFieldOriented(velocity.get());
    });
  }

  public ChassisSpeeds getTargetSpeeds(double xInput, double yInput, double headingX, double headingY) {
    return swerveDrive. swerveController.getTargetSpeeds(
        xInput,
        yInput,
        headingX,
        headingY,
        getHeading().getRadians());
  }

  public ChassisSpeeds getTargetSpeeds(double xInput, double yInput) {
    return swerveDrive.swerveController.getTargetSpeeds(xInput, yInput, 0, 0, 0, Tracao.MAX_SPEED);
  }

  public Pose2d getPose() {
    return swerveDrive.getPose();
  }

  public ChassisSpeeds getFieldVelocity() {
    return swerveDrive.getFieldVelocity();
  }

  public SwerveDriveConfiguration getSwerveDriveConfiguration() {
    return swerveDrive.swerveDriveConfiguration;
  }

  public SwerveController getSwerveController() {
    return swerveDrive.getSwerveController();
  }

  public Rotation2d getHeading() {
    return swerveDrive.getYaw();
  }

  public void resetOdometry(Pose2d posicao) {
    swerveDrive.resetOdometry(posicao);
  }

  public void resetGyro() {
    swerveDrive.zeroGyro();
  }

  public void resetHeading() {
    swerveDrive.setHeadingCorrection(true);
    correctionPID = true;
  }

  public void disableHeading() {
    correctionPID = false;
    swerveDrive.setHeadingCorrection(false);
  }

  public void setChassisSpeeds(ChassisSpeeds chassisSpeeds) {
    swerveDrive.setChassisSpeeds(chassisSpeeds);
  }

  public void setMotorBrake(boolean brake) {
    swerveDrive.setMotorIdleMode(brake);
  }

  public ChassisSpeeds getRobotVelocity() {
    return swerveDrive.getRobotVelocity();
  }

  public ChassisSpeeds discretize(ChassisSpeeds speeds) {
    var desiredDeltaPose = new Pose2d(
        speeds.vxMetersPerSecond * Tracao.DT,
        speeds.vyMetersPerSecond * Tracao.DT,
        new Rotation2d(speeds.omegaRadiansPerSecond * Tracao.DT * Tracao.constantRotation));

    var twist = new Pose2d().log(desiredDeltaPose);

    return new ChassisSpeeds((twist.dx / Tracao.DT), (twist.dy / Tracao. DT), (speeds.omegaRadiansPerSecond));
  }

  public Command getAutonomousCommand(String pathName, boolean setOdomToStart) {
    return new PathPlannerAuto(pathName);
  }

  public SwerveDrive getSwerveDrive() {
    return swerveDrive;
  }

  public Command driveToPose(Pose2d pose) {
    PathConstraints constraints = new PathConstraints(
        swerveDrive.getMaximumChassisVelocity(), 4.0,
        swerveDrive.getMaximumChassisAngularVelocity(), Units.degreesToRadians(720));

    return AutoBuilder.pathfindToPose(
        pose,
        constraints,
        edu.wpi.first.units.Units.MetersPerSecond. of(0)
    );
  }

  public double getYawRate() {
    return pigeon.getAngularVelocityZWorld().getValueAsDouble();
  }
}