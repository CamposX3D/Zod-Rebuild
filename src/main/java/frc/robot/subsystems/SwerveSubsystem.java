package frc.robot.subsystems;

import java.io.File;
import java.util.function.Supplier;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.hardware.core.CorePigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.SwerveConstants.Tracao;
import frc.robot.utils.LimelightHelpers;
import swervelib.SwerveController;
import swervelib.SwerveDrive;
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

  // Método construtor da classe
  public SwerveSubsystem(File directory) {
    SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;

    try {
      swerveDrive = new SwerveParser(directory).createSwerveDrive(Tracao.MAX_SPEED);
    } catch (Exception e) {
      throw new RuntimeException(e);
    }

    pigeon = new CorePigeon2(13);
    m_gyro = new Pigeon2(13);

    //Posição da limelight no robô
    LimelightHelpers.setCameraPose_RobotSpace("limelight", 0.355, 0, 0.18, 0.35, 0.5,0);

    swerveDrive.setHeadingCorrection(true);

    setupPathPlanner();
  }

  /**
   * Setup AutoBuilder for PathPlanner.
   */

  public void setupPathPlanner() {
    // Load the RobotConfig from the GUI settings. You should probably
    // store this in your Constants file
    RobotConfig config;
    try {
      config = RobotConfig.fromGUISettings();

      final boolean enableFeedforward = true;
      // Configure AutoBuilder last
      AutoBuilder.configure(
          this::getPose,
          // Robot pose supplier
          this::resetOdometry,
          // Method to reset odometry (will be called if your auto has a starting pose)
          this::getRobotVelocity,
          // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
          (speedsRobotRelative, moduleFeedForwards) -> {
            if (enableFeedforward) {
              swerveDrive.drive(
                  speedsRobotRelative,
                  swerveDrive.kinematics.toSwerveModuleStates(speedsRobotRelative),
                  moduleFeedForwards.linearForces());
            } else {
              swerveDrive.setChassisSpeeds(speedsRobotRelative);
            }
          },
          // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also
          // optionally outputs individual module feedforwards
          new PPHolonomicDriveController(
              // PPHolonomicController is the built in path following controller for holonomic
              // drive trains
              new PIDConstants(0.8, 0.0, 0.035),
              // Translation PID constants
              new PIDConstants(1.75, 0, 0.015)
          // Rotation PID constants
          ),
          config,
          // The robot configuration
          () -> {
            var alliance = DriverStation.getAlliance();
            if (alliance.isPresent()) {
              return alliance.get() == DriverStation.Alliance.Red;
            }
            return false;
          },
          this
      // Reference to this subsystem to set requirements
      );

    } catch (Exception e) {
      // Handle exception as needed
      e.printStackTrace();
    }
    // Preload PathPlanner Path finding
    // IF USING CUSTOM PATHFINDER ADD BEFORE THIS LINE
    PathfindingCommand.warmupCommand().schedule();
  }

  @Override
  public void periodic() {

    swerveDrive.updateOdometry();

    SmartDashboard.putData(field);

  }

  public void drive(Translation2d translation, double rotation, boolean fieldRelative){
    swerveDrive.drive(translation,
      rotation,
      fieldRelative,
      (false));
  }


  public void driveField(Supplier<ChassisSpeeds> velocity) {
    swerveDrive.driveFieldOriented(velocity.get());
  }

  public Command driveFieldOriented(Supplier<ChassisSpeeds> velocity) {
    return run(() -> {
      swerveDrive.driveFieldOriented(velocity.get());
    });
  }

  public ChassisSpeeds getTargetSpeeds(double xInput, double yInput, double headingX, double headingY) {
    return swerveDrive.swerveController.getTargetSpeeds(
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

    return new ChassisSpeeds((twist.dx / Tracao.DT), (twist.dy / Tracao.DT), (speeds.omegaRadiansPerSecond));
  }

  public Command getAutonomousCommand(String pathName, boolean setOdomToStart) {

    // Create a path following command using AutoBuilder. This will also trigger
    // event markers.
    return new PathPlannerAuto(pathName);
  }

  public SwerveDrive getSwerveDrive() {
    return swerveDrive;
  }

  public Command driveToPose(Pose2d pose) {
    // Create the constraints to use while pathfinding
    PathConstraints constraints = new PathConstraints(
        swerveDrive.getMaximumChassisVelocity(), 4.0,
        swerveDrive.getMaximumChassisAngularVelocity(), Units.degreesToRadians(720));

    // Since AutoBuilder is configured, we can use it to build pathfinding commands
    return AutoBuilder.pathfindToPose(
        pose,
        constraints,
        edu.wpi.first.units.Units.MetersPerSecond.of(0) // Goal end velocity in meters/sec
    );
  }

  // public double robotYawInDegrees() {
  //   if (DriverStation.getAlliance().get() == Alliance.Blue) {
  //     return 0.0;
  //   } else {
  //     return 1800.0;
  //   }
  // }

  public double getYawRate() {
    return pigeon.getAngularVelocityZWorld().getValueAsDouble();
  }
  
//   public void UpdateOdometry() {

//     boolean useMegaTag2 = true; //set to false to use MegaTag1
//     boolean doRejectUpdate = false;
//     if(useMegaTag2 == false)
//     {
//       LimelightHelpers.PoseEstimate mt1 = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight");
      
//       if(mt1.tagCount == 1 && mt1.rawFiducials.length == 1)
//       {
//         if(mt1.rawFiducials[0].ambiguity > .7)
//         {
//           doRejectUpdate = true;
//         }
//         if(mt1.rawFiducials[0].distToCamera > 3)
//         {
//           doRejectUpdate = true;
//         }
//       }
//       if(mt1.tagCount == 0)
//       {
//         doRejectUpdate = true;
//       }

//       if(!doRejectUpdate)
//       {
//         swerveDrive.setVisionMeasurementStdDevs(VecBuilder.fill(.5,.5,9999999));
//         swerveDrive.addVisionMeasurement(
//             mt1.pose,
//             mt1.timestampSeconds);
//       }
//     }
//     else if (useMegaTag2 == true)
//     {
//       // LimelightHelpers.SetRobotOrientation("limelight", 0, 0, 0, 0, 0, 0);
//       LimelightHelpers.SetRobotOrientation("limelight", swerveDrive.getOdometryHeading().getDegrees(), getYawRate(), m_gyro.getPitch().getValueAsDouble(), m_gyro.getAngularVelocityYWorld().getValueAsDouble(), m_gyro.getRoll().getValueAsDouble(), m_gyro.getAngularVelocityXWorld().getValueAsDouble());
//       LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight");
//       if(Math.abs(m_gyro.getRate()) > 720) // if our angular velocity is greater than 720 degrees per second, ignore vision updates
//       {
//         doRejectUpdate = true;
//       }
//       if(mt2.tagCount == 0)
//       {
//         doRejectUpdate = true;
//       }
//       if(!doRejectUpdate)
//       {
//         swerveDrive.setVisionMeasurementStdDevs(VecBuilder.fill(.7,.7,9999999));
//         swerveDrive.addVisionMeasurement(
//             mt2.pose,
//             mt2.timestampSeconds);
//       }
//     }
//   }

  // public void addVisionUpdate(){
  //   swerveDrive.addVisionMeasurement(new Pose2d(3,3,Rotation2d.fromDegrees(65)), Timer.getTimestamp());
  // }
} 