package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.AutoAlignHubConstants;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.utils.LimelightHelpers;

public class AlignToHubTagRelative extends Command {
  private PIDController xController, yController, rotController;
  private boolean isRightScore;
  private Timer  dontSeeTagTimer, stopTimer;
  private SwerveSubsystem swerveSubsystem;
  private double tagID = -1;

  public AlignToHubTagRelative(boolean isRightScore, SwerveSubsystem swerveSubsystem) {

    rotController = new PIDController(AutoAlignHubConstants.ROT_HUB_ALIGNMENT_P, 0, 0);  // Rotation
    
    this.isRightScore = isRightScore;
    this.swerveSubsystem = swerveSubsystem;
    addRequirements(swerveSubsystem);
  }

  @Override
  public void initialize() {
      this.stopTimer = new Timer();
      this.stopTimer.start();

      this.dontSeeTagTimer = new Timer();
      this.dontSeeTagTimer.start();

    rotController.setSetpoint(AutoAlignHubConstants.ROT_SETPOINT_HUB_ALIGNMENT);
    rotController.setTolerance(AutoAlignHubConstants.ROT_TOLERANCE_HUB_ALIGNMENT);


    tagID = LimelightHelpers.getFiducialID("limelight");
  }

  @Override
  public void execute() {

    if (LimelightHelpers.getTV("") && LimelightHelpers.getFiducialID("limelight") == tagID) {
      this.dontSeeTagTimer.reset();

      double[] postions = LimelightHelpers.getBotPose_TargetSpace("limelight");
      SmartDashboard.putNumber("x", postions[2]);


      double xSpeed = xController.calculate(postions[2]);
      SmartDashboard.putNumber("xspee", xSpeed);
      double ySpeed = -yController.calculate(postions[0]);
      double rotValue = -rotController.calculate(postions[4]);

      swerveSubsystem.drive(new Translation2d(xSpeed, ySpeed), rotValue, false);

      if (!rotController.atSetpoint() ||
          !yController.atSetpoint() ||
          !xController.atSetpoint()) {
          stopTimer.reset();
      }   

    } else {
      swerveSubsystem.drive(new Translation2d(), 0, false);
    }

    
    SmartDashboard.putNumber("poseValidTimer", stopTimer.get());
    SmartDashboard.putString("AlignTarget", isRightScore ? "Right Branch" : "Left Branch");
  }

  @Override
  public void end(boolean interrupted) {

    swerveSubsystem.drive(new Translation2d(), 0, false);
  }

  @Override
  public boolean isFinished() { 
    // Requires the robot to stay in the correct position for 0.3 seconds, as long as it gets a tag in the camera 
    return this.dontSeeTagTimer.hasElapsed(AutoAlignHubConstants.DONT_SEE_TAG_WAIT_TIME) ||
        stopTimer.hasElapsed(AutoAlignHubConstants.POSE_VALIDATION_TIME);
  }
}