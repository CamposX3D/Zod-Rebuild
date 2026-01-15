// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.ToogleShootCommand;
import frc.robot.constants.GamepadConstants;
import frc.robot.constants.ButtonsConstants.Buttons;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import swervelib.SwerveInputStream;

public class RobotContainer {

  private final File swerveConfigFile = new File(Filesystem.getDeployDirectory(), "swerve");
  private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem(swerveConfigFile);
  
  private final SendableChooser<Command> autoChooser;

  private final ShooterSubsystem shooter = new ShooterSubsystem();

  private XboxController p2Controller = new XboxController(GamepadConstants.P2_PORT);
  private XboxController p1Controller = new XboxController(GamepadConstants.P1_PORT);


  SwerveInputStream driveOriented = SwerveInputStream
      .of(swerveSubsystem.getSwerveDrive(),
          () -> -reversibleLeftY(),
          () -> -reversibleLeftX())
      .withControllerRotationAxis(() -> -p1Controller.getRightX())
      .deadband(GamepadConstants.DEADBAND)
      .scaleTranslation(0.7)
      .scaleRotation(0.7)
      .allianceRelativeControl(false);

  SwerveInputStream driveOrientedInverted = SwerveInputStream
      .of(swerveSubsystem.getSwerveDrive(),
          () -> -reversibleLeftY(),
          () -> -reversibleLeftX())
      .withControllerRotationAxis(() -> p1Controller.getRightX())
      .deadband(GamepadConstants.DEADBAND)
      .scaleTranslation(0.7)
      .scaleRotation(0.7)
      .allianceRelativeControl(false);

  SwerveInputStream driveOrientedLow = SwerveInputStream
      .of(swerveSubsystem.getSwerveDrive(),
          () -> -reversibleLeftY(),
          () -> -reversibleLeftX())
      .withControllerRotationAxis(() -> -p1Controller.getRightX())
      .deadband(GamepadConstants.DEADBAND)
      .scaleTranslation(0.2)
      .scaleRotation(0.2)
      .allianceRelativeControl(false);


  public RobotContainer() {
    configureBindings();
    setDefaultCommands();
    registerAutoCommands();

    autoChooser = AutoBuilder.buildAutoChooser("");
    SmartDashboard.putData("Auto Select", autoChooser);
  }

  private void setDefaultCommands() {}
  private void registerAutoCommands() {}

  private void configureBindings() {

    Command drive = swerveSubsystem.driveFieldOriented(driveOriented);
    Command driveLow = swerveSubsystem.driveFieldOriented(driveOrientedLow);
    swerveSubsystem.setDefaultCommand(drive);

    new JoystickButton(p1Controller, XboxController.Button.kRightBumper.value).toggleOnTrue(driveLow);

    new JoystickButton(p1Controller, XboxController.Button.kA.value)
        .onTrue(Commands.runOnce(() -> swerveSubsystem.resetGyro()));

    new JoystickButton(p1Controller, XboxController.Button.kY.value)
        .toggleOnTrue(Commands.startEnd(
            swerveSubsystem::disableHeading, swerveSubsystem::resetHeading));


    new JoystickButton(p2Controller, Buttons.BUTTON_A).whileTrue(new ToogleShootCommand(shooter));
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }

  public void setHeadingCorrection(boolean setHeadingCorrection) {
    swerveSubsystem.getSwerveDrive().setHeadingCorrection(setHeadingCorrection);
  }

  public SwerveSubsystem getSwerveSubsystem() {
    return swerveSubsystem;
  }

  public double reversibleLeftX() {
    double manualRevertMultiplier = 1;
    if(p1Controller.getLeftTriggerAxis() > 0.3){
      manualRevertMultiplier = -1;
    }
    if(DriverStation.getAlliance().isPresent()){
      if(DriverStation.getAlliance().get() == Alliance.Red) {
        return -p1Controller.getLeftX() * manualRevertMultiplier;
      }else{
        return p1Controller.getLeftX() * manualRevertMultiplier;
      }
    }else{
      return -p1Controller.getLeftX() * manualRevertMultiplier;
    }
  }  

  public double reversibleLeftY() {
    double manualRevertMultiplier = 1;
    if(p1Controller.getLeftTriggerAxis() > 0.3){
      manualRevertMultiplier = -1;
    }
    if (DriverStation.getAlliance().isPresent())  {
      if (DriverStation.getAlliance().get() == Alliance.Red) {
        return -p1Controller.getLeftY() * manualRevertMultiplier;
      } else {
        return p1Controller.getLeftY() * manualRevertMultiplier;
      }
    } else {
      return p1Controller.getLeftY() * manualRevertMultiplier;
    }
  }
}
