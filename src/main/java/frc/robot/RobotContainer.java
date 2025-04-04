// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.photonvision.PhotonCamera;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.commands.CommandSwerveGetOffset;
import frc.robot.commands.CommandSwerveTeleopDrive;
import frc.robot.commands.CommandSwerveXWheels;
import frc.robot.commands.GetCameraOffset;
import frc.robot.commands.automatics.TaxiCommand;
import frc.robot.subsystems.SubsystemLeds;
import frc.robot.subsystems.swerve.SubsystemSwerveDrivetrain;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // H! Main shuffleboard tab
  ShuffleboardTab mainTab = Shuffleboard.getTab("Main");

  // H! Auto Selector
  AutoSelector autoSelector = new AutoSelector(mainTab);

  // controllers
  private JoyUtil primaryController = new JoyUtil(0);
  private JoyUtil secondaryController = new JoyUtil(1);

  //
  // Subsystems
  //

  public SubsystemSwerveDrivetrain subsystemSwerveDrivetrain = new SubsystemSwerveDrivetrain();
  public SubsystemLeds subsystemLeds = new SubsystemLeds();

  //
  // Commands
  //

  private CommandSwerveTeleopDrive commandSwerveTeleopDrive = new CommandSwerveTeleopDrive(subsystemSwerveDrivetrain,
      primaryController);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // we construct the DataManager instance here since it is the
    // absolute soonest we have access to a RobotContainer object
    new DataManager(this);

    // set sensible default commands
    setDefaultCommands();
    mainTab.add(subsystemSwerveDrivetrain);

    // H! Set all commands in auto selector
    setAutoCommands();

    // configure the controller bindings
    configureBindings();
  }

  /**
   * Used to set default commands for subsystems.
   */
  private void setDefaultCommands() {
    subsystemSwerveDrivetrain.setDefaultCommand(commandSwerveTeleopDrive);
  }

  private void setAutoCommands() {
    autoSelector.addDefault(new InstantCommand(), "None");
    autoSelector.add(new TaxiCommand(subsystemSwerveDrivetrain, 0.5), "Small Taxi");

  }

  /**
   * Used to configure controller bindings.
   * Do not remove any of the commented out code. Most of it is commented for
   * testing purposes.
   */
  private void configureBindings() {

    // Manual intaking/depositing, elevator movement, reef setpoints

    primaryController.b().onTrue(Commands.runOnce(commandSwerveTeleopDrive::toggleFieldRelative));
    primaryController.a().whileTrue(new CommandSwerveXWheels(subsystemSwerveDrivetrain));

    
    // reef automatics
    
    // debug info
    secondaryController.povUp()
        .onTrue(
            new GetCameraOffset(new PhotonCamera("FrontLeftCamera"),
                new Transform3d(new Pose3d(),
                    new Pose3d(1, 0, Units.inchesToMeters(11.8),
                        new Rotation3d(Rotation2d.fromDegrees(180))))));
    secondaryController.povDown().onTrue(new CommandSwerveGetOffset(subsystemSwerveDrivetrain));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoSelector.get();
    // return ScoreIntakeAutoCommandBuilder.buildAuto(
    // FieldConstants.AutonomousPaths.testingPositions,
    // subsystemClaw, subsystemElevator,
    // subsystemSwerveDrivetrain, FieldConstants.AutonomousPaths.testingSetpoints);
    // return new SequentialCommandGroup(new
    // CommandSwerveTaxi(subsystemSwerveDrivetrain),
    // ScoreIntakeAutoCommandBuilder.buildAuto(
    // FieldConstants.AutonomousPaths.testingPositions,
    // subsystemClaw, subsystemElevator,
    // subsystemSwerveDrivetrain, FieldConstants.AutonomousPaths.testingSetpoints));
  }
}
