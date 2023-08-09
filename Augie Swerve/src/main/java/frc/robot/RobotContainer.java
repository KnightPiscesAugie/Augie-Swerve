// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import static frc.robot.settings.Constants.PS4Driver.DEADBAND_LARGE;
import static frc.robot.settings.Constants.PS4Driver.DEADBAND_NORMAL;
import static frc.robot.settings.Constants.PS4Driver.NO_INPUT;
import static frc.robot.settings.Constants.PS4Driver.X_AXIS;
import static frc.robot.settings.Constants.PS4Driver.Y_AXIS;
import static frc.robot.settings.Constants.PS4Driver.Z_AXIS;
import static frc.robot.settings.Constants.PS4Driver.Z_ROTATE;

import java.util.HashMap;
import java.util.Map;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.Autos;
import frc.robot.commands.Drive;
import frc.robot.settings.Constants;
import frc.robot.subsystems.DrivetrainSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private DrivetrainSubsystem drivetrain;
  private Drive defaultDriveCommand;
  private SendableChooser<Command> autoChooser;
  private final PS4Controller driveController;
  private Autos autos; 
  public static HashMap<String, Command> eventMap;
  private Field2d Field;
  // Replace with CommandPS4Controller or CommandJoystick if needed
  

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    driveController = new PS4Controller(0);
    autoChooser = new SendableChooser<>();
    eventMap = new HashMap<>();
    drivetrain = new DrivetrainSubsystem();

    defaultDriveCommand = new Drive(
      drivetrain, 
    () -> driveController.getL1Button(), 
    () -> modifyAxis(-driveController.getRawAxis(Y_AXIS), DEADBAND_NORMAL), 
    () -> modifyAxis(-driveController.getRawAxis(X_AXIS), DEADBAND_NORMAL), 
    () -> modifyAxis(-driveController.getRawAxis(Z_AXIS), DEADBAND_NORMAL));
    drivetrain.setDefaultCommand(defaultDriveCommand);
    SmartDashboard.putBoolean("use limelight", false);
    SmartDashboard.putBoolean("trust limelight", false);
    autoInit();
    // Configure the trigger bindings
    configureBindings();
  }
  private void autoInit(){
    autos = Autos.getInstance();
    eventMap.put("Marker 1", new PrintCommand("Passed marker 1"));
    eventMap.put("Marker 2", new PrintCommand("Passed marker 2"));
    eventMap.put("stop", new InstantCommand (drivetrain::stop, drivetrain));
    //autos.autoInit(autoChooser, eventMap, drivetrain);
    SmartDashboard.putData(autoChooser);
    Field2d Field  = drivetrain.m_Field2d;
    SmartDashboard.putData("Field", Field);
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    new Trigger(driveController::getPSButton).onTrue(Commands.runOnce(drivetrain::zeroGyroscope, drivetrain));
   
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return autoChooser.getSelected();
  }
  private double modifyAxis(double value, double deadband) {
    value = MathUtil.applyDeadband(value, deadband);
    value = Math.copySign(value*value, value);
    return value;
  }
  public void robotInit(){
    drivetrain.zeroGyroscope();
  }
  public void teleopInit() {
  }

  public void teleopPeriodic() {
  }
}
