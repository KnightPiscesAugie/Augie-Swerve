// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.HashMap;
import java.util.List;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.settings.Constants.DriveConstants;
import frc.robot.subsystems.DrivetrainSubsystem;

public final class Autos {
  private DrivetrainSubsystem drivetrain;
  public static SwerveAutoBuilder autoBuilder;
  private HashMap<String, Command> eventMap;
  /** Example static factory for an autonomous command. */
  private static Autos autos;
  
  private Autos() {
    
  }
  public static Autos getInstance() {
    if (autos == null) {
        autos = new Autos();
    }
    return autos;
}
public void autoInit(SendableChooser<Command> autoChooser, HashMap<String, Command> eventMap, DrivetrainSubsystem drivetrain){
  this.drivetrain = drivetrain;
  this.eventMap = eventMap;
  Autos.autoBuilder = new SwerveAutoBuilder(drivetrain::getPose, drivetrain::resetOdometry, drivetrain.kinematics, new PIDConstants(
    DriveConstants.k_XY_P,
    DriveConstants.k_XY_I,
    DriveConstants.k_XY_D), new PIDConstants(
      DriveConstants.k_THETA_P,
      DriveConstants.k_THETA_I,
      DriveConstants.k_THETA_D), drivetrain::setModuleStates, eventMap, true, drivetrain);
      //autoChooser.addOption(phAuto, null);
}
static List<PathPlannerTrajectory> driveForward = PathPlanner.loadPathGroup("driveForward", new PathConstraints(2.5, 1.75));
}
