// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Arrays;
import java.util.Collections;

import com.ctre.phoenix.sensors.Pigeon2;


import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.settings.Constants;
import frc.robot.settings.LimelightValues;
import frc.robot.settings.Constants.DriveConstants;
public class DrivetrainSubsystem extends SubsystemBase {
  public SwerveDriveKinematics kinematics = Constants.DriveConstants.kinematics;
  private final com.ctre.phoenixpro.hardware.Pigeon2 pigeon = new com.ctre.phoenixpro.hardware.Pigeon2(Constants.DriveConstants.DRIVETRAIN_PIGEON_ID, Constants.DriveConstants.CANIVORE_DRIVETRAIN);
  private final SwerveModule[] modules;
  private final Rotation2d[] lastAngles;
  private final SwerveDrivePoseEstimator odometer;
  public final Field2d m_Field2d = new Field2d();
  /** Creates a new DrivetrainSubsystem. */
  public DrivetrainSubsystem() {
    Preferences.initString("FL", "FL");
    Preferences.initString("FR", "FR");
    Preferences.initString("BL", "BL");
    Preferences.initString("BR", "BR");
    SmartDashboard.putData("Field", m_Field2d);
    //SmartDashboard.putData("resetOdometry", new InstantCommand(() -> this.resetOdometry()));
    modules = new SwerveModule[4];
    lastAngles = new Rotation2d[] {new Rotation2d(), new Rotation2d(), new Rotation2d(), new Rotation2d()};
    modules[0] = new SwerveModule(
      "FL",
      Constants.DriveConstants.FL_DRIVE_MOTOR_ID,
      Constants.DriveConstants.FL_STEER_MOTOR_ID,
      Constants.DriveConstants.FL_STEER_ENCODER_ID,
      Constants.DriveConstants.FL_STEER_OFFSET,
      Constants.DriveConstants.CANIVORE_DRIVETRAIN
    );
    modules[1] = new SwerveModule(
      "FR",
      Constants.DriveConstants.FR_DRIVE_MOTOR_ID,
      Constants.DriveConstants.FR_STEER_MOTOR_ID,
      Constants.DriveConstants.FR_STEER_ENCODER_ID,
      Constants.DriveConstants.FR_STEER_OFFSET,
      Constants.DriveConstants.CANIVORE_DRIVETRAIN
    );
    modules[2] = new SwerveModule(
      "BL",
      Constants.DriveConstants.BL_DRIVE_MOTOR_ID,
      Constants.DriveConstants.BL_STEER_MOTOR_ID,
      Constants.DriveConstants.BL_STEER_ENCODER_ID,
      Constants.DriveConstants.BL_STEER_OFFSET,
      Constants.DriveConstants.CANIVORE_DRIVETRAIN
    );
    modules[3] = new SwerveModule(
      "BR",
      Constants.DriveConstants.BR_DRIVE_MOTOR_ID,
      Constants.DriveConstants.BR_STEER_MOTOR_ID,
      Constants.DriveConstants.BR_STEER_ENCODER_ID,
      Constants.DriveConstants.BR_STEER_OFFSET,
      Constants.DriveConstants.CANIVORE_DRIVETRAIN
    );
odometer = new SwerveDrivePoseEstimator(kinematics, getGyroscopeRotation(), getModulePositions(), Constants.DriveConstants.DRIVE_ODOMETRY_ORIGIN);
  }
public void zeroGyroscope(){
  pigeon.setYaw(0.0);
  odometer.resetPosition(new Rotation2d(), getModulePositions(), new Pose2d());
}
public void zeroGyroscope(double angleDeg) {
  pigeon.setYaw(angleDeg);
  new Rotation2d();
  new Rotation2d();
  odometer.resetPosition(Rotation2d.fromDegrees(angleDeg), getModulePositions(), new Pose2d(getPose().getTranslation(), Rotation2d.fromDegrees(angleDeg)));
}
public Rotation2d getGyroscopeRotation() {
  return pigeon.getRotation2d();
}
public SwerveModulePosition[] getModulePositions() {
  SwerveModulePosition[] positions = new SwerveModulePosition[4];
  for (int i = 0; i < 4; i++) positions[i] = modules[i].getPosition();
  return positions;
}
public SwerveModuleState[] getModuleState() {
  SwerveModuleState[] states = new SwerveModuleState[4];
  for (int i = 0; i < 4; i++) states[i] = modules[i].getState();
  return states;
}
public Pose2d getPose() {
return odometer.getEstimatedPosition();
}
public void resetOdometry(Pose2d pose) {
zeroGyroscope(pose.getRotation().getDegrees());
odometer.resetPosition(getGyroscopeRotation(), getModulePositions(), Constants.DriveConstants.DRIVE_ODOMETRY_ORIGIN);
}
public void resetOdometry() {
  odometer.resetPosition(getGyroscopeRotation(), getModulePositions(), Constants.DriveConstants.DRIVE_ODOMETRY_ORIGIN);
}
public void pointWheelsForward() {
for (int i = 0; i < 4; i++){
  setModule(i, new SwerveModuleState(0, new Rotation2d()));
}
}
public void pointWheelsInward() {
  setModule(0, new SwerveModuleState(0, Rotation2d.fromDegrees(-135)));
  setModule(1, new SwerveModuleState(0, Rotation2d.fromDegrees(135)));
  setModule(2, new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
  setModule(3, new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
}
public void drive(ChassisSpeeds chassisSpeeds) {
  SwerveModuleState[] desiredStates = kinematics.toSwerveModuleStates(chassisSpeeds);
  double maxSpeed = Collections.max(Arrays.asList(desiredStates)).speedMetersPerSecond;
  if (maxSpeed <= Constants.DriveConstants.DRIVE_DEADBAND_MPS){
    for (int i = 0; i < 4; i++){
      stop();
    }
  } else {
    setModuleStates(desiredStates);
  }
  SmartDashboard.putNumber("Checkpoint 2", desiredStates[0].speedMetersPerSecond);
}
public void stop() {
  for (int i = 0; i < 4; i++) {
    modules[i].setDesiredState(new SwerveModuleState(0, lastAngles[i]));
  }
}
public void setModuleStates(SwerveModuleState[] desiredStates) {
  SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DriveConstants.MAX_VELOCITY_METERS_PER_SECOND);
  for (int i = 0; i < 4; i++) {
    setModule(i, desiredStates[i]);;
  } 
}
public void setModule(int i, SwerveModuleState desiredState) {
  modules [i].setDesiredState(desiredState);
  lastAngles[i] = desiredState.angle;
}
public void updateOdometry() {
  odometer.updateWithTime(Timer.getFPGATimestamp(), getGyroscopeRotation(), getModulePositions());
}
public void updateOdometryWithVision(Pose2d estimatedPose, double timeStampSeconds){
  odometer.addVisionMeasurement(estimatedPose, timeStampSeconds);
}
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    updateOdometry();
     if (SmartDashboard.getBoolean("use limelight", false)){
       LimelightValues visionData = new LimelightValues(LimelightHelpers.getLatestResults("").targetingResults, LimelightHelpers.getTV(""));
       Boolean isVisionValid = visionData.isResultValid;
       Boolean isVisionTrustworthy = isVisionValid && visionData.isPoseTrustworthy(odometer.getEstimatedPosition());
       SmartDashboard.putBoolean("visionValid", isVisionTrustworthy);
       if (isVisionTrustworthy || ((SmartDashboard.getBoolean("trust limelight", false)) && isVisionValid)) {
         updateOdometryWithVision(visionData.getBotPose(), visionData.gettimestamp());
       }
     }
    m_Field2d.setRobotPose(odometer.getEstimatedPosition());
    SmartDashboard.putNumber("Robot Angle", getGyroscopeRotation().getDegrees());
    SmartDashboard.putString("Robot Location", getPose().getTranslation().toString());
  }
}
