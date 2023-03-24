// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.ctre.phoenix.sensors.Pigeon2;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** Represents a swerve drive style drivetrain. */
@SuppressWarnings("unused")
public class Drivetrain extends SubsystemBase {
  private static Drivetrain instance;

  public static final double kMaxSpeed = 5.0;
  public static final double kMaxAngularSpeed = 2*Math.PI;
  public static final double kModuleMaxSpeed = 5.0;

  private final Translation2d m_frontLeftLocation = new Translation2d(-0.381, -0.381);
  private final Translation2d m_frontRightLocation = new Translation2d(0.381, -0.381);
  private final Translation2d m_backLeftLocation = new Translation2d(-0.381, 0.381);
  private final Translation2d m_backRightLocation = new Translation2d(0.381, 0.381);

  private final SwerveModule m_frontRight = new SwerveModule(20, 6, 0, false); //m_frontLeft
  private final SwerveModule m_backRight = new SwerveModule(11, 8, 41, true); //m_frontRight
  private final SwerveModule m_frontLeft = new SwerveModule(4, 5, 44, false); //m_backLeft
  private final SwerveModule m_backLeft = new SwerveModule(50, 3, 43, true); //m_backRight

  private final Pigeon2 m_gyro = Gyroscope.getPigeonInstance();

  private final Field2d m_fieldMap = new Field2d();
  
  private Pose2d startPos;

  private Translation2d centerOfRotation = new Translation2d(0,0);

  private final SwerveDriveKinematics m_kinematics =
      new SwerveDriveKinematics(
          m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation);

  private final SwerveDriveOdometry m_odometry =
      new SwerveDriveOdometry(
        m_kinematics, 
        new Rotation2d(), 
        new SwerveModulePosition[]{
          new SwerveModulePosition(),
          new SwerveModulePosition(),
          new SwerveModulePosition(),
          new SwerveModulePosition()
        });
      //new SwerveDriveOdometry(m_kinematics, new Rotation2d(-2*Math.PI*m_gyro.getYaw()/360d));//getRotation2d());

  private Drivetrain(Pose2d startingPose) {
    this.startPos = startingPose;
    m_gyro.setYaw(startingPose.getRotation().getDegrees());

    //The Field2d class allows robot visualization in the simulation GUI.
    SmartDashboard.putData("Field", m_fieldMap);
  }

  public static Drivetrain getInstance() {
    if(instance==null) instance = new Drivetrain(new Pose2d());
    return instance;
  }

  public void overrideCurrentPos(Pose2d pos) {
    this.startPos = pos;
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed Speed of the robot in the x direction (forward).
   * @param ySpeed Speed of the robot in the y direction (sideways).
   * @param rot Angular rate of the robot in degrees.
   * @param fieldRelative Whether the provided x and y speeds are relative to the field.
   */
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    if (Math.abs(xSpeed) + Math.abs(ySpeed) + Math.abs(rot) > 0.1) {
      var swerveModuleStates =
          m_kinematics.toSwerveModuleStates(
              fieldRelative
                  ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, 2*Math.PI*(rot/360), new Rotation2d(-2*Math.PI*m_gyro.getYaw()/360d)) //High Risk Change!
                  : new ChassisSpeeds(xSpeed, ySpeed, 2*Math.PI*(rot/360)),this.centerOfRotation);
      SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, kMaxSpeed); //Look into overloaded method with more parameters
      
      m_frontLeft.setDesiredState(swerveModuleStates[0]);
      m_frontRight.setDesiredState(swerveModuleStates[1]);
      m_backLeft.setDesiredState(swerveModuleStates[2]);
      m_backRight.setDesiredState(swerveModuleStates[3]);
    } else {
      m_frontLeft.zeroPower();
      m_frontRight.zeroPower();
      m_backLeft.zeroPower();
      m_backRight.zeroPower();
    }
  }

  public SwerveModuleState[] getModuleStates() {
    return new SwerveModuleState[]{ m_frontLeft.getState(), m_frontRight.getState(), m_backLeft.getState(), m_backRight.getState() };
  }

  public void setUniformDirection(Rotation2d rot) {
    m_frontLeft.setDesiredState(new SwerveModuleState(0, rot));
    m_frontRight.setDesiredState(new SwerveModuleState(0, rot));
    m_backLeft.setDesiredState(new SwerveModuleState(0, rot));
    m_backRight.setDesiredState(new SwerveModuleState(0, rot));
  }

  public void zeroWheels() {
    m_frontLeft.zeroModule();
    m_frontRight.zeroModule();
    m_backLeft.zeroModule();
    m_backRight.zeroModule();
  }

  public void setCenterOfRotation(Translation2d center)
  {
    this.centerOfRotation = center;
  }

  /** Updates the field relative position of the robot. */
  public void updateOdometry() {
    m_odometry.update(
        new Rotation2d(-2*Math.PI*m_gyro.getYaw()/360),
        new SwerveModulePosition[]{
          m_frontLeft.getPos(),
          m_frontRight.getPos(),
          m_backLeft.getPos(),
          m_backRight.getPos()
        });
    m_fieldMap.setRobotPose(m_odometry.getPoseMeters());
  }
  
  @Override
  public void periodic() {
    updateOdometry();
  }

  @Override 
  public void simulationPeriodic() {}
}