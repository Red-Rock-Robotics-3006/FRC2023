// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.ctre.phoenix.sensors.Pigeon2;
import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.SwerveAutoBuilder;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.math.controller.PIDController;
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
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.function.Consumer;

/** Represents a swerve drive style drivetrain. */
@SuppressWarnings("unused")
public class Drivetrain extends SubsystemBase {
  public static final double kMaxSpeed = 5.0;
  public static final double kMaxAngularSpeed = 2*Math.PI;
  public static final double kModuleMaxSpeed = 5.0;

  private final Translation2d m_frontLeftLocation = new Translation2d(-0.381, 0.381);
  private final Translation2d m_frontRightLocation = new Translation2d(-0.381, -0.381);
  private final Translation2d m_backLeftLocation = new Translation2d(0.381, 0.381);
  private final Translation2d m_backRightLocation = new Translation2d(0.381, -0.381);

  private final SwerveModule m_frontLeft = new SwerveModule(20, 6, 0, false);
  private final SwerveModule m_frontRight = new SwerveModule(11, 8, 41, true);
  private final SwerveModule m_backLeft = new SwerveModule(4, 5, 44, false);
  private final SwerveModule m_backRight = new SwerveModule(50, 3, 43, true);

  private final Pigeon2 m_gyro = new Pigeon2(45);

  private final Field2d m_fieldMap = new Field2d();
  
  private final Pose2d lastPos;

  
  // Fix pid controllers
  public Command followTrajectoryCommand(PathPlannerTrajectory traj, boolean isFirstPath) {
    return new SequentialCommandGroup(
        new InstantCommand(() -> {

            if(isFirstPath) 
              // Set odometry to initial holonomic pose
              this.resetOdometry(traj);
        }),
        new PPSwerveControllerCommand(
            traj,
            m_odometry::getPoseMeters,
            m_kinematics,
            new PIDController(1.0, 1.0, 1.0),
            new PIDController(1.0, 1.0, 1.0),
            new PIDController(1.0, 1.0, 1.0),
            this::setModuleStates,
            true,
            this
        )
    );
}


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

  public Drivetrain(Pose2d startingPose) {
    this.lastPos = startingPose;
    m_gyro.setYaw(startingPose.getRotation().getDegrees());

    //The Field2d class allows robot visualization in the simulation GUI.
    SmartDashboard.putData("Field", m_fieldMap);
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
                  : new ChassisSpeeds(xSpeed, ySpeed, 2*Math.PI*(rot/360)));
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

  public void setModuleStates(SwerveModuleState[] states){
    m_frontLeft.setDesiredState(states[0]);
    m_frontRight.setDesiredState(states[1]);
    m_backLeft.setDesiredState(states[2]);
    m_backRight.setDesiredState(states[3]);
  }


  public void resetOdometry(PathPlannerTrajectory traj){
    m_odometry.update(
      new Rotation2d(-2*Math.PI*m_gyro.getYaw()/360),
      new SwerveModulePosition[]{
        m_frontLeft.getPos(),
        m_frontRight.getPos(),
        m_backLeft.getPos(),
        m_backRight.getPos()
      });
    m_fieldMap.setRobotPose(traj.getInitialHolonomicPose());
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

  //Private helper methods
  private Pose2d getDeltaPos() {
    return this.m_odometry.getPoseMeters().relativeTo(this.lastPos);
  }

  public SwerveDriveOdometry getOdometry()
  {
    return m_odometry;
  }

  public SwerveDriveKinematics getKinematics()
  {
    return m_kinematics;
  }

  public void resetPose(Pose2d lastPos)
  {
    m_odometry.resetPosition(
      new Rotation2d(-2*Math.PI*m_gyro.getYaw()/360),
      new SwerveModulePosition[]{
        m_frontLeft.getPos(),
        m_frontRight.getPos(),
        m_backLeft.getPos(),
        m_backRight.getPos()
      },
      lastPos
    );
  }
}