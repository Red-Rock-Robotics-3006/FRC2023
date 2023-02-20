// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.CANCoder;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

@SuppressWarnings("unused")
public class SwerveModule extends SubsystemBase {
  private static final double kWheelRadius = 0.0508;
  private static final int kFalconEncoderResolution = 2048;
  private static final double kDriveGearRatio = 6.75; //?

  private SwerveModuleState targetState;

  private final WPI_TalonFX m_driveMotor;
  private final WPI_TalonFX m_turningMotor;
  private final CANCoder m_cCoder;

  private int inversion = 1;

  /**
   * Constructs a SwerveModule with a drive motor, turning motor, drive encoder and turning encoder.
   *
   * @param driveMotorChannel PWM output for the drive motor.
   * @param turningMotorChannel PWM output for the turning motor.
   */
  public SwerveModule(
      int driveMotorChannel,
      int turningMotorChannel,
      int cCoderChannel,
      boolean inverted) {

    this.targetState = new SwerveModuleState();

    //Create Motor Objects
    this.m_driveMotor = new WPI_TalonFX(driveMotorChannel);
    this.m_turningMotor = new WPI_TalonFX(turningMotorChannel);
    this.m_cCoder = new CANCoder(cCoderChannel);

    //Configure Motors
    this.m_driveMotor.configFactoryDefault();
    this.m_driveMotor.setInverted(inverted);
    this.m_driveMotor.setNeutralMode(NeutralMode.Brake);
    this.m_turningMotor.configFactoryDefault();
    this.m_turningMotor.setInverted(false);
    this.m_turningMotor.setNeutralMode(NeutralMode.Brake);

    //Configure Rotational Encoder
    this.m_turningMotor.getSensorCollection().setIntegratedSensorPosition(
      360 * this.m_cCoder.getPosition() / (2 * Math.PI), 
      0);
  }
  
  public void zeroModule() {
    setDesiredState(new SwerveModuleState(0d, new Rotation2d(0)));
  }

  public void zeroPower() {
    setDesiredState(new SwerveModuleState(0d, this.targetState.angle));
  }

  /**
   * Sets the desired state for the module.
   *
   * @param desiredState Desired state with speed and angle.
   */
  public void setDesiredState(SwerveModuleState desiredState) {

    double targetAngle = desiredState.angle.getDegrees();
    targetAngle = Math.IEEEremainder(targetAngle, 360);
    if (targetAngle < 0) targetAngle += 360;

    SwerveModuleState state = new SwerveModuleState(
      desiredState.speedMetersPerSecond, 
      new Rotation2d(
        2*Math.PI*targetAngle/360
      ));
    
      SmartDashboard.putNumber("Raw Target Angle" + m_turningMotor.getBaseID(), targetAngle/360d);

    SmartDashboard.putNumber("Sent Angle" + m_turningMotor.getBaseID(), targetAngle/360);
    SmartDashboard.putNumber("Min Travel" + m_turningMotor.getBaseID(), shortestAngleDist(this.m_cCoder.getAbsolutePosition(), targetAngle)/360);

    /*if (shortestAngleDist(this.m_cCoder.getAbsolutePosition(), targetAngle) > 90) {
      state = new SwerveModuleState(
        -desiredState.speedMetersPerSecond, 
        new Rotation2d(2*Math.PI*Math.IEEEremainder(targetAngle+180, 360)/360d)
        );
      SmartDashboard.putNumber("Sent Angle" + m_turningMotor.getBaseID(), Math.IEEEremainder(targetAngle+180, 360)/360d);
      SmartDashboard.putNumber("Optimization Mode" + m_turningMotor.getBaseID(), 1);
    } else {
      SmartDashboard.putNumber("Optimization Mode" + m_turningMotor.getBaseID(), 0);
    }*/
    
    this.targetState = state;
  }


  @Override
  public void periodic() {
    double targetAngle = -this.targetState.angle.getDegrees();
    targetAngle = Math.IEEEremainder(targetAngle, 360);
    if (targetAngle < 0) {
      targetAngle = 360 + targetAngle;
    }
    
    double linearControl;
    if (
      Math.abs(m_cCoder.getAbsolutePosition() - targetAngle) < 
      360 - Math.abs(m_cCoder.getAbsolutePosition() - targetAngle)
    ) {
      linearControl = 0.85*(m_cCoder.getAbsolutePosition() - targetAngle)/360d;
      SmartDashboard.putNumber("Turning Mode" + m_turningMotor.getBaseID(), 0);
    } else {
      linearControl = -0.85*(360 - Math.abs(m_cCoder.getAbsolutePosition() - targetAngle))/360d;
      linearControl *= Math.signum(m_cCoder.getAbsolutePosition() - targetAngle);
      SmartDashboard.putNumber("Turning Mode" + m_turningMotor.getBaseID(), 1);
    }

    final double turnOutput = Math.signum(linearControl) * Math.pow(
      Math.abs(linearControl),
      1d/1.5
    );

    m_turningMotor.set(ControlMode.PercentOutput, turnOutput);
    m_driveMotor.set(ControlMode.PercentOutput, 0.5*this.targetState.speedMetersPerSecond);

    SmartDashboard.putNumber("Target Angle: " + m_turningMotor.getBaseID(), targetAngle/360);
    SmartDashboard.putNumber("Motor Power: " + m_turningMotor.getBaseID(), turnOutput);
    SmartDashboard.putNumber("Motor Rotation: " + m_turningMotor.getBaseID(), m_cCoder.getAbsolutePosition()/360);
  }

  private double shortestAngleDist(double input, double target) {
    double norm_input = Math.IEEEremainder(input, 360); //205
    double norm_target = Math.IEEEremainder(target, 360); //68

    if (norm_input < 0) norm_input += 360; //205
    if (norm_target < 0) norm_target += 360; //68

    return Math.min(
      Math.abs(norm_input - norm_target), //abs(137)
      Math.abs(360 - Math.abs(norm_input - norm_target)) //abs(360 +210)
      );
  }

  //Public Info Methods
  /**
   * Returns the current state of the module.
   *
   * @return The current state of the module.
   */
  public SwerveModuleState getState() {
    return new SwerveModuleState(
      rawDriveUnitsToMeters(this.m_driveMotor.getSelectedSensorVelocity()), 
      new Rotation2d(m_cCoder.getAbsolutePosition())
    );
  }
  /**
   * Returns the current position of the module.
   *
   * @return The current position of the module.
   */
  public SwerveModulePosition getPos() {
    return new SwerveModulePosition(
      rawDriveUnitsToMeters(this.m_driveMotor.getSelectedSensorPosition()), 
      new Rotation2d(m_cCoder.getAbsolutePosition())
    );
  }

  //Private Helper Methods
  private double rawDriveUnitsToMeters(double units) {
    return (units / kFalconEncoderResolution) * kDriveGearRatio * (2*Math.PI*kWheelRadius);
  }
  private double metersToRawDriveUnits(double meters) {
    return ((meters / (2*Math.PI*kWheelRadius)) / kDriveGearRatio) * kFalconEncoderResolution;
  }
}
