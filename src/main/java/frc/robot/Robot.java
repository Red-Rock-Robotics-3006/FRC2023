// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Subsystems.Gyroscope;
import frc.robot.Subsystems.Limelight;
import frc.robot.Subsystems.Limelight.CAM_MODE;
import frc.robot.Subsystems.Limelight.PIPELINE;

public class Robot extends TimedRobot {
  private final RobotContainer m_robotContainer = new RobotContainer();

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Flush NetworkTables every loop. This ensures that robot pose and other values
    // are sent during every loop iteration.
    for(int port = 5800; port <= 5805; port++)
      PortForwarder.add(port, "limelight.local", port); 
    setNetworkTablesFlushEnabled(true); 
    Limelight.getInstance().setCamMode(CAM_MODE.PROCESSING);
    Limelight.getInstance().setPipeline(PIPELINE.RANDOM);
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void autonomousInit() {
    CommandScheduler.getInstance().cancelAll();
    m_robotContainer.getAutonomousCommand().schedule();
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    CommandScheduler.getInstance().cancelAll();
    m_robotContainer.enableControllers();
    m_robotContainer.confirmMechanismsResting();
    m_robotContainer.enableArmHoming();
    //Limelight.getInstance().setCamMode(CAM_MODE.RAW);

    Limelight.getInstance().setCamMode(CAM_MODE.PROCESSING);
    Limelight.getInstance().setPipeline(PIPELINE.RANDOM);
  }

  @Override
  public void teleopPeriodic() {
    System.out.println(Gyroscope.getPigeonInstance().getYaw());
  }

  @Override
  public void disabledInit() {
    CommandScheduler.getInstance().cancelAll();
    m_robotContainer.zeroAllOutputs();
  }

  @Override
  public void autonomousExit() {
    CommandScheduler.getInstance().cancel(m_robotContainer.getAutonomousCommand());
  }

  @Override
  public void teleopExit() {
    m_robotContainer.disableControllers();
  }
}
