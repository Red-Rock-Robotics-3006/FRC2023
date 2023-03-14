// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Robot extends TimedRobot {



  // Make options for autos and SendableChooser object
  private static final String kFirstAuto = "First Auto";
  private static final String kSecondAuto = "Second Auto";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();


  private final RobotContainer m_robotContainer = new RobotContainer();

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {


    // Set autos as options for the sendablechooser
    m_chooser.setDefaultOption("First Auto", kFirstAuto);
    m_chooser.addOption("Second Auto", kSecondAuto);
    SmartDashboard.putData("Auto choices", m_chooser);

    // Flush NetworkTables every loop. This ensures that robot pose and other values
    // are sent during every loop iteration.
    setNetworkTablesFlushEnabled(true);
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void autonomousInit() {

    m_autoSelected = m_chooser.getSelected();
    System.out.println("Auto selected: " + m_autoSelected);
    m_robotContainer.getAutonomousCommand().schedule();
  }

  @Override
  public void autonomousPeriodic() {
    switch (m_autoSelected) {
      case kSecondAuto:
      // Probably call to auto method?
      System.out.println("The first auto should run here");
      break;
      case kFirstAuto:
      default:
      // Auto code?
      System.out.println("The first auto should run here");
      break;

    }
  }

  @Override
  public void teleopInit() {
    m_robotContainer.enableControllers();
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
