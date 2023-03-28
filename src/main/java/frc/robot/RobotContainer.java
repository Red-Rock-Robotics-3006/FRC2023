package frc.robot;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Subsystems.*;
import frc.robot.Subsystems.Arm.Elevator;
import frc.robot.Subsystems.Arm.EndEffector;
import frc.robot.Subsystems.Arm.HorizontalExtender;
import frc.robot.Commands.*;

@SuppressWarnings("unused")
public class RobotContainer {
  private final Joystick m_joystick1 = new Joystick(0);
  private final CommandXboxController mechStick = new CommandXboxController(1);

  private final Drivetrain m_swerve = Drivetrain.getInstance();
  private final Elevator m_elevator = new Elevator();
  private final HorizontalExtender m_extender = new HorizontalExtender();
  private final EndEffector m_effector = new EndEffector();

  private SlewRateLimiter filterForAxis1 = new SlewRateLimiter(100); //Lower values to limit
  private SlewRateLimiter filterForAxis2 = new SlewRateLimiter(100); //Lower values to limit
  private SlewRateLimiter filterForRotation = new SlewRateLimiter(100); //Lower values to limit
  //mess with parameter a bit to get desired output-flow??

  // Auto selection
  private Command[] autoCommands = new Command[]{};
  private final SendableChooser<Command> m_chooser = new SendableChooser<>();
  
  private Command driveCommand = null;

  public RobotContainer() {
    // Post to SmartDashboard 
    // https://docs.wpilib.org/en/stable/docs/software/dashboards/
    //  smartdashboard/displaying-status-of-commands-and-subsystems.html
    SmartDashboard.putData(CommandScheduler.getInstance());
    SmartDashboard.putData(m_swerve);
    SmartDashboard.putData(m_elevator);
    SmartDashboard.putData(m_extender);

    // Auto selection setup
    if(this.autoCommands.length != 0) m_chooser.setDefaultOption(this.autoCommands[0].getName(), this.autoCommands[0]);
    for(int i = 1; i < this.autoCommands.length; i++) {
      m_chooser.addOption(this.autoCommands[i].getName(), this.autoCommands[i]);
    }
    SmartDashboard.putData("Auto choices", m_chooser);

    configureButtonBindings();
  }
  public void configureButtonBindings()
  {
    new JoystickButton(m_joystick1, 7)
        .onTrue(new InstantCommand(() -> {Gyroscope.getPigeonInstance().setYaw(0);}));
    mechStick.y()
      .onTrue(new InstantCommand(() -> {m_elevator.setSpeed(-0.2);}))
      .onFalse(new InstantCommand(() -> {m_elevator.setSpeed(0.0);}));
    mechStick.a()
      .onTrue(new InstantCommand(() -> {m_elevator.setSpeed(0.2);}))
      .onFalse(new InstantCommand(() -> {m_elevator.setSpeed(0.0);}));
    mechStick.x()
      .onTrue(new InstantCommand(() -> {m_extender.setSpeed(0.35);}))
      .onFalse(new InstantCommand(() -> {m_extender.setSpeed(0.0);}));
    mechStick.b()
      .onTrue(new InstantCommand(() -> {m_extender.setSpeed(-0.2);}))
      .onFalse(new InstantCommand(() -> {m_extender.setSpeed(0.0);}));
      mechStick.leftBumper()
        .onTrue(new InstantCommand(() -> {m_effector.setSolenoid(true);}))
        .onFalse(new InstantCommand(() -> {m_effector.setSolenoid(false);}));
    mechStick.povRight()
        .onTrue(new InstantCommand(() -> {m_effector.setIntakeSpeed(0.8);}))
        .onFalse(new InstantCommand(() -> {m_effector.setIntakeSpeed(0);}));
    mechStick.povLeft()
        .onTrue(new InstantCommand(() -> {m_effector.setIntakeSpeed(-0.8);}))
        .onFalse(new InstantCommand(() -> {m_effector.setIntakeSpeed(0);}));
    mechStick.povUp()
        .onTrue(new InstantCommand(() -> {m_effector.setArmSpeed(1);}))
        .onFalse(new InstantCommand(() -> {m_effector.setArmSpeed(0);}));
    mechStick.povDown()
        .onTrue(new InstantCommand(() -> {m_effector.setArmSpeed(-1);}))
        .onFalse(new InstantCommand(() -> {m_effector.setArmSpeed(0);}));
  }
  
  public void enableControllers() {
    if(m_swerve.getDefaultCommand() != null) m_swerve.getDefaultCommand().cancel();

    RunCommand dc = new RunCommand(
      () -> m_swerve.drive(
        filterForAxis1.calculate(m_joystick1.getRawAxis(0)*4), 
        filterForAxis2.calculate(m_joystick1.getRawAxis(1)*4), 
        filterForRotation.calculate(Math.pow(m_joystick1.getRawAxis(2),3)*2)*100,
        true
      ),
      m_swerve
    );
    dc.setName("Joystick Control");

    this.driveCommand = dc;
    m_swerve.setDefaultCommand(dc);
  }
  public void disableControllers() {
    if(this.driveCommand == null && this.driveCommand.getName().equals("Joystick Control")) {
      this.driveCommand.cancel();
      this.driveCommand = null;
    }
  }

  public void zeroAllOutputs() {
    if(m_swerve.getDefaultCommand() != null) m_swerve.getDefaultCommand().cancel();

    RunCommand dc = new RunCommand(
      () -> m_swerve.zeroWheels(),
      m_swerve
    );
    dc.setName("Zeroing Control");

    this.driveCommand = dc;
    m_swerve.setDefaultCommand(dc);
  }

  public Command getAutonomousCommand() {
    return this.m_chooser.getSelected();
  }
}
