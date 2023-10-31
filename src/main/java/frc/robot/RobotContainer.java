package frc.robot;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.Button;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Subsystems.*;
import frc.robot.Subsystems.Arm.Elevator;
import frc.robot.Subsystems.Arm.EndEffector;
import frc.robot.Subsystems.Arm.HorizontalExtender;
import frc.robot.Subsystems.Limelight.CAM_MODE;
import frc.robot.Commands.*;

@SuppressWarnings("unused")
public class RobotContainer {
  private final Joystick m_joystick1 = new Joystick(0);
  private final CommandXboxController mechStick = new CommandXboxController(1);

  private final Drivetrain m_swerve = Drivetrain.getInstance();
  private final Elevator m_elevator = new Elevator();
  private final HorizontalExtender m_extender = new HorizontalExtender();
  private final EndEffector m_effector = new EndEffector();

  // private SlewRateLimiter filterForAxis1 = new SlewRateLimiter(100); //Lower values to limit
  // private SlewRateLimiter filterForAxis2 = new SlewRateLimiter(100); //Lower values to limit
  // private SlewRateLimiter filterForRotation = new SlewRateLimiter(100); //Lower values to limit
  // //mess with parameter a bit to get desired output-flow??

  // Auto selection
  private Command[] autoCommands = new Command[]{
    new InstantCommand(() -> {
      this.m_swerve.drive(0, 0, 0, true);
    }),
    new InstantCommand(() -> {
      RunCommand dc = new RunCommand(
        () -> m_swerve.drive(
          0,-0.56,0,true
        ),
        m_swerve
      );
      dc.setName("Joystick Control");
  
      this.driveCommand = dc;
      m_swerve.setDefaultCommand(dc);
    }),
    new SequentialCommandGroup(
      new RunCommand(
        () -> m_swerve.setUniformDirection(new Rotation2d(0)),
        m_swerve
      ).withTimeout(0.15),
      new RunCommand(
        () -> m_swerve.drive(
          0,0.56,0,true
        ),
        m_swerve
      ).withTimeout(0.25),
      new RunCommand(
        () -> m_swerve.setUniformDirection(new Rotation2d(Math.PI)),
        m_swerve
      ).withTimeout(0.15),
      new RunCommand(
        () -> m_swerve.drive(
          0,-0.57,0,true
        ),
        m_swerve
      ).withTimeout(13)
    ),
    new SequentialCommandGroup(
      new InstantCommand(() -> {
        for(int i = 0; i < 10; i++) System.out.println("---");
        System.out.println(Timer.getFPGATimestamp());
        System.out.println("Start: " + m_elevator.getEncoderPos());
      }),
      new RunCommand(() -> {
        m_elevator.setSpeed(-0.25);
      }, m_elevator).withTimeout(0.75),
      new InstantCommand(() -> {
        m_elevator.setSpeed(0);
        for(int i = 0; i < 10; i++) System.out.println("---");
        System.out.println(Timer.getFPGATimestamp());
        System.out.println("End: " + m_elevator.getEncoderPos());
      })
    ),
    new SequentialCommandGroup(
      new InstantCommand(() -> {
        m_elevator.setTargetPos(0.35);
        m_elevator.enableHoming();
      }).withTimeout(2),
      new InstantCommand(() -> {
        m_elevator.disableHoming();
      })
    ),
    new SequentialCommandGroup(
      new RunCommand(() -> {
        m_effector.setSolenoid(false);
        m_elevator.setSpeed(-0.25);
      }, m_elevator).withTimeout(2),
      new InstantCommand(() -> {
        m_elevator.setSpeed(0);
      }),
      new RunCommand(() -> {
        m_extender.setSpeed(-0.3);
      }, m_extender).withTimeout(1),
      new InstantCommand(() -> {
        m_extender.setSpeed(0);
      }),
      new RunCommand(() -> {
        m_effector.setVelocityDelta(-0.1);
      }).withTimeout(0.5)
    )
  };
  private final SendableChooser<Command> m_chooser = new SendableChooser<>();
  
  private Command driveCommand = null;
  private boolean pneumaticsToggle = false;
  public RobotContainer() {
    PowerDistributionModule.getInstance();

    this.autoCommands[0].setName("Nothing");
    this.autoCommands[1].setName("Simple");
    this.autoCommands[2].setName("CubeDrop");
    this.autoCommands[3].setName("ElevatorEncodingTest");
    this.autoCommands[4].setName("ElevatorHomingTest");
    this.autoCommands[5].setName("MidScore");

    // Post to SmartDashboard 
    // https://docs.wpilib.org/en/stable/docs/software/dashboards/
    //  smartdashboard/displaying-status-of-commands-and-subsystems.html
    //SmartDashboard.putData(CommandScheduler.getInstance());
    //SmartDashboard.putData(m_swerve);
    //SmartDashboard.putData(m_elevator);
    //SmartDashboard.putData(m_extender);

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
    /*new JoystickButton(m_joystick1, 8)
        .onTrue(new InstantCommand(() -> {m_elevator.resetEncoder();}));*/
    new JoystickButton(m_joystick1, 11)
        .onTrue(new InstantCommand(() -> {
          RunCommand dc = new RunCommand(
            () -> m_swerve.drive(
              m_joystick1.getRawAxis(0)*4, 
              m_joystick1.getRawAxis(1)*4, 
              Math.pow(m_joystick1.getRawAxis(2),3)*2,
              false
            ),
            m_swerve
          );
          dc.setName("Joystick Control");

          this.driveCommand = dc;
          m_swerve.getDefaultCommand().cancel();
          m_swerve.setDefaultCommand(dc);
        }))
        .onFalse(new InstantCommand(() -> {
          RunCommand dc = new RunCommand(
            () -> m_swerve.drive(
              m_joystick1.getRawAxis(0)*4, 
              m_joystick1.getRawAxis(1)*4, 
              Math.pow(m_joystick1.getRawAxis(2),3)*100,
              true
            ),
            m_swerve
          );
          dc.setName("Joystick Control");

          this.driveCommand = dc;
          m_swerve.getDefaultCommand().cancel();
          m_swerve.setDefaultCommand(dc);
        }));
    /*mechStick.y()
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
      .onFalse(new InstantCommand(() -> {m_extender.setSpeed(0.0);}));*/

    mechStick.povRight()
        .onTrue(new InstantCommand(() -> {
          this.pneumaticsToggle = !pneumaticsToggle;
          m_effector.setSolenoid(this.pneumaticsToggle);
        }));
    mechStick.leftBumper()
        .onTrue(new InstantCommand(() -> {m_effector.setIntakeSpeed(1);}))
        .onFalse(new InstantCommand(() -> {m_effector.setIntakeSpeed(0);}));
    mechStick.rightBumper()
        .onTrue(new InstantCommand(() -> {m_effector.setIntakeSpeed(-1);}))
        .onFalse(new InstantCommand(() -> {m_effector.setIntakeSpeed(0);}));
    mechStick.povUp()
        .onTrue(new InstantCommand(() -> {m_effector.setVelocityDelta(0.5);;}))
        .onFalse(new InstantCommand(() -> {m_effector.setVelocityDelta(0);;}));
    mechStick.povDown()
        .onTrue(new InstantCommand(() -> {m_effector.setVelocityDelta(-0.4);;}))
        .onFalse(new InstantCommand(() -> {m_effector.setVelocityDelta(0);;}));
  }
  
  public void enableControllers() {
    if(m_swerve.getDefaultCommand() != null) m_swerve.getDefaultCommand().cancel();

    RunCommand dc = new RunCommand(
      () -> m_swerve.drive(
        m_joystick1.getRawAxis(0)*4, 
        m_joystick1.getRawAxis(1)*4, 
        Math.pow(m_joystick1.getRawAxis(2),3)*150,
        true
      ),
      m_swerve
    );
    dc.setName("Joystick Control");

    this.driveCommand = dc;
    m_swerve.setDefaultCommand(dc);

    RunCommand mce = new RunCommand(
      () -> {
        m_extender.setSpeed(0.5*mechStick.getRawAxis(4));
      },
      m_extender
    );
    mce.setName("Joystick Extend Control");

    m_extender.setDefaultCommand(mce);

    RunCommand mcu = new RunCommand(
      () -> {
        m_elevator.setSpeed(0.3*mechStick.getRawAxis(5));
      },
      m_elevator
    );
    mcu.setName("Joystick Elevator Control");

    m_elevator.setDefaultCommand(mcu);
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
    /*return new InstantCommand(() -> {
      RunCommand dc = new RunCommand(
        () -> m_swerve.drive(
          0,-0.56,0,true
        ),
        m_swerve
      );
      dc.setName("Joystick Control");
  
      this.driveCommand = dc;
      m_swerve.setDefaultCommand(dc);
    });*/
    return this.m_chooser.getSelected();
    //return new CSBalancing(false);
  }

  public void confirmMechanismsResting() {
    this.m_effector.setPosToStorred();
  }

  public void enableArmHoming() {
    this.m_effector.enableHoming();
  }
}
