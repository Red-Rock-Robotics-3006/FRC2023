package frc.robot;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.Subsystems.Drivetrain;

@SuppressWarnings("unused")
public class RobotContainer {
    private final Joystick m_joystick1 = new Joystick(0);
    private final Joystick m_joystick2 = new Joystick(1);
    private final Drivetrain m_swerve = new Drivetrain(new Pose2d());

    private SlewRateLimiter filterForAxis1 = new SlewRateLimiter(0.5); 
    private SlewRateLimiter filterForAxis2 = new SlewRateLimiter(0.5);
    private SlewRateLimiter filterForRotation = new SlewRateLimiter(0.5);
    //mess with parameter a bit to get desired output-flow??

    //Auto selection
    private Command[] autoCommands = new Command[]{};
    private final SendableChooser<Command> m_chooser = new SendableChooser<>();
    
    private Command driveCommand = null;

    public RobotContainer() {
        //Auto selection setup
        m_chooser.setDefaultOption(this.autoCommands[0].getName(), this.autoCommands[0]);
        for(int i = 1; i < this.autoCommands.length; i++) {
            m_chooser.addOption(this.autoCommands[i].getName(), this.autoCommands[i]);
        }
        SmartDashboard.putData("Auto choices", m_chooser);
    }

    public void enableControllers() {
        if(m_swerve.getDefaultCommand() != null) m_swerve.getDefaultCommand().cancel();

        RunCommand dc = new RunCommand(
            () -> m_swerve.drive(
                filterForAxis1.calculate(m_joystick1.getRawAxis(0)), 
                filterForAxis2.calculate(m_joystick1.getRawAxis(1)), 
                filterForRotation.calculate(m_joystick1.getRawAxis(2)*100),
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
