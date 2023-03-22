package frc.robot.Subsystems; 
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PneumaticsTest extends SubsystemBase{

  
    Solenoid testSolenoid = new Solenoid(33,PneumaticsModuleType.CTREPCM, 1);


    boolean solenoidToggle = false;

    public PneumaticsTest() {
      //pcmCompressor.enableDigital();
      //pcmCompressor.disable();
    }
   
    @Override
    public void periodic() {
      
    }
  
    @Override
    public void simulationPeriodic() {
      
    }
    public void toggleSolenoid(){
        solenoidToggle = !solenoidToggle;
        testSolenoid.set(solenoidToggle);
    }
}
