package frc.robot.Subsystems;

import edu.wpi.first.wpilibj.PowerDistribution;

public class PowerDistributionModule {
  private static PowerDistribution instance;

  public static PowerDistribution getInstance() {
    if(instance==null) instance = new PowerDistribution();
    instance.setSwitchableChannel(true);
    return instance;
  }
}
