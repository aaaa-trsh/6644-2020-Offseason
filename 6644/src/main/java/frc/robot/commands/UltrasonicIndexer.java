package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShooterSubsystem;

/**
 * A command that enables the indexer when the ultrasonic detects a ball
 */
public class UltrasonicIndexer extends CommandBase {
  private ShooterSubsystem m_shooter;

  /**
   * Enables the indexer sensor when a ball is detected.
   * @param shooter The shooter subsystem to use
   */
  public UltrasonicIndexer(ShooterSubsystem shooter) {
    m_shooter = shooter;
  }

  @Override
  public void execute() {
    if(m_shooter.getIntakeUltrasonic().getRangeInches() < 7) {
      m_shooter.setIndexer(0.4);
    } else {
      m_shooter.setIndexer(0);
    }
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}