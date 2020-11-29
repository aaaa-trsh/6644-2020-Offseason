package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class AimLimelight extends CommandBase {
  private DriveSubsystem m_drive; 
  private ShooterSubsystem m_shooter; 

  private double m_errorX = 0;
  private double m_errorY = 0;
  private boolean m_validTarget;

  /**
   * Creates a new AimLimelight.
   */
  public AimLimelight(DriveSubsystem drive, ShooterSubsystem shooter) {
    m_drive = drive;
    m_shooter = shooter;
  }

  @Override
  public void execute() {
    var target = m_shooter.getLimelightTarget();
    m_validTarget = target[0] == 1;
    m_errorX = -target[1];
    m_errorY = -target[2];

    double steering_adjust = 0;
    steering_adjust = ShooterConstants.kAimXP * m_errorX + (ShooterConstants.kMinAimGain * target[1] > 1 ? -1 : target[1] < 1 ? 1 : 0);

    double distance_adjust = ShooterConstants.kAimYP * m_errorY;
    m_drive.arcadeDrive(distance_adjust, steering_adjust);
  }

  @Override
  public void end(boolean interrupted) {
    m_drive.arcadeDrive(0, 0);
  }

  @Override
  public boolean isFinished() {
    return !m_validTarget || Math.abs(m_errorX) > ShooterConstants.kAimTolerance || Math.abs(m_errorY) > ShooterConstants.kAimTolerance;
  }
}
