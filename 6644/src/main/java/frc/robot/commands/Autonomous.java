package frc.robot.commands;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.Constants.ShooterConstants;

public class Autonomous extends SequentialCommandGroup {
  /**
   * Create a new autonomous command.
   */
  public Autonomous(DriveSubsystem drive, ShooterSubsystem shooter) {
    addCommands(
      new AimLimelight(drive, shooter),
      new ParallelCommandGroup(
        new PIDCommand(
          new PIDController(ShooterConstants.kFlywheelP, ShooterConstants.kFlywheelP, ShooterConstants.kFlywheelD),
          shooter::getFlywheelRPM,
          ShooterConstants.kFlywheelTargetRPM,
          output -> shooter.setFlywheel(output),
          shooter),
        
        new SequentialCommandGroup(
          new WaitCommand(4),
          new ParallelDeadlineGroup(
            new WaitCommand(1),
            new RunCommand(() -> shooter.setIndexer(0.7), shooter)
          ),
          new WaitCommand(0.5),
          new ParallelDeadlineGroup(
            new WaitCommand(1),
            new RunCommand(() -> shooter.setIndexer(0.7), shooter)
          ),
          new WaitCommand(0.5),
          new ParallelDeadlineGroup(
            new WaitCommand(1),
            new RunCommand(() -> shooter.setIndexer(0.7), shooter)
          ),
          new WaitCommand(0.5),
          new ParallelDeadlineGroup(
            new WaitCommand(1),
            new RunCommand(() -> shooter.setIndexer(0.7), shooter)
          ),
          new WaitCommand(0.5),
          new ParallelDeadlineGroup(
            new WaitCommand(1),
            new RunCommand(() -> shooter.setIndexer(0.7), shooter)
          )
        )
      )
    );
  }
}
/*
new AimLimelight(drive, shooter),
      new ParallelCommandGroup(
        new PIDCommand(
          new PIDController(ShooterConstants.kFlywheelP, ShooterConstants.kFlywheelP, ShooterConstants.kFlywheelD),
          m_shooter::getFlywheelRPM,
          ShooterConstants.kFlywheelTargetRPM,
          output -> m_shooter.setFlywheel(output),
          m_shooter),
        
        new SequentialCommandGroup(
          new WaitCommand(3),
          new ParallelDeadlineGroup(
            new WaitCommand(1),
            new RunCommand(() -> m_shooter.setIndexer(0.7), m_shooter)
          ),
          new ParallelDeadlineGroup(
            new WaitCommand(1),
            new RunCommand(() -> m_shooter.setIndexer(0.7), m_shooter)
          ),
          new ParallelDeadlineGroup(
            new WaitCommand(1),
            new RunCommand(() -> m_shooter.setIndexer(0.7), m_shooter)
          ),
          new ParallelDeadlineGroup(
            new WaitCommand(1),
            new RunCommand(() -> m_shooter.setIndexer(0.7), m_shooter)
          ),new ParallelDeadlineGroup(
            new WaitCommand(1),
            new RunCommand(() -> m_shooter.setIndexer(0.7), m_shooter)
          )
        )
      )
       */