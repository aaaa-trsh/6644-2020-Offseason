package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.commands.AimLimelight;
import frc.robot.commands.Autonomous;
import frc.robot.commands.UltrasonicIndexer;
import frc.robot.subsystems.*;

/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();
  private final ShooterSubsystem m_shooter = new ShooterSubsystem();
  private final ElevatorSubsystem m_elevator = new ElevatorSubsystem();

  // Autonomous
  private final CommandBase m_autonomousCommand = new Autonomous(m_robotDrive, m_shooter);

  // The driver's controller
  Joystick m_driverController = new Joystick(OIConstants.kDriverControllerPort);

  /**
   * The container for the robot.  Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Put some buttons on SmartDashboard
    SmartDashboard.putData("Aim Limelight", new AimLimelight(m_robotDrive, m_shooter));
    SmartDashboard.putData("Wrist Horizontal", new AimLimelight(m_robotDrive, m_shooter));
    
    // Configure default commands
    // Set the default drive command to arcade drive
    m_robotDrive.setDefaultCommand(
        new RunCommand(() -> m_robotDrive.arcadeDrive(m_driverController.getY(), m_driverController.getX()), m_robotDrive));
    
    // Sets the default shooter command ultrasonic indexer (not sure if working yet)
    m_shooter.setDefaultCommand(new UltrasonicIndexer(m_shooter));
    
    // Put the subsystems on SmartDashboard
    SmartDashboard.putData(m_robotDrive);
    SmartDashboard.putData(m_shooter);
    SmartDashboard.putData(m_elevator);

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Maps commands to buttons.
   */ 
  private void configureButtonBindings() {
    // Stabilize robot to drive straight with gyro when joystick trigger is held
    new JoystickButton(m_driverController, 0)
        .whenHeld(new PIDCommand(
            new PIDController(DriveConstants.kStabilizationP, DriveConstants.kStabilizationI, DriveConstants.kStabilizationD),
            m_robotDrive::getTurnRate,
            0,
            output -> m_robotDrive.arcadeDrive(m_driverController.getY(), output),
            m_robotDrive));

    // Drive at half speed when button 1 is held
    new JoystickButton(m_driverController, 1)
        .whenPressed(() -> m_robotDrive.setMaxOutput(0.5))
        .whenReleased(() -> m_robotDrive.setMaxOutput(1));

    // Aim, then use flywheel when button 2 is held
    new JoystickButton(m_driverController, 2)
        .whenPressed(new AimLimelight(m_robotDrive, m_shooter))
        .whenHeld(new PIDCommand(
                      new PIDController(ShooterConstants.kFlywheelP, ShooterConstants.kFlywheelP, ShooterConstants.kFlywheelD),
                      m_shooter::getFlywheelRPM,
                      ShooterConstants.kFlywheelTargetRPM,
                      output -> m_shooter.setFlywheel(output),
                      m_shooter))
        .whenReleased(new RunCommand(() -> m_shooter.setFlywheel(0), m_shooter));

    // Use intake when button 3 is held
    new JoystickButton(m_driverController, 3)
        .whenPressed(new RunCommand(() -> m_shooter.setIntake(1), m_shooter))
        .whenReleased(new RunCommand(() -> m_shooter.setIntake(0), m_shooter));

    // Use indexer when button 4 is held (manual alternative to UltrasonicIndexer)
    new JoystickButton(m_driverController, 4)
        .whenPressed(new RunCommand(() -> m_shooter.setIndexer(1), m_shooter))
        .whenReleased(new RunCommand(() -> m_shooter.setIndexer(0), m_shooter));
    
    // Do elevator up sequence when button 5 is held (solenoid on, motor up)
    new JoystickButton(m_driverController, 5)
        .whenHeld(new ParallelCommandGroup(new InstantCommand(() -> m_elevator.setElevatorSolenoid(true)), new RunCommand(() -> m_elevator.setElevatorSpeed(1))))
        .whenReleased(new RunCommand(() -> m_elevator.setElevatorSpeed(0), m_elevator));

    // Do elevator down sequence when button 6 is held (solenoid on, motor up)
    new JoystickButton(m_driverController, 6)
        .whenHeld(new RunCommand(() -> m_elevator.setElevatorSpeed(-0.5f), m_elevator))
        .whenReleased(new RunCommand(() -> m_elevator.setElevatorSpeed(0), m_elevator));
  }

  /**
   * The autonomous command.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return m_autonomousCommand;
  }
}