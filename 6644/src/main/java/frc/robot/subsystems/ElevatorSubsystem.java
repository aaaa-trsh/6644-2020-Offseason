package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.ElevatorConstants;

public class ElevatorSubsystem extends SubsystemBase {
  private final Spark m_elevatorMotor1 = new Spark(ElevatorConstants.kElevatorMotor1Port);
  private final Spark m_elevatorMotor2 = new Spark(ElevatorConstants.kElevatorMotor2Port);
  private final Solenoid m_elevatorSolenoid1 = new Solenoid(ElevatorConstants.kElevatorSolenoid1Port);
  private final Solenoid m_elevatorSolenoid2 = new Solenoid(ElevatorConstants.kElevatorSolenoid2Port);

  /**
   * Creates a new ElevatorSubsystem.
   */
  public ElevatorSubsystem() {
    
  }

  /**
   * Sets the elevator solenoid's states.
   * 
   * @param on The state to set the solenoid
   */
  public void setElevatorSolenoid(boolean on) {
    m_elevatorSolenoid1.set(on);
    m_elevatorSolenoid2.set(on);
  }

  /**
   * Sets the elevator motor's speed. 
   * 
   * @param spd The speed to set the elevator
   */
  public void setElevatorSpeed(double spd) {
    m_elevatorMotor1.set(spd);
    m_elevatorMotor2.set(spd);
  }
}
