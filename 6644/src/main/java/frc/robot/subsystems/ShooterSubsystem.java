package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.Ultrasonic;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase {
  private final Spark m_flywheelMotor = new Spark(ShooterConstants.kFlywheelMotorPort);
  private final Spark m_indexerMotor = new Spark(ShooterConstants.kIndexerMotorPort);
  private final Spark m_intakeMotor = new Spark(ShooterConstants.kIntakeMotorPort);

  private final Encoder m_flywheelEncoder = new Encoder(ShooterConstants.kFlywheelEncoderPorts[0], ShooterConstants.kFlywheelEncoderPorts[1], ShooterConstants.kFlywheelEncoderReversed);
  
  private final Ultrasonic m_intakeUltrasonic = new Ultrasonic(ShooterConstants.kIntakeUltrasonicPorts[0], ShooterConstants.kIntakeUltrasonicPorts[1]);
  private NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
  NetworkTableEntry tv = table.getEntry("tv");
  NetworkTableEntry tx = table.getEntry("tx");
  NetworkTableEntry ty = table.getEntry("ty");
  
  /**
   * Creates a new ShooterSubsystem.
   */
  public ShooterSubsystem() {
    m_flywheelEncoder.setDistancePerPulse(ShooterConstants.kFlywheelEncoderCPR);
    m_flywheelEncoder.reset();

    m_intakeUltrasonic.setAutomaticMode(true);
  }

  /**
   * Sets the indexer motor's speed.
   * 
   * @param spd
   */
  public void setIndexer(double spd)
  {
    m_indexerMotor.set(spd);
  }

  /**
   * Sets the intake motor's speed.
   * 
   * @param spd
   */
  public void setIntake(double spd)
  {
    m_intakeMotor.set(spd);
  }

  /**
   * Sets the flywheel motor's speed.
   * 
   * @param spd
   */
  public void setFlywheel(double spd)
  {
    m_flywheelMotor.set(spd);
  }

  /**
   * Gets the flywheel encoder.
   *
   * @return the flywheel encoder
   */
  public Encoder getFlywheelEncoder() {
    return m_flywheelEncoder;
  }

  /**
   * Gets the intake's ultrasonic sensor.
   * 
   * @return the intake's ultrasonic sensor
   */
  public Ultrasonic getIntakeUltrasonic(){
    return m_intakeUltrasonic;
  }

  /**
   * Returns the flywheel's rpm (rate)
   */
  public double getFlywheelRPM()
  {
    return m_flywheelEncoder.getRate();
  }

  public double[] getLimelightTarget() {
    return new double[]{tv.getDouble(0), tx.getDouble(0), ty.getDouble(0)};
  }
}
