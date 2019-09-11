
package com.team503.robot.subsystems;


import static com.ctre.phoenix.motorcontrol.ControlMode.*;


import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import java.util.Objects;
import java.util.function.DoubleConsumer;

/**
 * Controls a swerve drive wheel azimuth and drive motors.
 *
 * <p>The swerve-drive inverse kinematics algorithm will always calculate individual wheel angles as
 * -0.5 to 0.5 rotations, measured clockwise with zero being the straight-ahead position. Wheel
 * speed is calculated as 0 to 1 in the direction of the wheel angle.
 *
 * <p>This class will calculate how to implement this angle and drive direction optimally for the
 * azimuth and drive motors. In some cases it makes sense to reverse wheel direction to avoid
 * rotating the wheel azimuth 180 degrees.
 *
 * <p>Hardware assumed by this class includes a CTRE magnetic encoder on the azimuth motor and no
 * limits on wheel azimuth rotation. Azimuth Talons have an ID in the range 0-3 with corresponding
 * drive Talon IDs in the range 10-13.
 */
public class SwerveModule {
  private static final int TICKS = 1024;

  
  private final double driveSetpointMax;
  private final TalonSRX driveTalon;
  private final TalonSRX azimuthTalon;
  protected DoubleConsumer driver;
  private boolean isInverted = false;

  /**
   * This constructs a wheel with supplied azimuth and drive talons.
   *
   * <p>Wheels will scale closed-loop drive output to {@code driveSetpointMax}. For example, if
   * closed-loop drive mode is tuned to have a max usable output of 10,000 ticks per 100ms, set this
   * to 10,000 and the wheel will send a setpoint of 10,000 to the drive talon when wheel is set to
   * max drive output (1.0).
   *
   * @param azimuth the configured azimuth TalonSRX
   * @param drive the configured drive TalonSRX
   * @param driveSetpointMax scales closed-loop drive output to this value when drive setpoint = 1.0
   */
  public SwerveModule(TalonSRX azimuth, TalonSRX drive, double driveSetpointMax) {
    this.driveSetpointMax = driveSetpointMax;
    azimuthTalon = Objects.requireNonNull(azimuth);
    driveTalon = Objects.requireNonNull(drive);
  }

  /**
   * This method calculates the optimal driveTalon settings and applies them.
   *
   * @param drive 0 to 1.0 in the direction of the wheel azimuth
   * @param azimuth -180.0 to 180 degrees, measured clockwise with zero being the wheel's zeroed
   *     position
   * 
   */
  public void set(double drive, double azimuth) {
    // don't reset wheel azimuth direction to zero when returning to neutral
    azimuth = azimuth / 180.0;
    if (drive == 0) {
      driver.accept(0d);
      return;
    }

    azimuth *= -TICKS; // flip azimuth, hardware configuration dependent

    double azimuthPosition = azimuthTalon.getSelectedSensorPosition(0);
    double azimuthError = Math.IEEEremainder(azimuth - azimuthPosition, TICKS);

    // minimize azimuth rotation, reversing drive if necessary
    isInverted = Math.abs(azimuthError) > 0.25 * TICKS;
    if (isInverted) {
      azimuthError -= Math.copySign(0.5 * TICKS, azimuthError);
      drive = -drive;
    }

    azimuthTalon.set(MotionMagic, azimuthPosition + azimuthError);
    driver.accept(drive);
  }

  /**
   * Set azimuth to encoder position.
   *
   * @param position position in encoder ticks.
   */
  public void setAzimuthPosition(int position) {
    azimuthTalon.set(MotionMagic, position);
  }

  public void disableAzimuth() {
    azimuthTalon.neutralOutput();
  }

  
  /**
   * Stop azimuth and drive movement. This resets the azimuth setpoint and relative encoder to the
   * current position in case the wheel has been manually rotated away from its previous setpoint.
   */
  public void stop() {
    azimuthTalon.set(MotionMagic, azimuthTalon.getSelectedSensorPosition(0));
    driver.accept(0d);
  }

  /**
   * Set the azimuthTalon encoder relative to wheel zero alignment position. For example, if current
   * absolute encoder = 0 and zero setpoint = 2767, then current relative setpoint = -2767.
   *
   * <pre>
   *
   * relative:  -2767                               0
   *           ---|---------------------------------|-------
   * absolute:    0                               2767
   *
   * </pre>
   *
   * @param zero zero setpoint, absolute encoder position (in ticks) where wheel is zeroed.
   */
  public void setAzimuthZero(int zero) {
    int azimuthSetpoint = getAzimuthAbsolutePosition() - zero;
    ErrorCode err = azimuthTalon.setSelectedSensorPosition(azimuthSetpoint, 0, 10);
    azimuthTalon.set(MotionMagic, azimuthSetpoint);
  }

  /**
   * Returns the wheel's azimuth absolute position in encoder ticks.
   *
   * @return 0 - 4095, corresponding to one full revolution.
   */
  public int getAzimuthAbsolutePosition() {
    return azimuthTalon.getSensorCollection().getPulseWidthPosition() & 0xFFF;
  }

  /**
   * Get the azimuth Talon controller.
   *
   * @return azimuth Talon instance used by wheel
   */
  public TalonSRX getAzimuthTalon() {
    return azimuthTalon;
  }

  /**
   * Get the drive Talon controller.
   *
   * @return drive Talon instance used by wheel
   */
  public TalonSRX getDriveTalon() {
    return driveTalon;
  }

  public double getDriveSetpointMax() {
    return driveSetpointMax;
  }

  public boolean isInverted() {
    return isInverted;
  }

  @Override
  public String toString() {
    return "Wheel{"
        + "azimuthTalon="
        + azimuthTalon
        + ", driveTalon="
        + driveTalon
        + ", driveSetpointMax="
        + driveSetpointMax
        + '}';
  }
}