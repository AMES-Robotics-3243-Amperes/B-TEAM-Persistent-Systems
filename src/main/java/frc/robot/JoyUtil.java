package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.JoyUtilConstants;

/**
 * :3 {@link CommandXboxController} with many joystick tweaks in addition
 * to various other helper functions. All around a swell time :)
 *
 * <p>
 * Raw joystick output goes through deadzoning, curving, trigger
 * speed multiplier adjusting, then rate limiting. This means that
 * the rate limit is absolute and will always be obeyed no matter what.
 * </p>
 *
 * <p>
 * Curving is the following for input x, coefficients a and b,
 * and exponents n and k: output = a(x^n) + b(x^k)
 * </p>
 */
public class JoyUtil extends CommandXboxController {
  private final double exponent1, exponent2, coefficient1, coefficient2;

  // these are here so that triggers aren't being created every time a button is
  // read
  private final Trigger aTrigger = a();
  private final Trigger bTrigger = b();
  private final Trigger xTrigger = x();
  private final Trigger yTrigger = y();
  private final Trigger leftBumperTrigger = leftBumper();
  private final Trigger rightBumperTrigger = rightBumper();
  private final Trigger leftStickTrigger = leftStick();
  private final Trigger rightStickTrigger = rightStick();
  private final Trigger povLeftTrigger = povLeft();
  private final Trigger povRightTrigger = povRight();
  private final Trigger povUpTrigger = povUp();
  private final Trigger povDownTrigger = povDown();
  private final Trigger povUpLeftTrigger = povUpLeft();
  private final Trigger povUpRightTrigger = povUpRight();
  private final Trigger povDownLeftTrigger = povDownLeft();
  private final Trigger povDownRightTrigger = povDownRight();
  private final Trigger startTrigger = start();
  private final Trigger backTrigger = back();

  /**
   * :3 creates a new {@link JoyUtil} with the provided values
   *
   * @param port                             the assigned DriverStation port
   * @param deadzone                         the deadzone size for x and y axis of
   *                                         both joysticks
   * @param rateLimitLeft                    the max output change that can occur
   *                                         in one second for the left joystick
   * @param rateLimitRight                   the max output change that can occur
   *                                         in one second for the right joystick
   * @param exponent1                        the first exponent of the joystick
   *                                         curve
   * @param exponent2                        the second exponent of the joystick
   *                                         curve
   * @param coefficient1                     the coefficient applied to the first
   *                                         exponent of joystick curve
   * @param coefficient2                     the coefficient applied to the second
   *                                         exponent of joystick curve
   * @param leftTriggerLeftStickMultiplier   the amount the left joystick output
   *                                         is multiplied by if the left trigger
   *                                         is pressed fully
   * @param rightTriggerLeftStickMultiplier  the amount the left joystick output
   *                                         is multiplied by if the right
   *                                         trigger is pressed fully
   * @param leftTriggerRightStickMultiplier  the amount the right joystick output
   *                                         is multiplied by if the left
   *                                         trigger is pressed fully
   * @param rightTriggerRightStickMultiplier the amount the right joystick output
   *                                         is multiplied by if the right
   *                                         trigger is pressed fully
   */
  public JoyUtil(int port, double exponent1, double exponent2, double coefficient1, double coefficient2) {
    super(port);

    this.exponent1 = exponent1;
    this.exponent2 = exponent2;
    this.coefficient1 = coefficient1;
    this.coefficient2 = coefficient2;

    // even exponents make driving backwards inconvenient, so put
    // a one-time warning in the rio log if there are any
    if (exponent1 % 2 != 1 || exponent2 % 2 != 1) {
      DriverStation.reportError("Joystick curve exponents are not odd!", false);
    }

    // coefficients that don't add to one can cause unexpected behavior
    if (coefficient1 + coefficient2 != 1) {
      DriverStation.reportError("Joystick coefficients do not add to one!", false);
    }
  }

  /**
   * :3 creates a new {@link JoyUtil} with the values in constants
   *
   * @param port the port of the controller
   */
  public JoyUtil(int port) {
    this(port, JoyUtilConstants.exponent1, JoyUtilConstants.exponent2, JoyUtilConstants.coeff1, JoyUtilConstants.coeff2);
  }

  @Override
  public double getLeftX() {
    return composeJoystickFunctions(super.getLeftX());
  }

  @Override
  public double getRightX() {
    return composeJoystickFunctions(super.getRightX());
  }

  @Override
  public double getLeftY() {
    return composeJoystickFunctions(super.getLeftY());
  }

  @Override
  public double getRightY() {
    return composeJoystickFunctions(super.getRightY());
  }

  /**
   * :3 get value of A button as a boolean
   *
   * @return the value of the A button
   */
  public boolean getAButton() {
    return aTrigger.getAsBoolean();
  }

  /**
   * :3 get value of B button as a boolean
   *
   * @return the value of the B button
   */
  public boolean getBButton() {
    return bTrigger.getAsBoolean();
  }

  /**
   * :3 get value of X button as a boolean
   *
   * @return the value of the X button
   */
  public boolean getXButton() {
    return xTrigger.getAsBoolean();
  }

  /**
   * :3 get value of Y button as a boolean
   *
   * @return the value of the Y button
   */
  public boolean getYButton() {
    return yTrigger.getAsBoolean();
  }

  /**
   * :3 get value of left bumper as a boolean
   *
   * @return the value of the left bumper
   */
  public boolean getLeftBumper() {
    return leftBumperTrigger.getAsBoolean();
  }

  /**
   * :3 get value of right bumper as a boolean
   *
   * @return the value of the right bumper
   */
  public boolean getRightBumper() {
    return rightBumperTrigger.getAsBoolean();
  }

  /**
   * :3 get value of left stick pressed in as boolean
   *
   * @return the value of the left stick
   */
  public boolean getLeftStick() {
    return leftStickTrigger.getAsBoolean();
  }

  /**
   * :3 get value of right stick pressed in as boolean
   *
   * @return the value of the right stick
   */
  public boolean getRightStick() {
    return rightStickTrigger.getAsBoolean();
  }

  /**
   * :3 get value of start as boolean
   *
   * @return the value of the right stick
   */
  public boolean getStart() {
    return startTrigger.getAsBoolean();
  }

  /**
   * :3 get value of back as boolean
   *
   * @return the value of the right stick
   */
  public boolean getBack() {
    return backTrigger.getAsBoolean();
  }

  /**
   * :3 get if the d-pad is pointed left
   *
   * @return if the d-pad is facing left
   */
  public boolean getPOVLeft() {
    return povLeftTrigger.getAsBoolean();
  }

  /**
   * :3 get if the d-pad is pointed right
   *
   * @return if the d-pad is facing right
   */
  public boolean getPOVRight() {
    return povRightTrigger.getAsBoolean();
  }

  /**
   * :3 get if the d-pad is pointed up
   *
   * @return if the d-pad is facing up
   */
  public boolean getPOVUp() {
    return povUpTrigger.getAsBoolean();
  }

  /**
   * :3 get if the d-pad is pointed down
   *
   * @return if the d-pad is facing down
   */
  public boolean getPOVDown() {
    return povDownTrigger.getAsBoolean();
  }

  /**
   * :3 get if the d-pad is pointed up left
   *
   * @return if the d-pad is facing up left
   */
  public boolean getPOVUpLeft() {
    return povUpLeftTrigger.getAsBoolean();
  }

  /**
   * :3 get if the d-pad is pointed up right
   *
   * @return if the d-pad is facing up right
   */
  public boolean getPOVUpRight() {
    return povUpRightTrigger.getAsBoolean();
  }

  /**
   * :3 get if the d-pad is pointed down left
   *
   * @return if the d-pad is facing down left
   */
  public boolean getPOVDownLeft() {
    return povDownLeftTrigger.getAsBoolean();
  }

  /**
   * :3 get if the d-pad is pointed down right
   *
   * @return if the d-pad is facing down right
   */
  public boolean getPOVDownRight() {
    return povDownRightTrigger.getAsBoolean();
  }

  /**
   * :3 get if the d-pad isn't pressed
   *
   * @return if the d-pad isn't pressed
   */
  public boolean getPOVNotPressed() {
    return !getPOVLeft() && !getPOVUpLeft() && !getPOVUp() && !getPOVUpRight() && !getPOVRight() && !getPOVDownRight()
        && !getPOVDown() && !getPOVDownLeft();
  }

  /**
   * :3 curves a given input
   *
   * @param value the value before the curve
   * @return the value curved
   */
  private double applyCurve(double value) {
    double term1 = coefficient1 * Math.pow(value, exponent1);
    double term2 = coefficient2 * Math.pow(value, exponent2);

    return term1 + term2;
  }

  /**
   * :3 applies deadzoning, curve, and trigger multipliers to a raw input
   *
   * @param value                  the raw input from the joystick
   * @param leftTriggerMultiplier  the left trigger output multiplier for the axis
   *                               being calculated
   * @param rightTriggerMultiplier the right trigger output multiplier for the
   *                               axis being calculated
   * @return the raw input after being deadzoned, curved, and having trigger
   *         multipliers applied
   * @apiNote does not do any rate limiting
   */
  private double composeJoystickFunctions(double value) {
    double withCurve = applyCurve(value);

    return withCurve;
  }
}