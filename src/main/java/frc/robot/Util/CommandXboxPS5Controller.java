// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Util;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PS5Controller;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.event.BooleanEvent;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * A version of {@link XboxController} with {@link Trigger} factories for command-based.
 *
 * @see XboxController
 */
@SuppressWarnings("MethodName")
public class CommandXboxPS5Controller extends CommandGenericHID {
  private XboxController m_hidx = null;
  private PS5Controller m_hidp = null;
  private Boolean xbox;

  /**
   * Construct an instance of a controller.
   *
   * @param port The port index on the Driver Station that the controller is plugged into.
   */
  public CommandXboxPS5Controller(int port) {
    super(port);
    if (DriverStation.getJoystickName(port).contains("Wireless")) {
      xbox = false;
      m_hidp = new PS5Controller(port);
    } else {
      xbox = true;
      m_hidx = new XboxController(port);
    }

  }

  /**
   * Get the underlying GenericHID object.
   *
   * @return the wrapped GenericHID object
   */
  @Override
  public XboxController getHID() {
    return m_hidx;
  }

  /**
   * Constructs an event instance around the left bumper's digital signal.
   *
   * @return an event instance representing the left bumper's digital signal attached to the {@link
   *     CommandScheduler#getDefaultButtonLoop() default scheduler button loop}.
   * @see #leftBumper(EventLoop)
   */
  public Trigger leftBumper() {
      return leftBumper(CommandScheduler.getInstance().getDefaultButtonLoop());
  }

  /**
   * Constructs an event instance around the left bumper's digital signal.
   *
   * @param loop the event loop instance to attach the event to.
   * @return an event instance representing the right bumper's digital signal attached to the given
   *     loop.
   */
  public Trigger leftBumper(EventLoop loop) {
    if (xbox) {
      return m_hidx.leftBumper(loop).castTo(Trigger::new);
    } else {
      return m_hidp.L1(loop).castTo(Trigger::new);
    }
  }

  /**
   * Constructs an event instance around the right bumper's digital signal.
   *
   * @return an event instance representing the right bumper's digital signal attached to the {@link
   *     CommandScheduler#getDefaultButtonLoop() default scheduler button loop}.
   * @see #rightBumper(EventLoop)
   */
  public Trigger rightBumper() {
    return rightBumper(CommandScheduler.getInstance().getDefaultButtonLoop());
  }

  /**
   * Constructs an event instance around the right bumper's digital signal.
   *
   * @param loop the event loop instance to attach the event to.
   * @return an event instance representing the left bumper's digital signal attached to the given
   *     loop.
   */
  public Trigger rightBumper(EventLoop loop) {
    if (xbox) {
      return m_hidx.rightBumper(loop).castTo(Trigger::new);
    } else {
      return m_hidp.R1(loop).castTo(Trigger::new);
    }
  }

  /**
   * Constructs an event instance around the left stick button's digital signal.
   *
   * @return an event instance representing the left stick button's digital signal attached to the
   *     {@link CommandScheduler#getDefaultButtonLoop() default scheduler button loop}.
   * @see #leftStick(EventLoop)
   */
  public Trigger leftStick() {
    return leftStick(CommandScheduler.getInstance().getDefaultButtonLoop());
  }

  /**
   * Constructs an event instance around the left stick button's digital signal.
   *
   * @param loop the event loop instance to attach the event to.
   * @return an event instance representing the left stick button's digital signal attached to the
   *     given loop.
   */
  public Trigger leftStick(EventLoop loop) {
    if (xbox) {
      return m_hidx.leftStick(loop).castTo(Trigger::new);
    } else {
      return m_hidp.L3(loop).castTo(Trigger::new);
    }
    
  }

  /**
   * Constructs an event instance around the right stick button's digital signal.
   *
   * @return an event instance representing the right stick button's digital signal attached to the
   *     {@link CommandScheduler#getDefaultButtonLoop() default scheduler button loop}.
   * @see #rightStick(EventLoop)
   */
  public Trigger rightStick() {
    return rightStick(CommandScheduler.getInstance().getDefaultButtonLoop());
  }

  /**
   * Constructs an event instance around the right stick button's digital signal.
   *
   * @param loop the event loop instance to attach the event to.
   * @return an event instance representing the right stick button's digital signal attached to the
   *     given loop.
   */
  public Trigger rightStick(EventLoop loop) {
    if (xbox) {
      return m_hidx.rightStick(loop).castTo(Trigger::new);
    } else {
      return m_hidp.R3(loop).castTo(Trigger::new);
    }
  }

  /**
   * Constructs an event instance around the A button's digital signal.
   *
   * @return an event instance representing the A button's digital signal attached to the {@link
   *     CommandScheduler#getDefaultButtonLoop() default scheduler button loop}.
   * @see #a(EventLoop)
   */
  public Trigger a() {
    return a(CommandScheduler.getInstance().getDefaultButtonLoop());
  }

  /**
   * Constructs an event instance around the A button's digital signal.
   *
   * @param loop the event loop instance to attach the event to.
   * @return an event instance representing the A button's digital signal attached to the given
   *     loop.
   */
  public Trigger a(EventLoop loop) {
    if (xbox) {
      return m_hidx.a(loop).castTo(Trigger::new);
    } else {
      return m_hidp.cross(loop).castTo(Trigger::new);
    }
  }

  /**
   * Constructs an event instance around the B button's digital signal.
   *
   * @return an event instance representing the B button's digital signal attached to the {@link
   *     CommandScheduler#getDefaultButtonLoop() default scheduler button loop}.
   * @see #b(EventLoop)
   */
  public Trigger b() {
    return b(CommandScheduler.getInstance().getDefaultButtonLoop());
  }

  /**
   * Constructs an event instance around the B button's digital signal.
   *
   * @param loop the event loop instance to attach the event to.
   * @return an event instance representing the B button's digital signal attached to the given
   *     loop.
   */
  public Trigger b(EventLoop loop) {
    if (xbox) {
      return m_hidx.b(loop).castTo(Trigger::new);
    } else {
      return m_hidp.circle(loop).castTo(Trigger::new);
    }
  }

  /**
   * Constructs an event instance around the X button's digital signal.
   *
   * @return an event instance representing the X button's digital signal attached to the {@link
   *     CommandScheduler#getDefaultButtonLoop() default scheduler button loop}.
   * @see #x(EventLoop)
   */
  public Trigger x() {
    return x(CommandScheduler.getInstance().getDefaultButtonLoop());
  }

  /**
   * Constructs an event instance around the X button's digital signal.
   *
   * @param loop the event loop instance to attach the event to.
   * @return an event instance representing the X button's digital signal attached to the given
   *     loop.
   */
  public Trigger x(EventLoop loop) {
    if (xbox) {
      return m_hidx.x(loop).castTo(Trigger::new);
    } else {
      return m_hidp.square(loop).castTo(Trigger::new);
    }
  }

  /**
   * Constructs an event instance around the Y button's digital signal.
   *
   * @return an event instance representing the Y button's digital signal attached to the {@link
   *     CommandScheduler#getDefaultButtonLoop() default scheduler button loop}.
   * @see #y(EventLoop)
   */
  public Trigger y() {
    return y(CommandScheduler.getInstance().getDefaultButtonLoop());
  }

  /**
   * Constructs an event instance around the Y button's digital signal.
   *
   * @param loop the event loop instance to attach the event to.
   * @return an event instance representing the Y button's digital signal attached to the given
   *     loop.
   */
  public Trigger y(EventLoop loop) {
    if (xbox) {
      return m_hidx.y(loop).castTo(Trigger::new);
    } else {
      return m_hidp.triangle(loop).castTo(Trigger::new);
    }
  }

  /**
   * Constructs an event instance around the start button's digital signal.
   *
   * @return an event instance representing the start button's digital signal attached to the {@link
   *     CommandScheduler#getDefaultButtonLoop() default scheduler button loop}.
   * @see #start(EventLoop)
   */
  public Trigger start() {
    return start(CommandScheduler.getInstance().getDefaultButtonLoop());
  }

  /**
   * Constructs an event instance around the start button's digital signal.
   *
   * @param loop the event loop instance to attach the event to.
   * @return an event instance representing the start button's digital signal attached to the given
   *     loop.
   */
  public Trigger start(EventLoop loop) {
    if (xbox) {
      return m_hidx.start(loop).castTo(Trigger::new);
    } else {
      return m_hidp.options(loop).castTo(Trigger::new);
    }
  }

  /**
   * Constructs an event instance around the back button's digital signal.
   *
   * @return an event instance representing the back button's digital signal attached to the {@link
   *     CommandScheduler#getDefaultButtonLoop() default scheduler button loop}.
   * @see #back(EventLoop)
   */
  public Trigger back() {
    return back(CommandScheduler.getInstance().getDefaultButtonLoop());
  }

  /**
   * Constructs an event instance around the back button's digital signal.
   *
   * @param loop the event loop instance to attach the event to.
   * @return an event instance representing the back button's digital signal attached to the given
   *     loop.
   */
  public Trigger back(EventLoop loop) {
    if (xbox) {
      return m_hidx.back(loop).castTo(Trigger::new);
    } else {
      return m_hidp.create(loop).castTo(Trigger::new);
    }
  }

  /**
   * Constructs a Trigger instance around the axis value of the left trigger. The returned trigger
   * will be true when the axis value is greater than {@code threshold}.
   *
   * @param threshold the minimum axis value for the returned {@link Trigger} to be true. This value
   *     should be in the range [0, 1] where 0 is the unpressed state of the axis.
   * @param loop the event loop instance to attach the Trigger to.
   * @return a Trigger instance that is true when the left trigger's axis exceeds the provided
   *     threshold, attached to the given event loop
   */
  public Trigger leftTrigger(double threshold, EventLoop loop) {
    if (xbox) {
      return m_hidx.leftTrigger(threshold, loop).castTo(Trigger::new);
    } else {
      return new BooleanEvent(loop, () -> m_hidp.getL2Axis() > threshold).castTo(Trigger::new);
    }
  }

  /**
   * Constructs a Trigger instance around the axis value of the left trigger. The returned trigger
   * will be true when the axis value is greater than {@code threshold}.
   *
   * @param threshold the minimum axis value for the returned {@link Trigger} to be true. This value
   *     should be in the range [0, 1] where 0 is the unpressed state of the axis.
   * @return a Trigger instance that is true when the left trigger's axis exceeds the provided
   *     threshold, attached to the {@link CommandScheduler#getDefaultButtonLoop() default scheduler
   *     button loop}.
   */
  public Trigger leftTrigger(double threshold) {
    return leftTrigger(threshold, CommandScheduler.getInstance().getDefaultButtonLoop());
  }

  /**
   * Constructs a Trigger instance around the axis value of the left trigger. The returned trigger
   * will be true when the axis value is greater than 0.5.
   *
   * @return a Trigger instance that is true when the left trigger's axis exceeds 0.5, attached to
   *     the {@link CommandScheduler#getDefaultButtonLoop() default scheduler button loop}.
   */
  public Trigger leftTrigger() {
    return leftTrigger(0.5);
  }

  /**
   * Constructs a Trigger instance around the axis value of the right trigger. The returned trigger
   * will be true when the axis value is greater than {@code threshold}.
   *
   * @param threshold the minimum axis value for the returned {@link Trigger} to be true. This value
   *     should be in the range [0, 1] where 0 is the unpressed state of the axis.
   * @param loop the event loop instance to attach the Trigger to.
   * @return a Trigger instance that is true when the right trigger's axis exceeds the provided
   *     threshold, attached to the given event loop
   */
  public Trigger rightTrigger(double threshold, EventLoop loop) {
    if (xbox) {
      return m_hidx.rightTrigger(threshold, loop).castTo(Trigger::new);
    } else {
      return new BooleanEvent(loop, () -> m_hidp.getR2Axis() > threshold).castTo(Trigger::new);
    }
  }

  /**
   * Constructs a Trigger instance around the axis value of the right trigger. The returned trigger
   * will be true when the axis value is greater than {@code threshold}.
   *
   * @param threshold the minimum axis value for the returned {@link Trigger} to be true. This value
   *     should be in the range [0, 1] where 0 is the unpressed state of the axis.
   * @return a Trigger instance that is true when the right trigger's axis exceeds the provided
   *     threshold, attached to the {@link CommandScheduler#getDefaultButtonLoop() default scheduler
   *     button loop}.
   */
  public Trigger rightTrigger(double threshold) {
    return rightTrigger(threshold, CommandScheduler.getInstance().getDefaultButtonLoop());
  }

  /**
   * Constructs a Trigger instance around the axis value of the right trigger. The returned trigger
   * will be true when the axis value is greater than 0.5.
   *
   * @return a Trigger instance that is true when the right trigger's axis exceeds 0.5, attached to
   *     the {@link CommandScheduler#getDefaultButtonLoop() default scheduler button loop}.
   */
  public Trigger rightTrigger() {
    return rightTrigger(0.5);
  }

  /**
   * Get the X axis value of left side of the controller.
   *
   * @return The axis value.
   */
  public double getLeftX() {
    if (xbox) {
      return m_hidx.getLeftX();
    } else {
      return m_hidp.getLeftX();
    }
  }

  /**
   * Get the X axis value of right side of the controller.
   *
   * @return The axis value.
   */
  public double getRightX() {
    if (xbox) {
      return m_hidx.getRightX();
    } else {
      return m_hidp.getRightX();
    }
  }

  /**
   * Get the Y axis value of left side of the controller.
   *
   * @return The axis value.
   */
  public double getLeftY() {
    if (xbox) {
      return m_hidx.getLeftY();
    } else {
      return m_hidp.getLeftY();
    }
  }

  /**
   * Get the Y axis value of right side of the controller.
   *
   * @return The axis value.
   */
  public double getRightY() {
    if (xbox) {
      return m_hidx.getRightY();
    } else {
      return m_hidp.getRightY();
    }
  }

  /**
   * Get the left trigger (LT) axis value of the controller. Note that this axis is bound to the
   * range of [0, 1] as opposed to the usual [-1, 1].
   *
   * @return The axis value.
   */
  public double getLeftTriggerAxis() {
    if (xbox) {
      return m_hidx.getLeftTriggerAxis();
    } else {
      return m_hidp.getL2Axis();
    }
  }

  /**
   * Get the right trigger (RT) axis value of the controller. Note that this axis is bound to the
   * range of [0, 1] as opposed to the usual [-1, 1].
   *
   * @return The axis value.
   */
  public double getRightTriggerAxis() {
    if (xbox) {
      return m_hidx.getRightTriggerAxis();
    } else {
      return m_hidp.getR2Axis();
    }
  }
}
