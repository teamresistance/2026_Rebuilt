package frc.robot.input;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.GenericHID;
import java.awt.KeyEventDispatcher;
import java.awt.KeyboardFocusManager;
import java.awt.event.KeyEvent;

/**
 * A GenericHID-backed keyboard HID implementation for simulation.
 *
 * <p>Axes mapping: - axis 0: forward/back (W/S or Up/Down) - axis 1: strafe right/left (D/A or
 * Right/Left) - axis 2: rotation clockwise/counter-clockwise (E/Q)
 *
 * <p>Buttons: - button 1: boost (SHIFT)
 */
public class KeyboardHID extends GenericHID {
  private volatile boolean forward = false;
  private volatile boolean back = false;
  private volatile boolean left = false;
  private volatile boolean right = false;
  private volatile boolean rotateLeft = false;
  private volatile boolean rotateRight = false;
  private volatile boolean boost = false;

  private double keyMagnitude = 0.6;
  private double rotationMagnitude = 0.6;

  public KeyboardHID(int port) {
    super(port);

    KeyEventDispatcher dispatcher =
        e -> {
          int id = e.getID();
          int code = e.getKeyCode();

          boolean pressed = id == KeyEvent.KEY_PRESSED;
          boolean released = id == KeyEvent.KEY_RELEASED;

          if (pressed || released) {
            boolean set = pressed;
            switch (code) {
              case KeyEvent.VK_W:
              case KeyEvent.VK_UP:
                forward = set;
                break;
              case KeyEvent.VK_S:
              case KeyEvent.VK_DOWN:
                back = set;
                break;
              case KeyEvent.VK_A:
              case KeyEvent.VK_LEFT:
                left = set;
                break;
              case KeyEvent.VK_D:
              case KeyEvent.VK_RIGHT:
                right = set;
                break;
              case KeyEvent.VK_Q:
                rotateLeft = set;
                break;
              case KeyEvent.VK_E:
                rotateRight = set;
                break;
              case KeyEvent.VK_SHIFT:
                boost = set;
                break;
              default:
                break;
            }
          }

          return false;
        };

    KeyboardFocusManager.getCurrentKeyboardFocusManager().addKeyEventDispatcher(dispatcher);
  }

  @Override
  public double getRawAxis(int axis) {
    double v = 0.0;
    switch (axis) {
      case 0: // forward/back
        v = (forward ? 1.0 : 0.0) - (back ? 1.0 : 0.0);
        v *= keyMagnitude;
        break;
      case 1: // strafe right/left
        v = (right ? 1.0 : 0.0) - (left ? 1.0 : 0.0);
        v *= keyMagnitude;
        break;
      case 2: // rotation
        v = (rotateRight ? 1.0 : 0.0) - (rotateLeft ? 1.0 : 0.0);
        v *= rotationMagnitude;
        break;
      default:
        v = 0.0;
    }

    if (boost) v = Math.copySign(Math.min(1.0, Math.abs(v) * 1.25), v);
    return MathUtil.clamp(v, -1.0, 1.0);
  }

  @Override
  public boolean getRawButton(int button) {
    // Map button 1 to boost
    if (button == 1) {
      return boost;
    }
    return false;
  }

  public void setKeyMagnitude(double mag) {
    this.keyMagnitude = MathUtil.clamp(mag, 0.0, 1.0);
  }

  public void setRotationMagnitude(double mag) {
    this.rotationMagnitude = MathUtil.clamp(mag, 0.0, 1.0);
  }
}
