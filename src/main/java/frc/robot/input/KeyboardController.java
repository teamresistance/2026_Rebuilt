package frc.robot.input;

import edu.wpi.first.math.MathUtil;
import java.awt.KeyEventDispatcher;
import java.awt.KeyboardFocusManager;
import java.awt.event.KeyEvent;

/**
 * Lightweight keyboard helper that maps WASD / arrow keys and Q/E to drive inputs. Designed for
 * simulation use alongside a physical controller; its outputs are normalized in [-1, 1] so they can
 * be summed with joystick inputs.
 */
public class KeyboardController {
  // Movement keys
  private volatile boolean forward = false; // W / Up
  private volatile boolean back = false; // S / Down
  private volatile boolean left = false; // A / Left
  private volatile boolean right = false; // D / Right

  // Rotation keys
  private volatile boolean rotateLeft = false; // Q
  private volatile boolean rotateRight = false; // E

  // Optional boost (shift)
  private volatile boolean boost = false; // Shift

  // Per-key magnitude when pressed (normalized)
  private double keyMagnitude = 0.6; // default keyboard intensity
  private double rotationMagnitude = 0.6;

  public KeyboardController() {
    KeyEventDispatcher dispatcher =
        new KeyEventDispatcher() {
          @Override
          public boolean dispatchKeyEvent(KeyEvent e) {
            int id = e.getID();
            int code = e.getKeyCode();

            boolean isPressed = id == KeyEvent.KEY_PRESSED;
            boolean isReleased = id == KeyEvent.KEY_RELEASED;

            if (isPressed || isReleased) {
              boolean set = isPressed;
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
                  // ignore other keys
              }
            }

            // Return false so other dispatchers can process the event
            return false;
          }
        };

    KeyboardFocusManager.getCurrentKeyboardFocusManager().addKeyEventDispatcher(dispatcher);
  }

  /** Normalized forward/back component. Positive = forward. */
  public double getForward() {
    double v = (forward ? 1.0 : 0.0) - (back ? 1.0 : 0.0);
    if (boost) v *= 1.25;
    return MathUtil.clamp(v * keyMagnitude, -1.0, 1.0);
  }

  /** Normalized strafe (left/right). Positive = right. */
  public double getStrafe() {
    double v = (right ? 1.0 : 0.0) - (left ? 1.0 : 0.0);
    if (boost) v *= 1.25;
    return MathUtil.clamp(v * keyMagnitude, -1.0, 1.0);
  }

  /** Normalized rotational input. Positive = clockwise rotation. */
  public double getOmega() {
    double v = (rotateRight ? 1.0 : 0.0) - (rotateLeft ? 1.0 : 0.0);
    if (boost) v *= 1.25;
    return MathUtil.clamp(v * rotationMagnitude, -1.0, 1.0);
  }

  /** Set the keyboard movement magnitude (0..1). Default 0.6. */
  public void setKeyMagnitude(double mag) {
    this.keyMagnitude = MathUtil.clamp(mag, 0.0, 1.0);
  }

  /** Set the keyboard rotation magnitude (0..1). Default 0.6. */
  public void setRotationMagnitude(double mag) {
    this.rotationMagnitude = MathUtil.clamp(mag, 0.0, 1.0);
  }
}
