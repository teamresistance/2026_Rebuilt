package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.DeferredCommand;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.HoppertCommand;
import frc.robot.commands.IdleShooterCommand;
import frc.robot.commands.ShootCommand;
import frc.robot.commands.ToggleIntakeCommand;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.climber.ClimberIO;
import frc.robot.subsystems.climber.ClimberReal;
import frc.robot.subsystems.climber.ClimberSim;
import frc.robot.subsystems.drive.*;
import frc.robot.subsystems.hoppert.HoppertIO;
import frc.robot.subsystems.hoppert.HoppertReal;
import frc.robot.subsystems.hoppert.HoppertSim;
import frc.robot.subsystems.intake.IntakeIO;
import frc.robot.subsystems.intake.IntakeReal;
import frc.robot.subsystems.intake.IntakeSim;
import frc.robot.subsystems.leds.LEDStream;
import frc.robot.subsystems.leds.LEDSubsystem;
import frc.robot.subsystems.shooter.ShooterIO;
import frc.robot.subsystems.shooter.ShooterReal;
import frc.robot.subsystems.shooter.ShooterSim;
import frc.robot.subsystems.shooter.ShootingConstants;
import frc.robot.subsystems.vision.*;
import frc.robot.util.*;
import java.io.IOException;
import java.util.Set;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import org.photonvision.PhotonCamera;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  public final PhotonCamera frontLeftCamera = new PhotonCamera("front-left");
  public final PhotonCamera frontRightCamera = new PhotonCamera("front-right");
  public final PhotonCamera backLeftCamera = new PhotonCamera("back-left");
  public final PhotonCamera backRightCamera = new PhotonCamera("back-right");
  private final Alert cameraFailureAlert;

  // Subsystems
  private final SwerveDriveIO drive;
  private VisionSubsystem vision;
  private final ShooterIO shooter;
  private final ClimberIO climber;
  private final HoppertIO hoppert;
  private final IntakeIO intake;
  private final LEDSubsystem leds = new LEDSubsystem(); // does not need IO

  // Controller
  private final CommandXboxController driver = new CommandXboxController(0);
  private final CommandXboxController coDriver = new CommandXboxController(1);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;
  private final SendableChooser<Boolean> swivelStop = new SendableChooser<>();
  private final SendableChooser<String> manualShiftAssigner = new SendableChooser<>();

  // bump zone and prebuilt commands
  private final Trigger inBumpZone;
  private final Command driveAtAngleForBump;
  private final Command driveShooting;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    drive = configureDrive();
    vision = configureAprilTagVision();
    ShootingConstants.configureShootingConstants();

    switch (Constants.CURRENT_MODE) {
      case REAL:
        shooter = new ShooterReal();
        hoppert = new HoppertReal();
        climber = new ClimberReal();
        intake = new IntakeReal();
        break;
      case SIM:
        shooter = new ShooterSim();
        hoppert = new HoppertSim();
        climber = new ClimberSim();
        intake = new IntakeSim();
        break;
      default:
        intake = new IntakeReal();
        shooter = new ShooterReal();
        hoppert = new HoppertReal();
        climber = new ClimberReal();
    }

    configureNamedCommands();

    // bump stuff
    inBumpZone = new Trigger(() -> BumpUtil.inBumpZone(drive::getPose, drive::getChassisSpeeds));

    // pid angle control to the rotationToSnap() return
    driveAtAngleForBump =
        DriveCommands.joystickDriveAtAngle(
                drive,
                () -> -driver.getLeftY(),
                () -> -driver.getLeftX(),
                () -> BumpUtil.rotationToSnap(drive::getRotation))
            .withInterruptBehavior(Command.InterruptionBehavior.kCancelIncoming);
    driveAtAngleForBump.addRequirements(drive);

    // angle assist vs. slow driving
    driveShooting =
        new ConditionalCommand(
            DriveCommands.joystickDriveAtAngle(
                drive,
                () -> MathUtil.clamp(-driver.getLeftY(), -0.5, 0.5),
                () -> MathUtil.clamp(-driver.getLeftX(), -0.5, 0.5),
                () ->
                    drive
                        .getRotation()
                        .plus(Rotation2d.fromDegrees(shooter.getDriveAssistanceAngle() + 90))
                        .plus(Rotation2d.fromDegrees(-driver.getRightX() * 90))),
            DriveCommands.joystickDrive(
                drive,
                () -> MathUtil.clamp(-driver.getLeftY(), -0.5, 0.5),
                () -> MathUtil.clamp(-driver.getLeftX(), -0.5, 0.5),
                () -> MathUtil.clamp(-driver.getRightX(), -0.4, 0.4)),
            () ->
                Math.abs(shooter.getDriveAssistanceAngle()) > 2
                    && driver.y().negate().getAsBoolean());
    driveShooting.addRequirements(drive);

    manualShiftAssigner.addOption("Red", "R");
    manualShiftAssigner.addOption("Blue", "B");
    manualShiftAssigner.setDefaultOption("None", "");
    SmartDashboard.putData("Manual Shift Setup", manualShiftAssigner);

    swivelStop.addOption("STOP", true);
    swivelStop.addOption("GOOD", false);
    swivelStop.setDefaultOption("GOOD", false);
    SmartDashboard.putData("Turret Swivel Stop", swivelStop);

    configureDriverFeedback();
    autoChooser = configureAutos();
    configureButtonBindings();
    cameraFailureAlert = new Alert("Camera system failure", Alert.AlertType.kError);
  }

  private void configureNamedCommands() {
    NamedCommands.registerCommand(
        "Climb Up",
        Commands.runOnce(climber::unbrake)
            .andThen(climber::down)
            .andThen(new WaitUntilCommand(climber::atTarget))
            .andThen(climber::brake));
    NamedCommands.registerCommand("Stop", Commands.runOnce(drive::stop, drive));
    NamedCommands.registerCommand(
        "Shoot 5s",
        (new ShootCommand(drive, shooter, () -> false)
                .alongWith(new HoppertCommand(hoppert, shooter, intake, () -> true)))
            .withTimeout(5));
    NamedCommands.registerCommand(
        "Shoot 10s",
        (new ShootCommand(drive, shooter, () -> false)
                .alongWith(new HoppertCommand(hoppert, shooter, intake, () -> true)))
            .withTimeout(10));
    NamedCommands.registerCommand(
        "Shoot 7s",
        (new ShootCommand(drive, shooter, () -> false)
                .alongWith(new HoppertCommand(hoppert, shooter, intake, () -> true)))
            .withTimeout(7));
    // TODO: outpost shoot for longer?
    NamedCommands.registerCommand("Toggle Intake", new ToggleIntakeCommand(intake));
    NamedCommands.registerCommand(
        "Closest Climb",
        new DeferredCommand(
            () ->
                DriveCommands.goToTransform(
                    drive,
                    GeomUtil.poseToTransform(OtherUtil.getClimberAlignPos(drive.getPose())))));
  }

  private LoggedDashboardChooser<Command> configureAutos() {
    // Set up auto routines
    LoggedDashboardChooser<Command> chooser =
        new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    // Set up SysId routines
    chooser.addOption(
        "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
    chooser.addOption(
        "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
    chooser.addOption(
        "Drive SysId (Quasistatic Forward)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    chooser.addOption(
        "Drive SysId (Quasistatic Reverse)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    chooser.addOption(
        "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    chooser.addOption(
        "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));

    return chooser;
  }

  /**
   * Configures the AprilTag vision system with PhotonVision cameras.
   *
   * @return The configured VisionSubsystem, or null if initialization fails
   */
  private VisionSubsystem configureAprilTagVision() {
    try {
      vision =
          new VisionSubsystem(frontLeftCamera, frontRightCamera, backRightCamera, backLeftCamera);
      vision.setDataInterfaces(drive::getPose, drive::addAutoVisionMeasurement);

    } catch (IOException e) {
      if (cameraFailureAlert != null) {
        cameraFailureAlert.set(true);
      }

      Logger.recordOutput("Vision/FieldLayoutLoadError", e.getMessage());
      return null; // Return null on failure for proper error handling
    }
    return vision;
  }

  private SwerveDriveIO configureDrive() {
    // Real robot, instantiate hardware IO implementations
    // Sim robot, instantiate physics sim IO implementations
    // Replayed robot, disable IO implementations
    return switch (Constants.CURRENT_MODE) {
      case REAL ->
          // Real robot, instantiate hardware IO implementations
          new SwerveDriveReal(
              new GyroIOPigeon2(),
              new ModuleIOTalonFX(TunerConstants.FrontLeft),
              new ModuleIOTalonFX(TunerConstants.FrontRight),
              new ModuleIOTalonFX(TunerConstants.BackLeft),
              new ModuleIOTalonFX(TunerConstants.BackRight));
      case SIM ->
          // Sim robot, instantiate MapleSim drive simulation
          new SwerveDriveSim();
      default ->
          // Replayed robot, disable IO implementations
          new SwerveDriveReal(
              new GyroIO() {},
              new ModuleIO() {},
              new ModuleIO() {},
              new ModuleIO() {},
              new ModuleIO() {});
    };
  }

  /** Sets up LEDs and controller rumbles */
  private void configureDriverFeedback() {

    //    // SHOOTING/PASSING (priority 4, determines confidence and passing/shooting, framerate
    // based on
    //    // confidence)
    //    LEDStream shootingStream =
    //        new LEDStream(
    //                "shooting/passing",
    //                4,
    //                () -> {
    //                  boolean isShooting = ShootingUtil.getShootingType(drive::getPose) == 0;
    //                  boolean isConfident =
    //                      TurretConfidenceUtil.calculateConfidence(drive)
    //                          > Constants.CONFIDENCE_THRESHOLD;
    //
    //                  if (isShooting) {
    //                    return isConfident
    //                        ? Constants.LEDMode.SHOOTING_CONFIDENT
    //                        : Constants.LEDMode.SHOOTING_DOUBTFUL;
    //                  } else {
    //                    return isConfident
    //                        ? Constants.LEDMode.PASSING_CONFIDENT
    //                        : Constants.LEDMode.PASSING_DOUBTFUL;
    //                  }
    //                },
    //                () ->
    //                    driver
    //                        .rightTrigger()
    //                        .or((coDriver.rightTrigger().or(coDriver.rightBumper())))
    //                        .getAsBoolean())
    //            .withFramerateSupplier(
    //                () -> {
    //                  double confidence = TurretConfidenceUtil.calculateConfidence(drive);
    //                  return (confidence > 90.0)
    //                      ? 10 // very high framerate for very high confidence
    //                      : (confidence > 80.0)
    //                          ? 9
    //                          : (confidence > 70.0) ? 8 : (confidence > 60.0) ? 7 : 6;
    //                });
    //
    //    leds.addStream(shootingStream);
    //
    //    // INTAKING (priority 2, flashing yellow)
    //    //    LEDStream intakeStream =
    //    //        new LEDStream("intake", 2, () -> Constants.LEDMode.INTAKING,
    // intake::isIntaking);
    //    //    leds.addStream(intakeStream);
    //
    //    // DISABLED
    //    LEDStream disabledStream =
    //        new LEDStream("disabled", 999, () -> Constants.LEDMode.DISABLED,
    // DriverStation::isDisabled);
    //    leds.addStream(disabledStream);
    //
    //    // ACTIVE/INACTIVE
    //    LEDStream activeInactiveStream =
    //        new LEDStream(
    //            "active/inactive",
    //            1,
    //            () ->
    //                ShiftUtil.isOurs(ShiftUtil.getShift())
    //                    ? Constants.LEDMode.ACTIVE
    //                    : Constants.LEDMode.INACTIVE,
    //            () -> true);
    //    leds.addStream(activeInactiveStream);

    // DISABLED
    LEDStream disabledStream =
        new LEDStream("disabled", 999, () -> Constants.LEDMode.DISABLED, DriverStation::isDisabled);
    leds.addStream(disabledStream);

    // SHOOTING
    LEDStream shootStream =
        new LEDStream(
            "shooting",
            2,
            () -> Constants.LEDMode.SHOOTING_CONFIDENT,
            () ->
                Math.abs(driver.getHID().getRightTriggerAxis()) > 0.25
                    || driver.getHID().getRightBumperButton());
    leds.addStream(shootStream);

    // ACTIVE vs INACTIVE
    LEDStream activeStream =
        new LEDStream(
            "active/inactive",
            1,
            () ->
                ShiftUtil.isOurs(ShiftUtil.getShift())
                    ? Constants.LEDMode.ACTIVE
                    : Constants.LEDMode.INACTIVE,
            () -> true);
    leds.addStream(activeStream);

    LEDStream shiftCountdown =
        new LEDStream(
            "shift countdown 7s",
            3,
            () -> Constants.LEDMode.CLOSE_TO_NEXT_SHIFT,
            ShiftUtil::withinSevenSecondsOfNextShift);
    leds.addStream(shiftCountdown);

    LEDStream shiftCountdown2 =
        new LEDStream(
            "shift countdown 2s",
            4,
            () ->
                ShiftUtil.getNextShift() == Constants.ShiftOwner.RED
                    ? Constants.LEDMode.CLOSE_TO_NEXT_SHIFT_R
                    : Constants.LEDMode.CLOSE_TO_NEXT_SHIFT_B,
            ShiftUtil::withinTwoSecondsOfNextShift);
    leds.addStream(shiftCountdown2);

    LEDStream endgame =
        new LEDStream(
            "endgame countdown", 6, () -> Constants.LEDMode.ENDGAME, ShiftUtil::isDeepEndgame);
    leds.addStream(endgame);

    // BUMP (priority 5, timed 1s, cancels if leaving zone)
    LEDStream bumpStream =
        new LEDStream("bump", 5, () -> Constants.LEDMode.BUMP, inBumpZone::getAsBoolean);
    leds.addStream(bumpStream);

    // BUMP trigger (timed 1s when entering bump zone)
    driver
        .y()
        .negate()
        .and(inBumpZone)
        .and(() -> !DriverStation.isAutonomous())
        .onTrue(Commands.runOnce(() -> bumpStream.runForSeconds(1)));

    // RUMBLE when 5s from next shift
    new Trigger(ShiftUtil::nearNextShift)
        .onTrue(
            Commands.runOnce(() -> driver.setRumble(GenericHID.RumbleType.kBothRumble, 1))
                .andThen(new WaitCommand(1))
                .andThen(
                    Commands.runOnce(
                        () -> driver.setRumble(GenericHID.RumbleType.kBothRumble, 0))));
  }

  /** Defines button bindings and control triggers */
  private void configureButtonBindings() {

    // Default: normal drive
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive, () -> -driver.getLeftY(), () -> -driver.getLeftX(), () -> -driver.getRightX()));

    // While right stick held: lock rotation to heading at moment of press
    driver
        .rightStick()
        .whileTrue(
            Commands.defer(
                () ->
                    DriveCommands.joystickDriveAtAngle(
                        drive,
                        () -> -driver.getLeftY(),
                        () -> -driver.getLeftX(),
                        drive::getRotation), // captured when stick is pressed
                Set.of(drive)));

    // have hopper automatically deciding when to run or not to run
    hoppert.setDefaultCommand(
        new HoppertCommand(
            hoppert,
            shooter,
            intake,
            () ->
                Math.abs(driver.getHID().getRightTriggerAxis()) > 0.25
                    || driver.getHID().getRightBumperButton()));

    // when y (paddle) is not pressed and in bump zone, auto rotate.
    driver
        .y()
        .negate()
        .and(inBumpZone)
        .and(() -> !DriverStation.isAutonomous())
        .whileTrue(driveAtAngleForBump);

    // climb raise robot
    driver
        .back()
        .onTrue(
            Commands.runOnce(climber::unbrake)
                .andThen(Commands.waitSeconds(0.1))
                .andThen(climber::up)
                .andThen(new WaitUntilCommand(climber::atTarget))
                .andThen(climber::brake));

    // climb descend robot
    driver
        .start()
        .onTrue(
            Commands.runOnce(climber::unbrake)
                .andThen(Commands.waitSeconds(0.1))
                .andThen(climber::down)
                .andThen(new WaitUntilCommand(climber::atTarget))
                .andThen(climber::brake));

    // reverse intake
    driver.leftBumper().whileTrue(Commands.runOnce(intake::reverseIntake));
    driver.leftBumper().onFalse(Commands.runOnce(intake::stopIntake));

    // closest climb align
    driver
        .x()
        .whileTrue(
            new DeferredCommand(
                () ->
                    DriveCommands.goToTransform(
                        drive,
                        GeomUtil.poseToTransform(OtherUtil.getClimberAlignPos(drive.getPose())))));

    // auto-aim hood and turret always
    shooter.setDefaultCommand(
        new IdleShooterCommand(drive, shooter, () -> driver.rightBumper().getAsBoolean()));

    // Switch to X pattern when X button is pressed
    //    driver.x().onTrue(Commands.runOnce(drive::stopWithX, drive));

    // reverse mecanums
    driver.b().whileTrue(Commands.run(hoppert::reverseHopperWheels));
    driver.b().onFalse(Commands.runOnce(hoppert::stopHopper));

    // shoot
    (driver.rightTrigger().or(driver.rightBumper()))
        .or(coDriver.rightTrigger().or(coDriver.rightBumper()))
        .whileTrue(new ShootCommand(drive, shooter, () -> driver.rightBumper().getAsBoolean()));
    (driver.rightTrigger().or(driver.rightBumper()))
        .or((coDriver.rightTrigger().or(coDriver.rightBumper())))
        .and(driver.y().negate())
        .whileTrue(driveShooting);
    (driver.rightTrigger().or(driver.rightBumper()))
        .or((coDriver.rightTrigger().or(coDriver.rightBumper())))
        .onFalse(
            new ParallelDeadlineGroup(
                    new ShootCommand(drive, shooter, () -> driver.rightBumper().getAsBoolean())
                        .withTimeout(1.25),
                    Commands.run(hoppert::runTowerForwards),
                    Commands.run(hoppert::stopHopper))
                .andThen(Commands.runOnce(hoppert::stopTower)));

    // left trigger toggles intake
    driver.leftTrigger().onTrue(new ToggleIntakeCommand(intake));
    driver.leftTrigger().onFalse(new ToggleIntakeCommand(intake));

    // hold to zero
    Command zeroCmd =
        Commands.run(
            () -> {
              shooter.setHoodTarget(Constants.SHOOTER_HOOD_MIN_PITCH);
              shooter.setTurretTarget(0, 0);
            });
    zeroCmd.addRequirements(shooter);
    driver.leftStick().whileTrue(zeroCmd);

    // POV for adjusting shooter trim, with up/down adjusting vertical and left/right adjusting
    // horizontal.
    //    driver.povUp().onTrue(Commands.runOnce(() -> shooter.adjustVerticalTrim(true)));
    //    driver.povDown().onTrue(Commands.runOnce(() -> shooter.adjustVerticalTrim(false)));
    driver
        .povRight()
        .or(coDriver.povRight())
        .onTrue(Commands.runOnce(() -> shooter.adjustHorizontalTrim(false)));
    driver
        .povLeft()
        .or(coDriver.povRight())
        .onTrue(Commands.runOnce(() -> shooter.adjustHorizontalTrim(true)));
  }

  /**
   * Starts the shift timer with teleop to keep sync, waits, and then assigns the shift from FMS
   * data. The delay is to ensure the FMS data has been sent and avoids errors if it is late.
   */
  public void setupShiftUtil() {
    CommandScheduler.getInstance()
        .schedule(
            Commands.runOnce(ShiftUtil::startShiftTimer)
                .andThen(new WaitCommand(1))
                .andThen(Commands.runOnce(ShiftUtil::assignShifts)));
  }

  /**
   * Creates an LEDStream that runs the auto animation 20 seconds and then is never accessed again.
   */
  public void runAutoLEDs() {
    LEDStream autoStream = new LEDStream("auto", 100, () -> Constants.LEDMode.AUTO);
    leds.addStream(autoStream);
    autoStream.runForSeconds(20);
  }

  public String getShiftChosen() {
    return manualShiftAssigner.getSelected();
  }

  public void checkTurretStop() {
    shooter.setSwivelStop(swivelStop.getSelected());
  }

  public void descendEnteringTeleop() {
    CommandScheduler.getInstance()
        .schedule(
            Commands.runOnce(climber::unbrake)
                .andThen(Commands.waitSeconds(0.1))
                .andThen(climber::up)
                .andThen(new WaitUntilCommand(climber::atTarget))
                .andThen(climber::brake));
  }

  /** Returns the autonomous command to schedule for the auto period. */
  public Command getAutonomousCommand() {
    Command autoCommand = autoChooser.get();
    if (autoCommand == null) {
      Logger.recordOutput("Auto/NoCommandSelected", true);
      return Commands.none(); // Return empty command if no auto selected
    }

    return autoCommand;
  }

  public void coastDisabled() {
    shooter.coast();
  }

  public void brakeTeleop() {
    shooter.brake();
  }

  /**
   * Gets the drive subsystem for use in other classes
   *
   * @return the drive subsystem instance
   */
  public SwerveDriveIO getDrive() {
    return drive;
  }
}
