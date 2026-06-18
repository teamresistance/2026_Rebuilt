package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
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
import frc.robot.subsystems.LEDSubsystem;
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
  private final XboxController driverHID = driver.getHID();
  private final CommandXboxController coDriver = new CommandXboxController(1);
  private final XboxController operatorHID = coDriver.getHID();

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;
  private final SendableChooser<Boolean> swivelStop = new SendableChooser<>();
  private final SendableChooser<String> manualShiftAssigner = new SendableChooser<>();
  private final SendableChooser<Boolean> startTrimChooser = new SendableChooser<>();

  // bump zone and prebuilt commands
  private final Trigger inBumpZone;
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

    // angle assist vs. slow driving
    driveShooting =
        new ConditionalCommand(
            DriveCommands.joystickDriveAtAngle(
                drive,
                () -> MathUtil.clamp(-driverHID.getLeftY(), -0.7, 0.7),
                () -> MathUtil.clamp(-driverHID.getLeftX(), -0.7, 0.7),
                () ->
                    drive
                        .getRotation()
                        .plus(Rotation2d.fromDegrees(shooter.getDriveAssistanceAngle()))
                        .plus(Rotation2d.fromDegrees(-driverHID.getRightX() * 90))),
            DriveCommands.joystickDrive(
                drive,
                () -> MathUtil.clamp(-driverHID.getLeftY(), -0.7, 0.7),
                () -> MathUtil.clamp(-driverHID.getLeftX(), -0.7, 0.7),
                () -> MathUtil.clamp(-driverHID.getRightX(), -0.5, 0.5)),
            () -> Math.abs(shooter.getDriveAssistanceAngle()) > 2 && !driverHID.getYButton());
    driveShooting.addRequirements(drive);

    manualShiftAssigner.addOption("Red", "R");
    manualShiftAssigner.addOption("Blue", "B");
    manualShiftAssigner.setDefaultOption("None", "");
    SmartDashboard.putData("Manual Shift Setup", manualShiftAssigner);

    swivelStop.addOption("STOP", true);
    swivelStop.addOption("GOOD", false);
    swivelStop.setDefaultOption("GOOD", false);
    SmartDashboard.putData("Turret Swivel Stop", swivelStop);

    startTrimChooser.setDefaultOption("No", false);
    startTrimChooser.addOption("Yes", true);
    SmartDashboard.putData("Start Trimmed", startTrimChooser);

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
            // .alongWith(new HoppertCommand(hoppert, shooter, intake, () -> true)))
            .withTimeout(5)));
    NamedCommands.registerCommand(
        "Shoot 4s",
        (new ShootCommand(drive, shooter, () -> false)
            // .alongWith(new HoppertCommand(hoppert, shooter, intake, () -> true)))
            .withTimeout(4)));
    NamedCommands.registerCommand(
        "Shoot 3s",
        (new ShootCommand(drive, shooter, () -> false)
            // .alongWith(new HoppertCommand(hoppert, shooter, intake, () -> true)))
            .withTimeout(3)));
    NamedCommands.registerCommand(
        "Shoot 10s",
        (new ShootCommand(drive, shooter, () -> false)
            // .alongWith(new HoppertCommand(hoppert, shooter, intake, () -> true)))
            .withTimeout(10)));
    NamedCommands.registerCommand(
        "Shoot 7s",
        (new ShootCommand(drive, shooter, () -> false)
            // .alongWith(new HoppertCommand(hoppert, shooter, intake, () -> true)))
            .withTimeout(7)));
    // TODO: outpost shoot for longer?
    NamedCommands.registerCommand("Toggle Intake", new ToggleIntakeCommand(intake));
    // Java
    NamedCommands.registerCommand(
        "Closest Climb",
        new DeferredCommand(
            () -> {
              var currentPose = drive.getPose();
              var targetPose = OtherUtil.getClimberAlignPos(currentPose);
              var intermediatePose =
                  new Pose2d(targetPose.getX(), currentPose.getY(), targetPose.getRotation());
              return DriveCommands.goToTransform(drive, GeomUtil.poseToTransform(intermediatePose))
                  .andThen(
                      DriveCommands.goToTransform(drive, GeomUtil.poseToTransform(targetPose)));
            }));
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

    // TODO: Transfer LEDs to new system

    // RUMBLE when 5s from next shift
    new Trigger(ShiftUtil::nearNextShift)
        .onTrue(
            Commands.runOnce(() -> driverHID.setRumble(GenericHID.RumbleType.kBothRumble, 1))
                .andThen(new WaitCommand(1))
                .andThen(
                    Commands.runOnce(
                        () -> driverHID.setRumble(GenericHID.RumbleType.kBothRumble, 0))));
  }

  /** Defines button bindings and control triggers */
  private void configureButtonBindings() {

    // Default: normal drive
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -driverHID.getLeftY(),
            () -> -driverHID.getLeftX(),
            () -> -driverHID.getRightX()));

    // While right stick held: lock rotation to heading at moment of press
    driver
        .rightStick()
        .whileTrue(
            Commands.defer(
                () ->
                    DriveCommands.joystickDriveAtAngle(
                        drive,
                        () -> -driverHID.getLeftY(),
                        () -> -driverHID.getLeftX(),
                        drive::getRotation), // captured when stick is pressed
                Set.of(drive)));

    // have hopper automatically deciding when to run or not to run
    hoppert.setDefaultCommand(
        new HoppertCommand(
            hoppert,
            shooter,
            intake,
            () ->
                (Math.abs(driverHID.getRightTriggerAxis()) > 0.25
                        || driverHID.getRightBumperButton())
                    || (Math.abs(operatorHID.getRightTriggerAxis()) > 0.25
                        || operatorHID.getRightBumperButton())));

    // when POV up/down pressed and in bump zone, auto rotate to left/right side
    driver
        .povUp()
        .whileTrue(
            DriveCommands.joystickDriveAtAngle(
                drive,
                () -> -driverHID.getLeftY(),
                () -> -driverHID.getLeftX(),
                () ->
                    DriverStation.getAlliance().get().equals(DriverStation.Alliance.Red)
                        ? Rotation2d.fromDegrees(-135) // Left side for Red
                        : Rotation2d.fromDegrees(135))); // Left side for Blue

    driver
        .povDown()
        .whileTrue(
            DriveCommands.joystickDriveAtAngle(
                drive,
                () -> -driverHID.getLeftY(),
                () -> -driverHID.getLeftX(),
                () ->
                    DriverStation.getAlliance().get().equals(DriverStation.Alliance.Red)
                        ? Rotation2d.fromDegrees(-45) // Right side for Red
                        : Rotation2d.fromDegrees(45))); // Right side for Blue

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

    Command zeroCmd = Commands.run(() -> shooter.setTurretTarget(0, 0));
    zeroCmd.addRequirements(shooter);
    driver.leftStick().whileTrue(zeroCmd);

    // reverse intake
    driver.leftBumper().whileTrue(Commands.runOnce(intake::reverseIntake));
    driver.leftBumper().onFalse(Commands.runOnce(intake::stopIntake));

    // closest climb align
    driver
        .x()
        .whileTrue(
            new DeferredCommand(
                () -> {
                  var currentPose = drive.getPose();
                  var targetPose = OtherUtil.getClimberAlignPos(currentPose);
                  var intermediatePose =
                      new Pose2d(targetPose.getX(), currentPose.getY(), targetPose.getRotation());
                  return DriveCommands.goToTransform(
                          drive, GeomUtil.poseToTransform(intermediatePose))
                      .andThen(
                          DriveCommands.goToTransform(drive, GeomUtil.poseToTransform(targetPose)));
                }));

    // auto-aim hood and turret always
    shooter.setDefaultCommand(
        new IdleShooterCommand(drive, shooter, driverHID::getRightBumperButton));

    // Switch to X pattern when X button is pressed
    //    driverHID.x().onTrue(Commands.runOnce(drive::stopWithX, drive));

    // reverse mecanums
    driver.b().whileTrue(Commands.run(hoppert::reverseHopperWheels));
    driver.b().onFalse(Commands.runOnce(hoppert::stopWheels));

    Trigger shtTrg =
        new Trigger(driver.rightTrigger().or(driver.rightBumper()))
            .or(coDriver.rightTrigger().or(coDriver.rightBumper()));

    // shoot
    shtTrg.whileTrue(new ShootCommand(drive, shooter, driverHID::getRightBumperButton));
    shtTrg.whileTrue(driveShooting);
    shtTrg.onFalse(
        new ParallelDeadlineGroup(
                new ShootCommand(drive, shooter, driverHID::getRightBumperButton).withTimeout(0.9),
                Commands.run(hoppert::runTowerForwards),
                Commands.run(hoppert::stopHopper))
            .andThen(Commands.runOnce(hoppert::stopTower)));

    // left trigger toggles intake
    driver.leftTrigger().onTrue(new ToggleIntakeCommand(intake));
    driver.leftTrigger().onFalse(new ToggleIntakeCommand(intake));

    coDriver
        .back()
        .or(operatorHID::getStartButton)
        .whileTrue(Commands.run(climber::trimDown).withTimeout(0.1));

    // POV for adjusting shooter trim, with up/down adjusting vertical and left/right adjusting
    // horizontal.
    //    driverHID.povUp().onTrue(Commands.runOnce(() -> shooter.adjustVerticalTrim(true)));
    //    driverHID.povDown().onTrue(Commands.runOnce(() -> shooter.adjustVerticalTrim(false)));
    driver
        .povRight()
        .or(coDriver.povRight())
        .onTrue(Commands.runOnce(() -> shooter.adjustHorizontalTrim(false)));
    driver
        .povLeft()
        .or(coDriver.povLeft())
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
    Command autoCommand;
    if (startTrimChooser.getSelected()) {
      shooter.adjustHorizontalTrim(true);
      shooter.adjustHorizontalTrim(true);
      autoCommand =
          autoChooser
              .get()
              .andThen(
                  Commands.runOnce(
                      () -> {
                        shooter.adjustHorizontalTrim(false);
                        shooter.adjustHorizontalTrim(false);
                      }));
    } else {
      autoCommand = autoChooser.get();
    }
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
