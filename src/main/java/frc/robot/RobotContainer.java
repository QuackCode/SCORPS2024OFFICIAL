package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

// import frc.robot.autos.*;
import frc.robot.commands.*;
import frc.robot.subsystems.*;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    /* Controllers */
    private final Joystick driver = new Joystick(Constants.Drive.driveController);
    private final Joystick weapons = new Joystick(Constants.Drive.weaponController);

    /* Drive Controls */
    private final int d_translationAxis = XboxController.Axis.kLeftY.value;
    private final int d_strafeAxis = XboxController.Axis.kLeftX.value;
    private final int d_rotationAxis = XboxController.Axis.kRightX.value;

    /* Weapon Controls */
    private final int w_rotationAxis = XboxController.Axis.kLeftY.value;
    private final int w_climbAxis = XboxController.Axis.kRightY.value;
    private final int w_shooterAxis = XboxController.Axis.kRightTrigger.value;

    /* Driver Buttons */
    private final JoystickButton d_zeroGyro = new JoystickButton(driver, XboxController.Button.kY.value);
    private final JoystickButton d_robotCentric = new JoystickButton(driver, XboxController.Button.kLeftBumper.value);
    private final JoystickButton d_slowMode = new JoystickButton(driver, XboxController.Button.kRightBumper.value);
    private final JoystickButton d_getDriveInfoSnapshot = new JoystickButton(driver, XboxController.Button.kStart.value);

    /* Weapons Buttons */
    private final JoystickButton w_resetArm = new JoystickButton(weapons, XboxController.Button.kY.value);
    private final JoystickButton w_ampAutoArm = new JoystickButton(weapons, XboxController.Button.kB.value);
    private final JoystickButton w_speakerAutoArm = new JoystickButton(weapons, XboxController.Button.kX.value);
    private final JoystickButton w_instantShootNote = new JoystickButton(weapons, XboxController.Button.kLeftBumper.value);
    private final JoystickButton w_intakeNote = new JoystickButton(weapons, XboxController.Button.kRightBumper.value);
    private final JoystickButton w_getWeaponInfoSnapshot = new JoystickButton(weapons, XboxController.Button.kStart.value);

    /* Subsystems */
    private final Swerve s_Swerve = new Swerve();
    private final Arms a_Arms = new Arms();
    private final Jukebox j_Jukebox = new Jukebox();
    private final Climbers c_Climbers = new Climbers();
    private final Vision v_Vision = new Vision();
    private final Time t_Time = new Time();

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        
        /* PathPlanner Registered Commands */
        NamedCommands.registerCommand("Intake Note", new AutoIntake(t_Time, j_Jukebox, 2.25).withTimeout(2.25));
        NamedCommands.registerCommand("Shoot Note", new AutoJukebox(t_Time, j_Jukebox).withTimeout(2.5));
        NamedCommands.registerCommand("Arm to Amp", new InstantAutoArm(t_Time, a_Arms, Constants.Arms.calculatedArmThetaAtAmp, 2.25).withTimeout(2.25)); // NOT USED
        NamedCommands.registerCommand("Arm to Bottom", new InstantAutoArm(t_Time, a_Arms, 650, 0.75).withTimeout(0.75)); // 3 for single auto
        
        NamedCommands.registerCommand("Arm to Speaker", new InstantAutoArm(t_Time, a_Arms, Constants.Arms.calculatedArmThetaAtSpeaker, 3).withTimeout(3)); // 3 for single auto
        NamedCommands.registerCommand("Fast Arm to Speaker", new InstantAutoArm(t_Time, a_Arms, Constants.Arms.calculatedArmThetaAtSpeaker, 1.75).withTimeout(1.75));

        s_Swerve.setDefaultCommand(
            new TeleopSwerve(
                s_Swerve, 
                v_Vision, 
                () -> -driver.getRawAxis(d_translationAxis), 
                () -> -driver.getRawAxis(d_strafeAxis), 
                () -> -driver.getRawAxis(d_rotationAxis), 
                () -> d_robotCentric.getAsBoolean(),
                () -> d_slowMode.getAsBoolean(),
                () -> d_angleAlign.getAsBoolean()
            )
        );

        a_Arms.setDefaultCommand(
            new TeleopArm(
                a_Arms,
                () -> -weapons.getRawAxis(w_rotationAxis),
                () -> w_slowMode.getAsBoolean()
            )
        );

        j_Jukebox.setDefaultCommand(
            new TeleopJukebox(
                j_Jukebox,
                () -> weapons.getRawAxis(w_shooterAxis),  
                () -> w_intakeNote.getAsBoolean()
            )
        );

        c_Climbers.setDefaultCommand(
            new TeleopClimb(
                c_Climbers,
                () -> weapons.getRawAxis(w_climbAxis)
            )
        );

        // Configure the button bindings
        configureButtonBindings();
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        /* Driver Buttons */
        d_zeroGyro.onTrue(new InstantCommand(() -> s_Swerve.zeroHeading()));
        d_getDriveInfoSnapshot.onTrue(new InstantCommand(() -> s_Swerve.displaySwerveInfoSnapshot()));
        d_getDriveInfoSnapshot.onTrue(new InstantCommand(() -> v_Vision.displayVisionInfoSnapshot()));

        /* Weapon Buttons */
        w_instantShootNote.onTrue(new InstantCommand(() -> j_Jukebox.runJukebox(1.0)));
        w_getWeaponInfoSnapshot.onTrue(new InstantCommand(() -> a_Arms.displayArmInfoSnapshot()));
        w_getWeaponInfoSnapshot.onTrue(new InstantCommand(() -> j_Jukebox.displayJukeboxInfoSnapshot()));

        if (Constants.Drive.useInstantPID) {
            if (Constants.Drive.useMotionMagicPID) {
                w_resetArm.onTrue(new InstantCommand(() -> a_Arms.motionMagicAutoSetArmPosition(Constants.Arms.calculatedArmThetaAtDefault)));
                w_upAutoArm.onTrue(new InstantCommand(() -> a_Arms.motionMagicAutoSetArmPosition(Constants.Arms.calculatedArmThetaAtUp)));
                w_ampAutoArm.onTrue(new InstantCommand(() -> a_Arms.motionMagicAutoSetArmPosition(Constants.Arms.calculatedArmThetaAtAmp)));
                w_speakerAutoArm.onTrue(new InstantCommand(() -> a_Arms.motionMagicAutoSetArmPosition(Constants.Arms.calculatedArmThetaAtSpeaker)));
            } else {
                if (Constants.Drive.useIndividualMotorPID) {
                    w_resetArm.onTrue(new InstantCommand(() -> a_Arms.autoSetLeftArmPosition(Constants.Arms.calculatedArmThetaAtDefault)));
                    w_resetArm.onTrue(new InstantCommand(() -> a_Arms.autoSetRightArmPosition(Constants.Arms.calculatedArmThetaAtDefault)));
                    w_upAutoArm.onTrue(new InstantCommand(() -> a_Arms.autoSetLeftArmPosition(Constants.Arms.calculatedArmThetaAtUp)));
                    w_upAutoArm.onTrue(new InstantCommand(() -> a_Arms.autoSetRightArmPosition(Constants.Arms.calculatedArmThetaAtUp)));
                    w_ampAutoArm.onTrue(new InstantCommand(() -> a_Arms.autoSetLeftArmPosition(Constants.Arms.calculatedArmThetaAtAmp)));
                    w_ampAutoArm.onTrue(new InstantCommand(() -> a_Arms.autoSetRightArmPosition(Constants.Arms.calculatedArmThetaAtAmp)));
                    w_speakerAutoArm.onTrue(new InstantCommand(() -> a_Arms.autoSetLeftArmPosition(Constants.Arms.calculatedArmThetaAtSpeaker)));
                    w_speakerAutoArm.onTrue(new InstantCommand(() -> a_Arms.autoSetRightArmPosition(Constants.Arms.calculatedArmThetaAtSpeaker)));
                } else {
                    w_resetArm.onTrue(new InstantCommand(() -> a_Arms.autoSetArmPosition(Constants.Arms.calculatedArmThetaAtDefault)));
                    w_upAutoArm.onTrue(new InstantCommand(() -> a_Arms.autoSetArmPosition(Constants.Arms.calculatedArmThetaAtUp)));
                    w_ampAutoArm.onTrue(new InstantCommand(() -> a_Arms.autoSetArmPosition(Constants.Arms.calculatedArmThetaAtAmp)));
                    w_speakerAutoArm.onTrue(new InstantCommand(() -> a_Arms.autoSetArmPosition(Constants.Arms.calculatedArmThetaAtSpeaker)));
                }
            }
        } else {
            w_resetArm.whileTrue(new HeldAutoArm(a_Arms, () -> Constants.Arms.calculatedArmThetaAtDefault));
            w_upAutoArm.whileTrue(new HeldAutoArm(a_Arms, () -> Constants.Arms.calculatedArmThetaAtUp));
            w_ampAutoArm.whileTrue(new HeldAutoArm(a_Arms, () -> Constants.Arms.calculatedArmThetaAtAmp));
            w_speakerAutoArm.whileTrue(new HeldAutoArm(a_Arms, () -> Constants.Arms.calculatedArmThetaAtSpeaker));
        }
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // The PathPlanner path will run
        return new PathPlannerAuto("CENTERSPEAKER");
    }
}