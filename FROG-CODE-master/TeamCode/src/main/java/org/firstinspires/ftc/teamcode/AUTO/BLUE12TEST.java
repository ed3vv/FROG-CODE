package org.firstinspires.ftc.teamcode.AUTO;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
import com.seattlesolvers.solverslib.command.ParallelDeadlineGroup;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.seattlesolvers.solverslib.controller.PIDController;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;
import com.seattlesolvers.solverslib.hardware.servos.ServoEx;
import com.seattlesolvers.solverslib.pedroCommand.FollowPathCommand;

import org.firstinspires.ftc.teamcode.GLOBALS.globals;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous
public class BLUE12TEST extends CommandOpMode {
    private Follower follower;
    private boolean scheduled = false;
    private SequentialCommandGroup froggyroute;
    public PathChain Path1, Path2, Path3, Path4, Path5, Path6, Path7, Path8, Path9;

    // ==================== SUBSYSTEMS ====================

    /**
     * IntakeSubsystem — owns intake motor, transfer motor, and gate servo.
     * Gate lives here because intake logic decides when to open/close gate
     * during feeding, and the launcher commands interact with it via
     * public methods (not by requiring this subsystem).
     */
    public static class IntakeSubsystem extends SubsystemBase {
        public final MotorEx intake, transfer;
        public final ServoEx gate;

        public IntakeSubsystem(HardwareMap hardwareMap) {
            intake = new MotorEx(hardwareMap, "intake");
            transfer = new MotorEx(hardwareMap, "transfer");

            intake.setRunMode(MotorEx.RunMode.RawPower);
            transfer.setRunMode(MotorEx.RunMode.RawPower);
            transfer.setInverted(true);
            intake.setInverted(false);
            intake.setZeroPowerBehavior(MotorEx.ZeroPowerBehavior.FLOAT);
            transfer.setZeroPowerBehavior(MotorEx.ZeroPowerBehavior.FLOAT);
            intake.stopAndResetEncoder();

            gate = new ServoEx(hardwareMap, "gate");
            gate.set(globals.gate.close);
        }

        public void startIntake() {
            intake.set(1);
            transfer.set(0.5);
            gate.set(globals.gate.close);
        }

        public void stopIntake() {
            intake.set(0);
            transfer.set(0);
        }

        /** Feed balls into launcher — called by launcher logic */
        public void feedLauncher() {
            gate.set(globals.gate.open);
            intake.set(0.6);
            transfer.set(0.6);
        }

        public void closeGate() {
            gate.set(globals.gate.close);
        }
    }

    /**
     * TurretSubsystem — owns turret servos and hood servo.
     * Separated so turret positioning can happen in parallel with
     * intake or launcher operations without resource conflicts.
     */
    public static class TurretSubsystem extends SubsystemBase {
        public final ServoEx turret1, turret2, hood;

        public TurretSubsystem(HardwareMap hardwareMap) {
            turret1 = new ServoEx(hardwareMap, "t1", 0, 360);
            turret2 = new ServoEx(hardwareMap, "t2", 0, 360);
            turret1.setInverted(true);
            turret2.setInverted(true);

            hood = new ServoEx(hardwareMap, "hood", 300);
        }

        public void aimForLaunch() {
            hood.set(200);
            turret1.set(globals.turret.turretangle);
            turret2.set(globals.turret.turretangle);
        }
    }

    /**
     * LauncherSubsystem — owns launcher motors and velocity PIDF.
     * Does NOT require IntakeSubsystem, but holds a reference to it
     * so it can call feedLauncher() when RPM is at setpoint.
     * This reference pattern avoids subsystem requirement conflicts
     * while allowing cross-subsystem coordination.
     */
    public static class LauncherSubsystem extends SubsystemBase {
        public final MotorEx launcher1, launcher2;
        private final PIDController launchPID;
        private final IntakeSubsystem intakeSub;

        private double lastTime;
        private double lastPosition;
        private double RPM;
        private boolean initialized = false;

        public LauncherSubsystem(HardwareMap hardwareMap, IntakeSubsystem intakeSub) {
            this.intakeSub = intakeSub;

            launcher1 = new MotorEx(hardwareMap, "l1", 28, 6000);
            launcher2 = new MotorEx(hardwareMap, "l2", 28, 6000);
            launcher1.setRunMode(Motor.RunMode.RawPower);
            launcher2.setRunMode(Motor.RunMode.RawPower);
            launcher2.setInverted(true);
            launcher1.setInverted(false);
            launcher1.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);
            launcher2.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);

            launchPID = new PIDController(globals.launcher.p, globals.launcher.i, globals.launcher.d);
            launchPID.setTolerance(50);
        }

        public void launch() {
            launchPID.setSetPoint(4500);
            double launchPower = launchPID.calculate(RPM);

            if (RPM < 400) {
                launcher1.set(0.55);
                launcher2.set(0.55);
            } else {
                double output = launchPower + globals.launcher.kv * 4500 + globals.launcher.ks;
                launcher1.set(output);
                launcher2.set(output);
            }

            if (launchPID.atSetPoint()) {
                intakeSub.feedLauncher();
            }
        }

        public void idle() {
            launcher1.set(0.2);
            launcher2.set(0.2);
        }

        public void stop() {
            launcher1.set(0);
            launcher2.set(0);
        }

        public double getRPM() {
            return RPM;
        }

        private void updateRPM() {
            double currentTime = com.qualcomm.robotcore.util.ElapsedTime
                    .class.equals(null) ? 0 : 0; // placeholder — see periodic()
            // We need the opmode runtime. Since SubsystemBase doesn't have getRuntime(),
            // we use a monotonic clock instead.
            double currentTimeMs = System.nanoTime() / 1_000_000_000.0;
            int currentPosition = launcher2.getCurrentPosition();

            if (!initialized) {
                lastTime = currentTimeMs;
                lastPosition = currentPosition;
                initialized = true;
                return;
            }

            double deltaTime = currentTimeMs - lastTime;
            double deltaTicks = currentPosition - lastPosition;

            if (deltaTime > 0.02) {
                double revs = deltaTicks / 28.0;
                RPM = -(revs / deltaTime) * 60.0;
                lastTime = currentTimeMs;
                lastPosition = currentPosition;
            }
        }

        @Override
        public void periodic() {
            updateRPM();
        }
    }

    // ==================== COMMANDS ====================

    /**
     * Runs the intake continuously. No isFinished — designed to be
     * cancelled by a deadline group or composed with a timeout.
     */
    public static class IntakeCommand extends CommandBase {
        private final IntakeSubsystem intakeSub;

        public IntakeCommand(IntakeSubsystem intakeSub) {
            this.intakeSub = intakeSub;
            addRequirements(intakeSub);
        }

        @Override
        public void initialize() {
            intakeSub.startIntake();
        }

        @Override
        public void end(boolean interrupted) {
            intakeSub.stopIntake();
        }
    }

    /**
     * Runs the launcher PIDF loop, feeding balls when at speed.
     * Requires both LauncherSubsystem and TurretSubsystem so
     * the turret is aimed before spinning up.
     */
    public static class LaunchCommand extends CommandBase {
        private final LauncherSubsystem launcherSub;
        private final TurretSubsystem turretSub;

        public LaunchCommand(LauncherSubsystem launcherSub, TurretSubsystem turretSub) {
            this.launcherSub = launcherSub;
            this.turretSub = turretSub;
            addRequirements(launcherSub, turretSub);
        }

        @Override
        public void initialize() {
            turretSub.aimForLaunch();
        }

        @Override
        public void execute() {
            launcherSub.launch();
        }

        @Override
        public void end(boolean interrupted) {
            launcherSub.idle();
        }
    }

    // ==================== PATHS ====================

    private void buildPaths() {
        Path1 = follower.pathBuilder().addPath(
                new BezierLine(
                        new Pose(51.000, 9.000),
                        new Pose(11.000, 9.000)
                )
        ).setConstantHeadingInterpolation(Math.toRadians(180)).build();

        Path2 = follower.pathBuilder().addPath(
                new BezierLine(
                        new Pose(11.000, 9.000),
                        new Pose(51.000, 9.000)
                )
        ).setConstantHeadingInterpolation(Math.toRadians(180)).build();

        Path3 = follower.pathBuilder().addPath(
                new BezierLine(
                        new Pose(51.000, 9.000),
                        new Pose(42.500, 35.500)
                )
        ).setConstantHeadingInterpolation(Math.toRadians(180)).build();

        Path4 = follower.pathBuilder().addPath(
                new BezierLine(
                        new Pose(42.500, 35.500),
                        new Pose(20.000, 35.500)
                )
        ).setConstantHeadingInterpolation(Math.toRadians(180)).build();

        Path5 = follower.pathBuilder().addPath(
                new BezierLine(
                        new Pose(20.000, 35.500),
                        new Pose(51.000, 9.000)
                )
        ).setConstantHeadingInterpolation(Math.toRadians(180)).build();

        Path6 = follower.pathBuilder().addPath(
                new BezierLine(
                        new Pose(51.000, 9.000),
                        new Pose(42.500, 60.000)
                )
        ).setConstantHeadingInterpolation(Math.toRadians(180)).build();

        Path7 = follower.pathBuilder().addPath(
                new BezierLine(
                        new Pose(42.500, 60.000),
                        new Pose(20.000, 60.000)
                )
        ).setConstantHeadingInterpolation(Math.toRadians(180)).build();

        Path8 = follower.pathBuilder().addPath(
                new BezierLine(
                        new Pose(20.000, 60.000),
                        new Pose(51.000, 9.000)
                )
        ).setConstantHeadingInterpolation(Math.toRadians(180)).build();

        Path9 = follower.pathBuilder().addPath(
                new BezierLine(
                        new Pose(51.000, 9.000),
                        new Pose(35.000, 9.000)
                )
        ).setConstantHeadingInterpolation(Math.toRadians(180)).build();
    }

    // ==================== OPMODE LIFECYCLE ====================

    @Override
    public void initialize() {
        // Initialize subsystems
        IntakeSubsystem intakeSub = new IntakeSubsystem(hardwareMap);
        TurretSubsystem turretSub = new TurretSubsystem(hardwareMap);
        LauncherSubsystem launcherSub = new LauncherSubsystem(hardwareMap, intakeSub);

        register(intakeSub, turretSub, launcherSub);

        // Reset pinpoint IMU
        GoBildaPinpointDriver pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        pinpoint.resetPosAndIMU();
        try {
            Thread.sleep(1000);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }

        // Initialize follower
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(51, 9, Math.toRadians(180)));

        buildPaths();

        // Build autonomous routine
        // Note: IntakeCommand only requires intakeSub.
        //       LaunchCommand requires launcherSub + turretSub.
        //       These don't overlap, so they CAN run in parallel if needed.
        //       Gate is accessed by launcherSub via reference (not requirement),
        //       so the scheduler won't block it.

        froggyroute = new SequentialCommandGroup(
                // --- Cycle 1: Launch preloaded balls ---
                new ParallelDeadlineGroup(
                        new WaitCommand(4000),
                        new LaunchCommand(launcherSub, turretSub)
                ),
                new InstantCommand(intakeSub::closeGate, intakeSub),

                // --- Cycle 2: Drive out, intake, drive back, launch ---
                new ParallelDeadlineGroup(
                        new SequentialCommandGroup(
                                new FollowPathCommand(follower, Path1),
                                new WaitCommand(500),
                                new FollowPathCommand(follower, Path2)
                        ),
                        new IntakeCommand(intakeSub)
                ),
                new ParallelDeadlineGroup(
                        new WaitCommand(4000),
                        new LaunchCommand(launcherSub, turretSub)
                ),
                new InstantCommand(intakeSub::closeGate, intakeSub),

                // --- Cycle 3: Drive to sample 2, intake, drive back, launch ---
                new FollowPathCommand(follower, Path3),
                new ParallelDeadlineGroup(
                        new FollowPathCommand(follower, Path4),
                        new IntakeCommand(intakeSub)
                ),
                new FollowPathCommand(follower, Path5),
                new ParallelDeadlineGroup(
                        new WaitCommand(4000),
                        new LaunchCommand(launcherSub, turretSub)
                ),
                new InstantCommand(intakeSub::closeGate, intakeSub),

                // --- Cycle 4: Drive to sample 3, intake, drive back, launch ---
                new FollowPathCommand(follower, Path6),
                new ParallelDeadlineGroup(
                        new FollowPathCommand(follower, Path7),
                        new IntakeCommand(intakeSub)
                ),
                new FollowPathCommand(follower, Path8),
                new ParallelDeadlineGroup(
                        new WaitCommand(4000),
                        new LaunchCommand(launcherSub, turretSub)
                ),
                new InstantCommand(intakeSub::closeGate, intakeSub),

                // --- Park ---
                new FollowPathCommand(follower, Path9),
                new InstantCommand(launcherSub::stop, launcherSub)
        );
    }

    @Override
    public void run() {
        if (!scheduled) {
            schedule(froggyroute);
            scheduled = true;
        }
        super.run();
        // NOTE: Do NOT call follower.update() here.
        // FollowPathCommand already calls follower.update() in its execute().
        // Calling it twice per loop causes double correction vectors.
    }
}
