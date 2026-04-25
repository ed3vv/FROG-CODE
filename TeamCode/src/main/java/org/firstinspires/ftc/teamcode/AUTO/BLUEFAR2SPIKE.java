package org.firstinspires.ftc.teamcode.AUTO;

import com.pedropathing.control.PIDFController;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.MathFunctions;
import com.pedropathing.math.Vector;
import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.ParallelDeadlineGroup;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.seattlesolvers.solverslib.controller.PIDController;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;
import com.seattlesolvers.solverslib.hardware.servos.ServoEx;
import com.seattlesolvers.solverslib.pedroCommand.FollowPathCommand;
import com.seattlesolvers.solverslib.util.TelemetryData;
import com.skeletonarmy.marrow.zones.Point;
import com.skeletonarmy.marrow.zones.PolygonZone;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.GLOBALS.globals;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.List;

@Autonomous
public class BLUEFAR2SPIKE extends CommandOpMode {
    private Follower follower;
    TelemetryData telemetryData = new TelemetryData(telemetry);
    private boolean scheduled = false;
    private SequentialCommandGroup froggyroute;
    public PathChain Path1, Path2,Path2half, Path3, Path4, Path5, Path6, Path7, Path8, Path9, Path10, Path11, Path12, Path13, Path14, Path15, Path16, Path17;

    //SUBSYSTEMS/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    public class IntakeSubsystem extends SubsystemBase {
        //DECLARE VARIABLES
        public final MotorEx intake, transfer;
        public final ServoEx gate;

        //THIS SECTION ACTS AS INIT, THUS WE PUT OUR HARDWARE INIT HERE
        public IntakeSubsystem(HardwareMap hardwareMap) {
            intake = new MotorEx(hardwareMap, "intake");
            intake.stopAndResetEncoder();
            intake.setRunMode(Motor.RunMode.RawPower);
            intake.setInverted(false);
            intake.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);
            intake.resetEncoder();

            transfer = new MotorEx(hardwareMap, "transfer");
            transfer.setRunMode(Motor.RunMode.RawPower);
            transfer.setInverted(true);
            transfer.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);

            gate = new ServoEx(hardwareMap, "gate");
            gate.set(globals.gate.close);
        }

        //ACTUAL FUNCTIONS ARE CREATED BELOW
        public void startIntake() {
            intake.set(-0.75);
            transfer.set(-1);
            gate.set(globals.gate.close);
        }

        public void stopIntake() {
            intake.set(0);
            transfer.set(0);
            gate.set(globals.gate.close);
        }

        public void feedLauncher() {
            gate.set(globals.gate.open);
            intake.set(-1);//TODO -0.7 IF FAR AUTO
            transfer.set(-1);
        }
    }

    public class OuttakeSubsystem extends SubsystemBase {
        public ServoEx turret1, turret2, hood;
        public MotorEx launcher1, launcher2;
        private final IntakeSubsystem intakeSub;
        private ElapsedTime timer = new ElapsedTime();
        private final PolygonZone closeLaunchZone = new PolygonZone(new Point(144, 144), new Point(72, 72), new Point(0, 144));
        private final PolygonZone robotZone = new PolygonZone(17, 17);
        public boolean tagReady = false, turretInRange, robotinZone = false, initialized = false, camTimerReset = false;
        private PIDController launchPIDF = new PIDController(globals.launcher.p, globals.launcher.i, globals.launcher.d);
        public double tagAng, turretAng, dist, offset, RPM, previousRPM, targetRPM, hoodAngle, lastTime, lastPosition, camTimer;

        public OuttakeSubsystem(HardwareMap hardwareMap, IntakeSubsystem intakeSub) {
            this.intakeSub = intakeSub;

            turret1 = new ServoEx(hardwareMap, "t1", 360);
            turret2 = new ServoEx(hardwareMap, "t2", 360);
            turret2.setInverted(true);
            turret1.setInverted(true);

            launcher1 = new MotorEx(hardwareMap, "l1", 28, 6000);
            launcher2 = new MotorEx(hardwareMap, "l2", 28, 6000);
            launcher1.setRunMode(Motor.RunMode.RawPower);
            launcher2.setRunMode(Motor.RunMode.RawPower);
            launcher2.setInverted(true);
            launcher1.setInverted(false);
            launcher1.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);
            launcher2.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);

            hood = new ServoEx(hardwareMap, "hood", 300);
        }




        public void turretaim(){
            if (turretInRange) {
                // only go out of range if it exceeds 155
                if (Math.abs(turretAng) > 155) {
                    turretInRange = false;
                    turretAng = 0;
                    turret1.set(180);
                    turret2.set(180);
                }
            } else {
                // only come back in range if it drops below 145
                if (Math.abs(turretAng) <= 145) {
                    turretInRange = true;
                } else {
                    turret1.set(180);
                    turret2.set(180);
                }
            }

            if (turretInRange) {
                double set = MathFunctions.clamp((180 - (turretAng * 1.054)), 25, 335);
                turret1.set(set);
                turret2.set(set);
            }
        }


        public void RPM() {
            double currentTime = getRuntime();
            int currentPosition = launcher1.getCurrentPosition();

            double deltaTime = currentTime - lastTime;
            double deltaTicks = currentPosition - lastPosition;

            if (deltaTime > 0.02) {
                previousRPM = RPM;
                double revs = deltaTicks / 28.0; // GoBILDA CPR
                RPM = (revs / deltaTime) * 60.0;

                lastTime = currentTime;
                lastPosition = currentPosition;
            }
        }

        private void launchCalc() {
            double x = follower.getPose().getX();
            double y = follower.getPose().getY();
            Pose robot = new Pose(x, y);
            robotZone.setPosition(x, y);
            robotZone.setRotation(follower.getPose().getHeading());
            Pose goal = new Pose(globals.turret.goalX, globals.turret.goalY);

            Pose target = goal.minus(robot);
            Vector robotToGoal = target.getAsVector();
            double goalAngle = Math.atan2(goal.getY() - y, goal.getX() - x);

            turretAng = Math.toDegrees(AngleUnit.normalizeRadians(follower.getHeading() - goalAngle));
            dist = robotToGoal.getMagnitude();

            if (robotZone.isInside(closeLaunchZone)) {
                launchPIDF.setTolerance(230);
                if (dist < 55) {
                    targetRPM = 14 * dist + 2010;
                    hoodAngle = 4.2 * dist - 159;
                } else if (dist  < 75) {
                    targetRPM = -1.1429 * Math.pow(dist, 2) + 172 * dist - 3322.9;
                    hoodAngle = 100;
                } else {
                    targetRPM = 20 * dist + 1600;
                    hoodAngle = 2 * dist -50;
                }
            } else {
                launchPIDF.setTolerance(230);
                if (follower.getPose().getY() < 56) {
                    targetRPM = 3300;
                    hoodAngle = 120;
                } else {
                    targetRPM = 4300;
                    hoodAngle = 120;
                }
            }

        }

        @Override
        public void periodic() {
            RPM();
            launchCalc();}
    }

    //COMMANDS///////////////////////////////////////////////////////////////////////////////////////////////////////////////////



    //PATHS//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    public void buildpath(){
        Path1 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(45.000, 9.000),

                                new Pose(11.000, 9.000)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(180))

                .build();

        Path2 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(11.000, 9.000),

                                new Pose(45.000, 9.000)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(180))

                .build();

        Path3 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(45.000, 9.000),

                                new Pose(42.000, 35.000)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(180))

                .build();

        Path4 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(42.000, 35.000),

                                new Pose(23.000, 35.000)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(180))

                .build();

        Path5 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(23.000, 35.000),

                                new Pose(45.000, 9.000)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(180))

                .build();

        Path6 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(45.000, 9.000),

                                new Pose(42.000, 60.000)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(180))

                .build();

        Path7 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(42.000, 60.000),

                                new Pose(23.000, 60.000)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(180))

                .build();

        Path8 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(23.000, 60.000),

                                new Pose(45.000, 9.000)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(180))

                .build();

        Path9 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(45.000, 9.000),

                                new Pose(11.000, 9.000)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(180))

                .build();

        Path10 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(11.000, 9.000),

                                new Pose(45.000, 9.000)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(180))

                .build();

        Path11 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(45.000, 9.000),

                                new Pose(11.000, 9.000)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(180))

                .build();

        Path12 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(11.000, 9.000),

                                new Pose(45.000, 9.000)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(180))

                .build();

        Path13 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(45.000, 9.000),

                                new Pose(11.000, 9.000)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(180))

                .build();

        Path14 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(11.000, 9.000),

                                new Pose(45.000, 9.000)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(180))

                .build();

        Path15 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(45.000, 9.000),

                                new Pose(11.000, 9.000)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(180))

                .build();

        Path16 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(11.000, 9.000),

                                new Pose(45.000, 9.000)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(180))

                .build();

        Path17 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(45.000, 9.000),

                                new Pose(33.000, 9.000)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(180))

                .build();
    }

    @Override
    public void initialize() {
        //PINPOINT INITIALIZATION, CALIBRATES OUR HARDWARE BEFORE RUNNING
        GoBildaPinpointDriver pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        pinpoint.resetPosAndIMU();

        long start = System.currentTimeMillis();
        while (System.currentTimeMillis() - start < 1000) {
        }

        //SET INITIAL POSITION AFTER HARDWARE CALIBRATION
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(45, 9, Math.toRadians(180)));//TODO

        IntakeSubsystem intakeSub = new IntakeSubsystem(hardwareMap);
        OuttakeSubsystem outtakeSub = new OuttakeSubsystem(hardwareMap, intakeSub);
        register(intakeSub, outtakeSub);

        buildpath();

        //AUTONOMOUS ROUTINE.
        froggyroute = new SequentialCommandGroup(
                new FollowPathCommand(follower, Path1),
                new WaitCommand(1500),
                new FollowPathCommand(follower, Path2, false),
                new FollowPathCommand(follower, Path2half),
                new FollowPathCommand(follower, Path3),
                new WaitCommand(1500),
                new FollowPathCommand(follower, Path4),
                new FollowPathCommand(follower, Path5),
                new WaitCommand(1500),
                new FollowPathCommand(follower, Path6),
                new FollowPathCommand(follower, Path7),
                new WaitCommand(1500),
                new FollowPathCommand(follower, Path8),
                new FollowPathCommand(follower, Path9),
                new WaitCommand(1500),
                new FollowPathCommand(follower, Path10),
                new FollowPathCommand(follower, Path11),
                new WaitCommand(1500),
                new FollowPathCommand(follower, Path12),
                new FollowPathCommand(follower, Path13),
                new WaitCommand(1500)
        );
    }

    @Override
    public void run(){
        if (!scheduled) {
            schedule(froggyroute);
            scheduled = true;
        }
        super.run();
        follower.update();
    }
}