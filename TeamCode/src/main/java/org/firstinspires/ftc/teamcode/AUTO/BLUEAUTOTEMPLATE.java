package org.firstinspires.ftc.teamcode.AUTO;

import com.pedropathing.control.PIDFController;
import com.pedropathing.follower.Follower;
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
public class BLUEAUTOTEMPLATE extends CommandOpMode {
    private static Follower follower;
    TelemetryData telemetryData = new TelemetryData(telemetry);
    private boolean scheduled = false;
    private SequentialCommandGroup froggyroute;
    public PathChain Path1, Path2, Path3, Path4, Path5, Path6, Path7, Path8, Path9, Path10, Path11, Path12, Path13, Path14;

    //SUBSYSTEMS/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    public static class IntakeSubsystem extends SubsystemBase {
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

    public static class OuttakeSubsystem extends SubsystemBase {
        public ServoEx turret1, turret2, hood;
        public MotorEx launcher1, launcher2;
        private Limelight3A limelight;
        private final PolygonZone closeLaunchZone = new PolygonZone(new Point(144, 144), new Point(72, 72), new Point(0, 144));
        private final PolygonZone robotZone = new PolygonZone(17, 17);
        public boolean tagReady = false, turretInRange, camTimerReset = false, robotinZone = false, initialized = false;
        private ElapsedTime timer = new ElapsedTime();
        public double tagAng, turretAng, dist, offset, RPM, previousRPM, targetRPM, hoodAngle, lastTime, lastPosition, camTimer;

        public OuttakeSubsystem(HardwareMap hardwareMap) {
            turret1 = new ServoEx(hardwareMap, "t1", 360);
            turret2 = new ServoEx(hardwareMap, "t2", 360);
            turret2.setInverted(true);
            turret1.setInverted(true);

            limelight = hardwareMap.get(Limelight3A.class, "limelight");
            limelight.setPollRateHz(85);
            limelight.pipelineSwitch(0);
            limelight.start();

        }

        public void setTurret() {
            tagReady = false;
            LLResult result = limelight.getLatestResult();
            if (result != null && result.isValid()) {
                if (result.getStaleness() < 500) {
                    List<LLResultTypes.FiducialResult> tags = result.getFiducialResults();
                    if (!tags.isEmpty()) {
                        for (LLResultTypes.FiducialResult tag : tags) {
                            tagAng = tag.getTargetXDegrees();
                            tagReady = true;
                        }
                    }
                }
            }

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
                double set = MathFunctions.clamp((180 - turretAng + offset) * 1.03, 25, 335);
                if (tagReady && !camTimerReset) {
                    camTimer = timer.seconds();
                    camTimerReset = true;
                } else if (!tagReady) {
                    camTimerReset = false;
                }
                if (tagReady && Math.abs(tagAng) > 1.5 && camTimer + 0.2 < timer.seconds() && follower.getAngularVelocity() < 0.5 && follower.getVelocity().getMagnitude() < 5) {
                    offset -= globals.turret.camP * tagAng;
                }
                turret1.set(set);
                turret2.set(set);
            }
        }

        private void updateRPM() {
                double currentTime = System.nanoTime() / 1_000_000_000.0;
                int currentPosition = launcher2.getCurrentPosition();

                if (!initialized) {
                    lastTime = currentTime;
                    lastPosition = currentPosition;
                    initialized = true;
                    return;
                }

                double deltaTime = currentTime - lastTime;
                double deltaTicks = currentPosition - lastPosition;

                if (deltaTime > 0.02) {
                    double revs = deltaTicks / 28.0;
                    RPM = -(revs / deltaTime) * 60.0;
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
            Pose goal = new Pose(globals.turret.goalX,  globals.turret.goalY);

            Pose target = goal.minus(robot);
            Vector robotToGoal = target.getAsVector();
            double goalAngle = Math.atan2(goal.getY() - y, goal.getX() - x);

            turretAng = Math.toDegrees(AngleUnit.normalizeRadians(follower.getHeading() - goalAngle));
            dist = robotToGoal.getMagnitude();


            if (robotZone.isInside(closeLaunchZone)) {
                robotinZone = true;
            }

            targetRPM = globals.tuning.targetRPM;
            hoodAngle = globals.tuning.ang;
        }

        @Override
        public void periodic() {
                updateRPM();
                launchCalc();
        }
    }

    //COMMANDS///////////////////////////////////////////////////////////////////////////////////////////////////////////////////



    //PATHS//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    public void buildpath(){
        Path1 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(51.000, 9.000),

                                new Pose(11.000, 9.000)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(180))

                .build();

        Path2 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(11.000, 9.000),

                                new Pose(51.000, 9.000)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(180))

                .build();

        Path3 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(51.000, 9.000),

                                new Pose(42.500, 35.500)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(180))

                .build();

        Path4 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(42.500, 35.500),

                                new Pose(20.000, 35.500)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(180))

                .build();

        Path5 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(20.000, 35.500),

                                new Pose(51.000, 9.000)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(180))

                .build();

        Path6 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(51.000, 9.000),

                                new Pose(42.500, 60.000)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(180))

                .build();

        Path7 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(42.500, 60.000),

                                new Pose(20.000, 60.000)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(180))

                .build();

        Path8 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(20.000, 60.000),

                                new Pose(51.000, 9.000)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(180))

                .build();
        Path9 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(51.000, 9.000),

                                new Pose(35.000, 9.000)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(180))

                .build();
    }

    @Override
    public void initialize() {



        //PINPOINT INITIALIZATION, CALIBRATES OUR HARDWARE BEFORE RUNNING
        GoBildaPinpointDriver pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        pinpoint.resetPosAndIMU();
        try {
            Thread.sleep(1000);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }

        //SET INITIAL POSITION AFTER HARDWARE CALIBRATION
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(51, 9, Math.toRadians(180)));//TODO

        buildpath();

        //AUTONOMOUS ROUTINE.
        froggyroute = new SequentialCommandGroup(

        );
    }

    @Override
    public void run(){
        if (!scheduled) {
            schedule(froggyroute);
            scheduled = true;
        }
        super.run();
    }
}
