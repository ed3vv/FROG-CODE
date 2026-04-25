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

import org.firstinspires.ftc.teamcode.GLOBALS.globals;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous
public class BLUE18CLOSE extends CommandOpMode {
    private Follower follower;
    TelemetryData telemetryData = new TelemetryData(telemetry);
    private boolean scheduled = false;
    private SequentialCommandGroup froggyroute;
    public PathChain Path1, Path2, Path3, Path4, Path5, Path6, Path7, Path8, Path9, Path10, Path11, Path12, Path13, Path14;


    //paths
    public void buildpath(){
        Path1 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(22.364, 118.172),
                                new Pose(50.000, 90.000)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        Path2 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(50.000, 90.000),
                                new Pose(42.000, 59.000)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        Path3 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(42.000, 59.000),
                                new Pose(23.000, 59.000)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        Path4 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(23.000, 59.000),
                                new Pose(55.000, 76.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(150))
                .build();

        Path5 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(55.000, 76.000),
                                new Pose(12.500, 60.000)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(150))
                .build();

        Path6 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(12.500, 60.000),
                                new Pose(55.000, 76.000)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(150))
                .build();

        Path7 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(55.000, 76.000),
                                new Pose(12.500, 60.000)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(150))
                .build();

        Path8 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(12.500, 60.000),
                                new Pose(55.000, 76.000)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(150))
                .build();

        Path9 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(55.000, 76.000),
                                new Pose(42.000, 82.500)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(150), Math.toRadians(180))
                .build();

        Path10 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(42.000, 82.500),
                                new Pose(23.000, 82.500)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        Path11 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(23.000, 82.500),
                                new Pose(45.000, 82.500)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        Path12 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(45.000, 82.500),
                                new Pose(42.000, 35.000)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        Path13 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(42.000, 35.000),
                                new Pose(23.000, 35.000)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        Path14 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(23.000, 35.000),
                                new Pose(58.000, 106.000)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(180))
                .build();
    }

    //subsystems
    public class everythingsubsys extends SubsystemBase {
        public MotorEx intake, transfer, launcher1, launcher2;
        public ServoEx gate, turret1, turret2, hood;
        private PIDController launchPID = new PIDController(globals.launcher.p, globals.launcher.i, globals.launcher.d);
        private double lastTime, launchPower, RPM, previousRPM, lastPosition;
        public everythingsubsys(HardwareMap hardwareMap){
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

            turret1 = new ServoEx(hardwareMap, "t1", 0, 360);
            turret2 = new ServoEx(hardwareMap, "t2", 0, 360);
            turret1.setInverted(true);
            turret2.setInverted(true);

            launcher1 = new MotorEx(hardwareMap, "l1", 28, 6000);
            launcher2 = new MotorEx(hardwareMap, "l2", 28, 6000);
            launcher1.setRunMode(Motor.RunMode.RawPower);
            launcher2.setRunMode(Motor.RunMode.RawPower);
            launcher2.setInverted(true);
            launcher1.setInverted(false);
            launcher1.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);
            launcher2.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);

            hood = new ServoEx(hardwareMap, "hood", 300);

            launchPID.setTolerance(50);
        }

        public void intakestart(){
            intake.set(1);
            transfer.set(0.5);
            gate.set(globals.gate.close);
        }

        public void intakefinish(){
            intake.set(0);
            transfer.set(0);
        }

        public void launchfinish(){
            launcher1.set(0.2);
            launcher2.set(0.2);
        }

        public void launch(){
            //TODO
        }

        public void setup() {
            //TODO
        }

        public void RPM() {
            double currentTime = getRuntime();
            int currentPosition = launcher2.getCurrentPosition();

            double deltaTime = currentTime - lastTime;
            double deltaTicks = currentPosition - lastPosition;

            if (deltaTime > 0.02) {
                previousRPM = RPM;
                double revs = deltaTicks / 28.0; // GoBUILDA CPR
                RPM = -(revs / deltaTime) * 60.0;

                lastTime = currentTime;
                lastPosition = currentPosition;
            }
        }

        @Override
        public void periodic() {
            RPM();
        }
    }
    //commandbase
    public static class froggyeat extends CommandBase {
        public everythingsubsys everythingsubsystem;
        public froggyeat(everythingsubsys everythingsubsystem){
            this.everythingsubsystem = everythingsubsystem;
            addRequirements(everythingsubsystem);
        }

        @Override
        public void initialize(){
            everythingsubsystem.intakestart();
        }

        @Override
        public void end(boolean interrupted){
            everythingsubsystem.intakefinish();
        }
    }

    public static class froggyspit extends CommandBase{
        public everythingsubsys everythingsubsystem;
        public froggyspit(everythingsubsys everythingsubsystem){
            this.everythingsubsystem = everythingsubsystem;
            addRequirements(everythingsubsystem);
        }

        @Override
        public void initialize(){
            everythingsubsystem.setup();
        }

        @Override
        public void execute(){
            everythingsubsystem.launch();
        }

        @Override
        public void end(boolean interrupted){
            everythingsubsystem.launchfinish();
        }
        //TODO ADD LAUNCH
    }

    @Override
    public void initialize() {
        everythingsubsys everythingsubsystem = new everythingsubsys(hardwareMap);
        register(everythingsubsystem);

        GoBildaPinpointDriver pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        pinpoint.resetPosAndIMU();

        long start = System.currentTimeMillis();
        while (System.currentTimeMillis() - start < 1000) {
        }

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(22.364, 118.172, Math.toRadians(180)));
        buildpath();


        froggyroute = new SequentialCommandGroup(
                new FollowPathCommand(follower, Path1),
                new ParallelDeadlineGroup(
                        new WaitCommand(4000),
                        new froggyspit(everythingsubsystem)
                ),
                new FollowPathCommand(follower, Path2),
                new ParallelDeadlineGroup(
                        new FollowPathCommand(follower, Path3),
                        new froggyeat(everythingsubsystem)
                ),
                new FollowPathCommand(follower, Path4),
                new ParallelDeadlineGroup(
                        new FollowPathCommand(follower, Path5),
                        new froggyeat(everythingsubsystem)
                ),
                new FollowPathCommand(follower, Path6),
                new ParallelDeadlineGroup(
                        new WaitCommand(4000),
                        new froggyspit(everythingsubsystem)
                ),
                new ParallelDeadlineGroup(
                        new FollowPathCommand(follower, Path7),
                        new froggyeat(everythingsubsystem)
                ),
                new FollowPathCommand(follower, Path8),
                new ParallelDeadlineGroup(
                        new WaitCommand(4000),
                        new froggyspit(everythingsubsystem)
                ),
                new FollowPathCommand(follower, Path9),
                new ParallelDeadlineGroup(
                        new FollowPathCommand(follower, Path10),
                        new froggyeat(everythingsubsystem)
                ),
                new FollowPathCommand(follower, Path11),
                new ParallelDeadlineGroup(
                        new WaitCommand(4000),
                        new froggyspit(everythingsubsystem)
                ),
                new FollowPathCommand(follower, Path12),
                new ParallelDeadlineGroup(
                        new FollowPathCommand(follower, Path13),
                        new froggyeat(everythingsubsystem)
                ),
                new FollowPathCommand(follower, Path14),
                new ParallelDeadlineGroup(
                        new WaitCommand(4000),
                        new froggyspit(everythingsubsystem)
                )

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
