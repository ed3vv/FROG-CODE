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
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;
import com.seattlesolvers.solverslib.hardware.servos.ServoEx;
import com.seattlesolvers.solverslib.pedroCommand.FollowPathCommand;
import com.seattlesolvers.solverslib.util.TelemetryData;

import org.firstinspires.ftc.teamcode.GLOBALS.globals;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous
public class BLUE12FAR extends CommandOpMode {
    private Follower follower;
    TelemetryData telemetryData = new TelemetryData(telemetry);
    private boolean scheduled = false;
    private SequentialCommandGroup froggyroute;
    public PathChain Path1, Path2, Path3, Path4, Path5, Path6, Path7, Path8, Path9, Path10, Path11, Path12, Path13, Path14;


    //paths
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

    //subsystems
    public class everythingsubsys extends SubsystemBase {
        public MotorEx intake, transfer, launcher1, launcher2;
        public ServoEx gate, turret1, turret2, hood;

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

        public void intakestart(){
            intake.set(1);
            transfer.set(0.5);
            gate.set(globals.gate.close);
        }

        public void intakefinish(){
            intake.set(0);
            transfer.set(0);
        }

        public void launch(){
            //TODO
        }

        public void setup(){
            hood.set(20);
            turret1.set(288);
            turret2.set(288);
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
        //TODO ADD LAUNCH
    }

    @Override
    public void initialize() {
        everythingsubsys everythingsubsystem = new everythingsubsys(hardwareMap);

        GoBildaPinpointDriver pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        pinpoint.resetPosAndIMU();

        long start = System.currentTimeMillis();
        while (System.currentTimeMillis() - start < 1000) {
        }

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(51, 9, Math.toRadians(180)));//TODO

        buildpath();

        froggyroute = new SequentialCommandGroup(
                new ParallelDeadlineGroup(
                        new WaitCommand(4000),
                        new froggyspit(everythingsubsystem)
                ),
                new ParallelDeadlineGroup(
                        new SequentialCommandGroup(
                                new FollowPathCommand(follower, Path1),
                                new WaitCommand(500),
                                new FollowPathCommand(follower, Path2)
                                ),
                        new froggyeat(everythingsubsystem)
                ),
                new ParallelDeadlineGroup(
                        new WaitCommand(4000),
                        new froggyspit(everythingsubsystem)
                ),
                new FollowPathCommand(follower, Path3),
                new ParallelDeadlineGroup(
                        new FollowPathCommand(follower, Path4),
                        new froggyeat(everythingsubsystem)
                ),
                new FollowPathCommand(follower, Path5),
                new ParallelDeadlineGroup(
                        new WaitCommand(4000),
                        new froggyspit(everythingsubsystem)
                ),
                new FollowPathCommand(follower, Path6),
                new ParallelDeadlineGroup(
                        new FollowPathCommand(follower, Path7),
                        new froggyeat(everythingsubsystem)
                ),
                new FollowPathCommand(follower, Path8),
                new ParallelDeadlineGroup(
                        new WaitCommand(4000),
                        new froggyspit(everythingsubsystem)
                ),
                new FollowPathCommand(follower, Path9)

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
