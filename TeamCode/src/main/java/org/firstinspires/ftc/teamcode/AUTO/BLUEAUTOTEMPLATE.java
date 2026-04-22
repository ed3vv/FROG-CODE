package org.firstinspires.ftc.teamcode.AUTO;

import com.pedropathing.control.PIDFController;
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
public class BLUEAUTOTEMPLATE extends CommandOpMode {
    private Follower follower;
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

        //ACTUAL FUNCTIONS ARE CREATED BELOW
        public void startIntake() {
            intake.set(1);
            transfer.set(0.5);
            gate.set(globals.gate.close);
        }

        public void stopIntake() {
            intake.set(0);
            transfer.set(0);
        }

        public void feedLauncher() {
            gate.set(globals.gate.open);
            intake.set(0.6);
            transfer.set(0.6);
        }

        public void closeGate() {
            gate.set(globals.gate.close);
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
