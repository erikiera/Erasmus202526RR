package org.firstinspires.ftc.teamcode.Robot;

import android.graphics.Color;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.MecanumDrive;

@Config
public class ErasmusRobot {
    // ==================== Declarations =========================
    // Shooter
    public DcMotorEx shooterMotor ;
    public static double SHOOTERSPEED = 1400;
    public static double SHOOTERMARGIN = 25 ;
    public static double SHOOTERCUTOFFTIME = 4000 ;  // Milliseconds  // TODO: Remove ??
    public static double LEVELTIME = 1000 ;   // TODO: Remove ??
    public static double SHOOTERPIDP = 7 ;
    public static double SHOOTERPIDI = 3 ;
    public static double SHOOTERPIDD = 3 ;
    public static double SHOOTERPIDF = 0 ;
    public static double ALPHA = 0.1 ;
    public static double KP = 0.0007 ;
    public static double SLEWRATE = 50 ;
    public static double KV = 1.0/2160 ;  // TODO: Adjust this so we get there
    // Feeder
    public Servo feedServo ;
    public static double FEEDERUP = 0.40 ;
    public static double FEEDERDOWN = 0.22 ;
    public static double FEEDERDROPTIME = 2 ;
    public boolean feederFired = false ; //TODO: Do we need this??
    // Indexer
    public Servo indexerServo ;
    public ColorSensor colorSensor ;
    public DistanceSensor distanceSensor ;
    public int indexerCurrentPosition = 0 ;
    public int[] indexerContents = {0,0,0} ;
    public static double[] indexerPositions = {0.319, 0.390, 0.47} ;
    public double[] [] [] shootOrder = {
            // Pattern | Green Index | Order
            { {0,1,2}, {1,0,2}, {2,1,0} },
            { {1,0,2}, {0,1,2}, {0,2,1} },
            { {2,1,0}, {0,2,1}, {0,1,2} }
    } ;
    public static double greenIndex = 1 ;
    public static double pattern = 0 ;
    public double[] aprilTagLookup = {0,0,1,2,3,4} ;
    public double lastDistanceReading = 6 ;
    public static double DISTANCE = 2.0;
    // Intake
    DcMotorEx intakeMotor ;
    public static double INTAKEPOWER = 0.8;
    // HuskyLens
    private HuskyLens huskyLens;
    // OpMode / Robot
    private HardwareMap hardwareMap ;
    private OpMode opMode ;
    private Telemetry telemetry ;
    public MecanumDrive drive ;
    // ================ Constructors ================================
    public ErasmusRobot(OpMode newOpMode, Pose2d newPose) {
        // Robot
        opMode = newOpMode ;
        hardwareMap = opMode.hardwareMap ;
        telemetry = opMode.telemetry ;
        drive = new MecanumDrive(hardwareMap, newPose);
//        drive.localizer.setPose(newPose);
        // Shooter
        shooterMotor = hardwareMap.get(DcMotorEx.class, "shooterMotor");
        shooterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooterMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        shooterMotor.setVelocity(0);
        shooterMotor.setVelocityPIDFCoefficients(SHOOTERPIDP, SHOOTERPIDI, SHOOTERPIDD, SHOOTERPIDF);
        // Feeder
        feedServo = hardwareMap.get(Servo.class, "feedServo") ;
        // Indexer
        indexerServo = hardwareMap.get(Servo.class, "indexerServo") ;
        colorSensor = hardwareMap.get(ColorSensor.class, "colorSensor") ;
        distanceSensor = hardwareMap.get(DistanceSensor.class, "distanceSensor") ;
        // Intake
        intakeMotor = hardwareMap.get(DcMotorEx.class, "intakeMotor");
        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        // HuskyLens
        huskyLens = hardwareMap.get(HuskyLens.class, "huskylens");
        huskyLens.selectAlgorithm(HuskyLens.Algorithm.TAG_RECOGNITION);

        telemetry.addLine("----- Robot Initialized !!! -----") ;
    }
    public ErasmusRobot(OpMode newOpMode) {
        this(newOpMode, new Pose2d(0,0,0)) ;
    }

    // ==================== Methods =======================
    public void shutdownShooter() {
        shooterMotor.setVelocity(0) ;        // Stop motor
        feedServo.setPosition(FEEDERDOWN);   // Scissor lift down
    }
    public void moveIndexer(int position) {
        indexerServo.setPosition(indexerPositions[checkIndex(position)]);
        indexerCurrentPosition = position ;
    }
    public int checkIndex(int position) {
        if (position > 1) return 2 ;
        else if (position < 1) return 0 ;
        return position ;
    }
    public void resetIndexer() {  // TODO: Double-check this new way of using this and moveIndexer
//        indexerServo.setPosition(indexerPositions[0]);
        moveIndexer(0) ;
//        indexerCurrentPosition = 0 ;
    }
    public float readColorSensorHSV() {
        float[] hsvValues = new float[3];
        Color.RGBToHSV(colorSensor.red(), colorSensor.green(), colorSensor.blue(), hsvValues);
        return hsvValues[0] ;
    }
    public boolean ballPresent() {
        return (distanceSensor.getDistance(DistanceUnit.INCH)<3.1) ;
    }
    public void startIntake() {
        intakeMotor.setPower(INTAKEPOWER);
    }
    public void stopIntake() {
        intakeMotor.setPower(0);
    }
    public double huskyReadPattern() {
        HuskyLens.Block[] blocks = huskyLens.blocks();
        for (HuskyLens.Block block : blocks ) {
            if (block.id>0 && block.id<4) pattern = aprilTagLookup[block.id] ;
        }
        return pattern ;
    }
    double slewSpeed(double target, double current) {
        if (Math.abs(target-current)>SLEWRATE) {
            if (target>current) return current+SLEWRATE ;
            else return current - SLEWRATE ;
        }
        return target ;
    }

    // =================== Actions =======================
    // --------- Higher Level Actions --------------
    public Action shootBall() { // Revs motor and feeds 1 ball
        return new SequentialAction(
                new ParallelAction(
                    new ShooterControl(),
                    new SleepAction(0.8)
                ),
                    feedAction()
                ) ;
    }
    public Action shootBlank() { // Revs motor and feeds 1 ball
        return new SequentialAction(
//                new StartShooter(),
                new ShooterControl(),
                new SleepAction(0.8)
        ) ;
    }

    public Action feedAction() {
        return new SequentialAction(
                new InstantAction(()->feedServo.setPosition(FEEDERUP)),
                new SleepAction(0.5),
                new InstantAction(()->feedServo.setPosition(FEEDERDOWN)),
                new SleepAction(0.5)
        ) ;
    }

    public Action stageAll() {
        return new SequentialAction(
                new InstantAction(()->moveIndexer(0)),
                new SleepAction(0.9),
                feedAction(),
                new InstantAction(()->moveIndexer(1)),
                new SleepAction(0.5),
                feedAction(),
                new InstantAction(()->moveIndexer(2)),
                new SleepAction(0.5),
                feedAction(),
                new InstantAction(()->resetIndexer())
        ) ;
    }

    public Action shootAll() { // Shoots all 3 balls and resets intake
        return new SequentialAction(
                new InstantAction(()->moveIndexer((int) shootOrder[(int)pattern][(int)greenIndex][0])),
//                new SleepAction(0.3),
                shootBall(),
                new InstantAction(()->moveIndexer((int) shootOrder[(int)pattern][(int)greenIndex][1])),
//                new SleepAction(0.5), // TODO: We could have a greenlight flag in the robot class
                shootBall(),
                new InstantAction(()->moveIndexer((int) shootOrder[(int)pattern][(int)greenIndex][2])), new SleepAction(0.5),
                shootBall(),
                new SleepAction(0.7),
                new InstantAction(()->shutdownShooter()),
                new InstantAction(()->resetIndexer())
        ) ;
    }

    public Action intakeAll() {
        return new SequentialAction(
                new InstantAction(()->moveIndexer(0)),
                new SleepAction(0.5),
                new IntakeOne(),
                new InstantAction(()->moveIndexer(1)),
                new SleepAction(0.5),
                new IntakeOne(),
                new InstantAction(()->moveIndexer(2)),
                new SleepAction(0.5),
                new IntakeOne(),
                new SleepAction(0.5),
                new InstantAction(()->moveIndexer(0))
        ) ;
    }

    public Action newShoot() {
        return new SequentialAction(
                new ShooterControl()
        ) ;
    }

    // ---------- Lower Level Actions -------------
    private class StartShooter implements Action {
        private boolean started = false;
        private double targetSpeed = SHOOTERSPEED;
        private double startTime = 0;
        private double levelTime = 0 ;
        private double cutoffTime = SHOOTERCUTOFFTIME;
        // Constructors
        public StartShooter() { }
        public StartShooter(double newTargetSpeed) {
            this();
            targetSpeed = newTargetSpeed; // TODO: Create way to alter this during the opMode. Telemetry packet??
        }
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            telemetry.addData("Shooter Velocity", shooterMotor.getVelocity());
            if (!started) {
                started = true;
                shooterMotor.setVelocity(targetSpeed);
                startTime = opMode.getRuntime();
                telemetry.addLine("+++ Shooter Revving Up +++");
                return true;
            } else if (Math.abs(targetSpeed - shooterMotor.getVelocity()) < SHOOTERMARGIN) {
                // We are at the desired speed.
                if (levelTime == 0) levelTime = opMode.getRuntime() ;
                return (opMode.getRuntime() > levelTime + LEVELTIME) ;
//                return false; // Ends this action
            } else if (opMode.getRuntime() < startTime+cutoffTime) {
                levelTime = 0 ;
                return true ;
            }
            // If we started and we are not fully revved, do nothing but wait
            return false;
        }
    }

    private class ShooterControl implements Action {
        private boolean started = false;
        private double targetSpeed = SHOOTERSPEED;
        private double startTime = 0;
        private double levelTime = 0;
        private double cutoffTime = SHOOTERCUTOFFTIME;
        double smoothed = 0;
        // Constructors
        public ShooterControl() {
        }
        public ShooterControl(double newTargetSpeed) {
            this();
            targetSpeed = newTargetSpeed; // TODO: Create way to alter this during the opMode. Telemetry packet??
        }
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if (!started) {
                started = true;
                startTime = opMode.getRuntime();
                targetSpeed = SHOOTERSPEED;
                shooterMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }
            telemetry.addData("Shooter Velocity", shooterMotor.getVelocity());
            smoothed = (1 - ALPHA) * smoothed + ALPHA * shooterMotor.getVelocity();
            if (Math.abs(targetSpeed - smoothed) < SHOOTERMARGIN) {
                telemetry.addData("Time to Shoot", opMode.getRuntime()-startTime) ;
                return false ;
            }
                // We are at the desired speed.
//                if (levelTime == 0) levelTime = opMode.getRuntime();
//                else if (opMode.getRuntime() > levelTime + LEVELTIME) return false;
//            } else levelTime = 0;
            shooterMotor.setPower(targetSpeed * KV + (targetSpeed - smoothed) * KP);
            return true;
        }
    }

    private class ShooterControl2 implements Action {
        private boolean started = false;
        private double targetSpeed = SHOOTERSPEED;
        private double startTime = 0;
        private double levelTime = 0;
        private double cutoffTime = SHOOTERCUTOFFTIME;
        double smoothed = 0;
        double slewedTarget ;
        // Constructors
        public ShooterControl2() {
        }
        public ShooterControl2(double newTargetSpeed) {
            this();
            targetSpeed = newTargetSpeed; // TODO: Create way to alter this during the opMode. Telemetry packet??
        }
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if (!started) {
                started = true;
                startTime = opMode.getRuntime();
                targetSpeed = SHOOTERSPEED;
                shooterMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }
            telemetry.addData("Shooter Speed (Raw)", shooterMotor.getVelocity());
            smoothed = (1 - ALPHA) * smoothed + ALPHA * shooterMotor.getVelocity();
            if (Math.abs(targetSpeed - smoothed) < SHOOTERMARGIN) {
                telemetry.addData("Time to Shoot", opMode.getRuntime()-startTime) ;
                return false ;
            }
            slewedTarget = slewSpeed(targetSpeed, smoothed) ;
            shooterMotor.setPower(slewedTarget * KV + (slewedTarget - smoothed) * KP);
            return true;
        }
    }


    private class IntakeOne implements Action {
        private boolean started = false;
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if (!started) {
                started = true;
                lastDistanceReading = 6 ;
                startIntake() ;
            }
            lastDistanceReading = 0.1*distanceSensor.getDistance(DistanceUnit.INCH) + 0.9*lastDistanceReading ;

            if (lastDistanceReading<DISTANCE) { //TODO: Because the REV Distance Sensor is a piece of crap
//                double startTime = opMode.getRuntime() ;
//                while (opMode.getRuntime() < startTime + 0.2) {}
                if (readColorSensorHSV() < 180) greenIndex = indexerCurrentPosition ;
                stopIntake() ;
                return false ;
            } else return true ;
        }
    }

    private class AdvanceIndexer implements Action {
        private boolean started = false;
        private double startTime = 0;

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            telemetry.addData("Shooter Velocity", shooterMotor.getVelocity());
            if (!started) {
                started = true;
            }
            return true;
        }
    }

    private class ResetIndexer implements Action {
        private boolean started = false;
        private double startTime = 0;

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            telemetry.addData("Shooter Velocity", shooterMotor.getVelocity());
            if (!started) {
                started = true;
            }
            return true;
        }
    }


    // Maybe overkill compared to the lambda method
    private class FeedShooter implements Action{
        private boolean started = false;
        private double startTime = 0;
        private double dropTime = FEEDERDROPTIME;

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if (!started) {
                started = true;
                feedServo.setPosition(FEEDERUP);
                startTime = opMode.getRuntime() ;
            }
            else if (opMode.getRuntime() > startTime+0.5) return false ;
            return true ;
        }
    }


}
