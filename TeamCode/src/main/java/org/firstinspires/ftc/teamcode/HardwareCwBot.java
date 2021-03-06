package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.hardware.stmicroelectronics.VL53L0X;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchDevice;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Quaternion;

public class HardwareCwBot
{
    /* Public OpMode members. */
    public DcMotor backRight = null;
    public DcMotor backLeft = null;
    public DcMotor frontRight = null;
    public DcMotor frontLeft = null;
    public Servo phoneServo = null;
    public DistanceSensor rangeSensor = null;
    public MyVL53L0X rev2M = null;
    DeviceInterfaceModule dim;
    AnalogInput dsFront;
    AnalogInput dsLeft;
    AnalogInput revPotentiometer;

    double systemVoltage;
    DigitalChannel armEndSwitch;

    public static final double PHONE_VERTICAL =  0.4 ;
    public static final double PHONE_AT_45 =  0.2 ;

    // The IMU sensor object
    BNO055IMU imu;
    BNO055IMU.Parameters imuParameters;

    /* local OpMode members. */
    HardwareMap hwMap           =  null;
    private ElapsedTime period  = new ElapsedTime();

    DcMotor[] allMotors;
    static final int FRONTLEFT = 0;
    static final int FRONTRIGHT = 1;
    static final int BACKLEFT = 2;
    static final int BACKRIGHT = 3;
    double[] turnFactors;
    double[] driveFactors;
    double[] strafeFactors;
    ElapsedTime driveTimer = new ElapsedTime();

    /* Constructor */
    public HardwareCwBot()
    {}

    // Mecanum wheels - Neverest Orbital 20
    // 1.0 inch is this many full power milliseconds.
    // Was 17.2 msec/inch before calibration with ultrasound and laser.
    // It was observed to be a factor of 1.0286 too large.
    static public final double msPerInch = 16.72;
    static public final double msPerCm = 6.583;
    static public final double msPerDegree = 3.474;

    // Return run time in msec for driving amounts.
    public static int inches(double len) {return (int) Math.round(len * msPerInch);}
    public static int cms(double len)
    {
        return (int) Math.round(len * msPerCm);
    }
    public static int degrees(double d)
    {
        return  (int) Math.round(d * msPerDegree);
    }


    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap)
    {
        Log.i("foo","robot.init");

        // Save reference to Hardware map
        hwMap = ahwMap;
        dim = hwMap.get(DeviceInterfaceModule.class, "DIM1");   //  Use generic form of device mapping
//        AnalogInput ds = hardwareMap.get(AnalogInput.class, "Ultrasound");
        dsFront = new AnalogInput(dim,7);
        revPotentiometer = new AnalogInput(dim,6);

        systemVoltage = dsFront.getMaxVoltage();
        setBlueLED(false);
        setRedLED(false);

        armEndSwitch = hwMap.get(DigitalChannel.class, "armEndSwitch");
        armEndSwitch.setMode(DigitalChannel.Mode.INPUT);

        // Define and Initialize Motors
        backRight = hwMap.dcMotor.get("backRight");
        backLeft = hwMap.dcMotor.get("backLeft");
        frontRight = hwMap.dcMotor.get("frontRight");
        frontLeft = hwMap.dcMotor.get("frontLeft");

        I2cDeviceSynch i2c = hwMap.get(I2cDeviceSynch.class, "TwoMeter");
        rev2M = new MyVL53L0X(i2c);

        allMotors = new DcMotor[] {frontLeft, frontRight, backLeft, backRight};
        turnFactors = new double[]{1.0, -1.0, 1.0, -1.0};
        driveFactors = new double[] {1.0, 1.0, 1.0, 1.0};
        // The back end of the robot is heavy. This makes the back motors more effective.
        strafeFactors = new double[] {1.0, -1.0, -0.85, 0.85};

        frontRight.setDirection(DcMotor.Direction.REVERSE);
        frontLeft.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.FORWARD);

        for (DcMotor m : allMotors)
        {
            m.setPower(0.0);
            m.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            m.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        // Define and initialize ALL installed servos.
        phoneServo = hwMap.servo.get("phoneServo");
        phoneServo.setPosition(PHONE_VERTICAL);

        imuParameters = new BNO055IMU.Parameters();
        imuParameters.accelRange = BNO055IMU.AccelRange.G16;
        imuParameters.gyroRange = BNO055IMU.GyroRange.DPS500;

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hwMap.get(BNO055IMU.class, "IMU");
        imu.initialize(imuParameters);
    }

    /* Initialize standard Hardware interfaces */
    public void simpleInit(HardwareMap ahwMap)
    {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Motors
        backRight = hwMap.dcMotor.get("backRight");
        backLeft = hwMap.dcMotor.get("backLeft");
        frontRight = hwMap.dcMotor.get("frontRight");
        frontLeft = hwMap.dcMotor.get("frontLeft");

        allMotors = new DcMotor[] {frontLeft, frontRight, backLeft, backRight};
        turnFactors = new double[]{1.0, -1.0, 1.0, -1.0};
        driveFactors = new double[] {1.0, 1.0, 1.0, 1.0};
        // The back end of the robot is heavy. This makes the back motors more effective.
        strafeFactors = new double[] {1.0, -1.0, -1.0, 1.0};

        frontRight.setDirection(DcMotor.Direction.REVERSE);
        frontLeft.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.FORWARD);

        for (DcMotor m : allMotors)
        {
            m.setPower(0.0);
            m.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            m.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
    }

    public double[] getDrivePowersFromAngle(double angle) {
        double[] unscaledPowers = new double[4];
        unscaledPowers[0] = Math.sin(angle + Math.PI / 4);
        unscaledPowers[1] = Math.cos(angle + Math.PI / 4);
        unscaledPowers[2] = unscaledPowers[1];
        unscaledPowers[3] = unscaledPowers[0];
        return unscaledPowers;
    }

    int AverageEncoders()
    {
        int sum = 0;
        for (DcMotor m : allMotors)
            sum += m.getCurrentPosition();
        return sum/4;
    }

    void TestRun2(double runTime, double power, LinearOpMode caller)
    {
        Log.i("foo",String.format("%.2f %.2f", runTime,power));
        Log.i("foo", "TestRun2: Time and power");

        long timeStep = 50;  // 50 msec cycles
        double deltaT = timeStep / 1000.0;
        double remainingIntegral = runTime * power;
        int powerTicks = (int)Math.floor(runTime/deltaT-0.5)-1;
        if (powerTicks < 0) powerTicks = 0;
        resetTickPeriod();
        double lastTime = driveTimer.time();
        double startTime = lastTime;

        double startupPower = 0.33 * power;
        for (int i=0; i<allMotors.length; i++)
            allMotors[i].setPower(startupPower);
        logEncoders(lastTime-startTime, startupPower);
        waitForTick(timeStep);
        double t0 = driveTimer.time();
        remainingIntegral -= (t0 - lastTime)*startupPower;
        lastTime = t0;

        startupPower = 0.67 * power;
        for (int i=0; i<allMotors.length; i++)
            allMotors[i].setPower(startupPower);
        logEncoders(lastTime-startTime, startupPower);
        waitForTick(timeStep);
        t0 = driveTimer.time();
        remainingIntegral -= (t0 - lastTime)*startupPower;
        lastTime = t0;

        if (powerTicks > 0)
        {
            for (int i=0; i<allMotors.length; i++)
                allMotors[i].setPower(power);
            for (int p=0; caller.opModeIsActive() && p<powerTicks; p++)
            {
                logEncoders(lastTime-startTime, power);
                waitForTick(timeStep);
                double newTime = driveTimer.time();
                remainingIntegral -= (newTime - lastTime)*power;
                lastTime = newTime;
                if (remainingIntegral <= 0.0) break;
            }
        }
        for (int p=0; caller.opModeIsActive() && p<5 && remainingIntegral > 0.0; p++)
        {
            double reducedPower = 0.8*remainingIntegral/deltaT;
            for (int i=0; i<allMotors.length; i++)
                allMotors[i].setPower(reducedPower);
            logEncoders(lastTime-startTime, reducedPower);
            waitForTick(timeStep);
            double newTime = driveTimer.time();
            remainingIntegral -= (newTime - lastTime)*reducedPower;
            lastTime = newTime;
        }
        if (remainingIntegral > 0.0)
        {
            double reducedPower = remainingIntegral/deltaT;
            for (int i=0; i<allMotors.length; i++)
                allMotors[i].setPower(reducedPower);
            logEncoders(driveTimer.time()-startTime, reducedPower);
            waitForTick(timeStep);
        }
        for (int i=0; i<allMotors.length; i++)
            allMotors[i].setPower(0.0);
        logEncoders(driveTimer.time()-startTime,0.0);
        for (int p=0; p<(int) (0.4/deltaT); p++)
        {
            waitForTick(timeStep);
            logEncoders(driveTimer.time()-startTime,0.0);
        }
    }

    double runPower = 0.5;

    void Drive(int fullPowerMs, LinearOpMode caller)
    {
        double time = fullPowerMs/1000.0;
        String msg = String.format("Drive: %d",fullPowerMs);
        Log.i("foo", msg);
        AllRun(Math.abs(time/runPower),runPower*Math.signum(time),driveFactors,caller);
    }

    // A turn is clockwise for the given full-power time.
    void Turn(int fullPowerMs, LinearOpMode caller)
    {
        double time = fullPowerMs/1000.0;
        AllRun(Math.abs(time/runPower),runPower*Math.signum(time),turnFactors,caller);
    }

    // A strafe is to the right.
    void Strafe(int fullPowerMs, LinearOpMode caller)
    {
        double time = fullPowerMs/1000.0;
        AllRun(Math.abs(time/runPower),runPower*Math.signum(time),strafeFactors,caller);
    }

    void TurnToHeading(int heading, LinearOpMode caller)
    {
        double diff = normalizeAngle(heading - getHeadingWithLog());
        String msg = String.format("TurnToHeading: %d diff: %.2f", heading, diff);
        Log.i("foo",msg);
        Turn(degrees(diff), caller);

        diff = normalizeAngle(heading - getHeadingWithLog());
        if (Math.abs(diff) > 2.0) {
            msg = String.format("TurnToHeading2: %d diff: %.2f", heading, diff);
            Log.i("foo", msg);
            Turn(degrees(diff), caller);
        }
    }

    void ApproachTo(int desiredDistanceInCm, LinearOpMode caller)
    {
//        double presentDistance = getFrontDistance();
        double presentDistance = rev2M.readRangeContinuousMillimeters()/10.0;
        double driveDistance = presentDistance-desiredDistanceInCm;
        String msg = String.format("ApproachTo: %d drive: %.2f", desiredDistanceInCm, driveDistance);
        Log.i("foo", msg);
        if (Math.abs(driveDistance)<40.0)
            Drive(cms(driveDistance), caller);
    }

    // AllRun - Handles all programed movement, including driving, strafing, and turning.

    void AllRun(double runTime, double power, double[] motorFactors, LinearOpMode caller)
    {
        long timeStepMsec = 50;  // 50 msec cycles
        double deltaT = timeStepMsec / 1000.0;
        double remainingIntegral = Math.abs(runTime * power);

        // If there is so little movement that there is no time for
        // ramps, then adjust the power down and time up.
        if (runTime/deltaT < 3.0)
        {
            runTime = 3.000001 * deltaT;
            power = remainingIntegral / runTime * Math.signum(power);
        }
        int powerTicks = (int)Math.floor(runTime/deltaT-3.0);
        resetTickPeriod();
        double lastTime = driveTimer.time();
        double startTime = lastTime;

        // Ramp up
        for (int p=1; caller.opModeIsActive() && p<4; p++) {
            double rampPower = (p * power)/4.0;
            for (int i = 0; i < allMotors.length; i++)
                allMotors[i].setPower(rampPower*motorFactors[i]);
            waitForTick(timeStepMsec);
            double t0 = driveTimer.time();
            remainingIntegral -= (t0 - lastTime) * Math.abs(rampPower);
            lastTime = t0;
        }

        if (powerTicks > 0)
        {
            double absPower = Math.abs(power);
            for (int i=0; i<allMotors.length; i++)
                allMotors[i].setPower(power*motorFactors[i]);
            for (int p=0; caller.opModeIsActive() && p<powerTicks; p++)
            {
                waitForTick(timeStepMsec);
                double newTime = driveTimer.time();
                remainingIntegral -= (newTime - lastTime)*absPower;
                lastTime = newTime;
                if (remainingIntegral <= 0.0) break;
            }
        }
        // Ramp down
        for (int p=3; caller.opModeIsActive() && p>0; p--) {
            if (remainingIntegral <= 0.0) break;
            double rampPower = (p * power)/4.0;
            double absPower = Math.abs(rampPower);
            for (int i = 0; i < allMotors.length; i++)
                allMotors[i].setPower(rampPower*motorFactors[i]);
            long step = (long)((2.0/(p+1))*remainingIntegral/absPower*1000.0);
            waitForTick(step);
            double t0 = driveTimer.time();
            remainingIntegral -= (t0 - lastTime) * absPower;
            lastTime = t0;
        }

        for (int i=0; i<allMotors.length; i++)
            allMotors[i].setPower(0.0);
        waitForTick(400);
    }


    void Rotate(double runTime, double power, LinearOpMode caller)
    {
        long timeStepMsec = 50;  // 50 msec cycles
        double deltaT = timeStepMsec / 1000.0;
        double remainingIntegral = runTime * Math.abs(power);

        // If there is so little movement that there is no time for
        // ramps, then adjust the power down and time up.
        if (runTime/deltaT < 3.0)
        {
            runTime = 3.000001 * deltaT;
            power = remainingIntegral / runTime;
        }
        int powerTicks = (int)Math.floor(runTime/deltaT-3.0);
        resetTickPeriod();
        double lastTime = driveTimer.time();
        double startTime = lastTime;

        // Ramp up
        for (int p=1; caller.opModeIsActive() && p<4; p++) {
            double rampPower = (p * power)/4.0;
            SetRotationPower(rampPower);
            waitForTick(timeStepMsec);
            double t0 = driveTimer.time();
            remainingIntegral -= (t0 - lastTime) * Math.abs(rampPower);
            lastTime = t0;
        }

        if (powerTicks > 0)
        {
            SetRotationPower(power);
            for (int p=0; caller.opModeIsActive() && p<powerTicks; p++)
            {
                waitForTick(timeStepMsec);
                double newTime = driveTimer.time();
                remainingIntegral -= (newTime - lastTime)*Math.abs(power);
                lastTime = newTime;
                if (remainingIntegral <= 0.0) break;
            }
        }
        // Ramp down
        for (int p=3; caller.opModeIsActive() && p>0; p--) {
            double rampPower = (p * power)/4.0;
            SetRotationPower(rampPower);
            long step = (long)((2.0/(p+1))*remainingIntegral/Math.abs(rampPower)*1000.0);
            waitForTick(step);
            double t0 = driveTimer.time();
            remainingIntegral -= (t0 - lastTime) * Math.abs(rampPower);
            lastTime = t0;
            if (remainingIntegral <= 0.0) break;
        }

        for (int i=0; i<allMotors.length; i++)
            allMotors[i].setPower(0.0);
        waitForTick(400);
    }

    void SetRotationPower(double power)
    {
        allMotors[FRONTRIGHT].setPower(power);
        allMotors[FRONTLEFT].setPower(-power);
        allMotors[BACKRIGHT].setPower(power);
        allMotors[BACKLEFT].setPower(-power);
    }

    void TestRun3(double runTime, double power, LinearOpMode caller)
    {
        Log.i("foo",String.format("%.2f %.2f", runTime,power));
        Log.i("foo", "TestRun3: Time and power");

        long timeStepMsec = 50;  // 50 msec cycles
        double deltaT = timeStepMsec / 1000.0;
        double remainingIntegral = runTime * power;

        // If there is so little movement that there is no time for
        // ramps, then adjust the power down and time up.
        if (runTime/deltaT < 3.0)
        {
            runTime = 3.000001 * deltaT;
            power = remainingIntegral / runTime;
        }
        int powerTicks = (int)Math.floor(runTime/deltaT-3.0);
        resetTickPeriod();
        double lastTime = driveTimer.time();
        double startTime = lastTime;

        // Ramp up
        for (int p=1; caller.opModeIsActive() && p<4; p++) {
            double rampPower = (p * power)/4.0;
            for (int i = 0; i < allMotors.length; i++)
                allMotors[i].setPower(rampPower);
            logEncoders(lastTime - startTime, rampPower);
            waitForTick(timeStepMsec);
            double t0 = driveTimer.time();
            remainingIntegral -= (t0 - lastTime) * rampPower;
            lastTime = t0;
        }

        if (powerTicks > 0)
        {
            for (int i=0; i<allMotors.length; i++)
                allMotors[i].setPower(power);
            for (int p=0; caller.opModeIsActive() && p<powerTicks; p++)
            {
                logEncoders(lastTime-startTime, power);
                waitForTick(timeStepMsec);
                double newTime = driveTimer.time();
                remainingIntegral -= (newTime - lastTime)*power;
                lastTime = newTime;
                if (remainingIntegral <= 0.0) break;
            }
        }
        // Ramp down
        for (int p=3; caller.opModeIsActive() && p>0; p--) {
            double rampPower = (p * power)/4.0;
            for (int i = 0; i < allMotors.length; i++)
                allMotors[i].setPower(rampPower);
            logEncoders(lastTime - startTime, rampPower);
            long step = (long)((2.0/(p+1))*remainingIntegral/rampPower*1000.0);
            waitForTick(step);
            double t0 = driveTimer.time();
            remainingIntegral -= (t0 - lastTime) * rampPower;
            lastTime = t0;
            if (remainingIntegral <= 0.0) break;
        }

        for (int i=0; i<allMotors.length; i++)
            allMotors[i].setPower(0.0);
        logEncoders(driveTimer.time()-startTime,0.0);
        for (int p=0; p<(int) (0.4/deltaT); p++)
        {
            waitForTick(timeStepMsec);
            logEncoders(driveTimer.time()-startTime,0.0);
        }
    }

    // Rotation PID settings.
    public double runWithHeadingKp = 0.10;
    public double runWithHeadingKi = 0.0;
    public double runWithHeadingKd = 0.0;


    // We started to see oscillation at P = 0.0011.  Not yet noticeable at 0.0008


    void TestRunWithHeading(double runTime, double power, LinearOpMode caller)
    {
        Log.i("foo",String.format("%.2f %.2f", runTime,power));
        Log.i("foo", String.format("TestRunWithHeading: Time and power"));
        Log.i("foo", String.format("P = %9.5f  I = %9.5f  D = %9.5f",runWithHeadingKp,runWithHeadingKi,runWithHeadingKd));

        long timeStepMsec = 50;  // 50 msec cycles
        double deltaT = timeStepMsec / 1000.0;
        double remainingIntegral = runTime * power;

        double targetHeading = getHeading();
        double lastEncoders = averageEncoder();
        double lastDiffHeading = 0.0;
        double deltaY = 0.0;  // Give the error correction a kick start.
        double lastDeltaY = 0.0;
        double integralDeltaY = 0.0;
        double dt = 0;

        // If there is so little movement that there is no time for
        // ramps, then adjust the power down and time up.
        if (runTime/deltaT < 3.0)
        {
            runTime = 3.000001 * deltaT;
            power = remainingIntegral / runTime;
        }
        int powerTicks = (int)Math.floor(runTime/deltaT-3.0);
        resetTickPeriod();
        double lastTime = driveTimer.time();
        double startTime = lastTime;

        double rDifferential = (deltaY * runWithHeadingKp + integralDeltaY * runWithHeadingKi)/100.0;

        // Ramp up
        for (int p=1; caller.opModeIsActive() && p<4; p++) {
            double rampPower = (p * power)/4.0;
            setDifferentialPowers(rampPower,rDifferential);
            logEncoders(lastTime - startTime, rampPower, rDifferential, deltaY);
            waitForTick(timeStepMsec);
            double t0 = driveTimer.time();
            remainingIntegral -= (t0 - lastTime) * rampPower;
            lastTime = t0;
        }

        double newDiffHeading = -diffHeading(targetHeading);
        double avgDiffHeading = 0.5*(newDiffHeading + lastDiffHeading);
        double newEncoders = averageEncoder();
        // Our SetPoint for deltaY is 0, and the control depends on SetPoint - PresentValue
        dt = lastTime - startTime;
        deltaY -= Math.sin(avgDiffHeading * Math.PI / 180.0) *(newEncoders - lastEncoders);
        integralDeltaY = 0.5 * deltaY * dt;
        lastDeltaY = deltaY;
        lastEncoders = newEncoders;
        lastDiffHeading = newDiffHeading;
        rDifferential = (deltaY * runWithHeadingKp + integralDeltaY * runWithHeadingKi)/100.0;

        if (powerTicks > 0)
        {
            for (int p=0; caller.opModeIsActive() && p<powerTicks; p++)
            {
                logEncoders(lastTime-startTime, power, rDifferential, deltaY);
                setDifferentialPowers(power,rDifferential);
                waitForTick(timeStepMsec);

                newDiffHeading = -diffHeading(targetHeading);
                avgDiffHeading = 0.5*(newDiffHeading + lastDiffHeading);
                newEncoders = averageEncoder();
                double newTime = driveTimer.time();
                dt = newTime - lastTime;
                deltaY -= Math.sin(avgDiffHeading * Math.PI / 180.0) *(newEncoders - lastEncoders);
                integralDeltaY = 0.5 * (deltaY + lastDeltaY) * dt;
                lastDeltaY = deltaY;
                lastEncoders = newEncoders;
                lastDiffHeading = newDiffHeading;
                rDifferential = (rDifferential = deltaY * runWithHeadingKp + integralDeltaY * runWithHeadingKi)/100.0;

                remainingIntegral -= dt*power;
                lastTime = newTime;
                if (remainingIntegral <= 0.0) break;
            }
        }
        // Ramp down
        for (int p=3; caller.opModeIsActive() && p>0; p--) {
            double rampPower = (p * power)/4.0;
            setDifferentialPowers(rampPower,rDifferential);
            logEncoders(lastTime - startTime, rampPower, rDifferential, deltaY);
            long step = (long)((2.0/(p+1))*remainingIntegral/rampPower*1000.0);
            waitForTick(step);

            newDiffHeading = -diffHeading(targetHeading);
            avgDiffHeading = 0.5*(newDiffHeading + lastDiffHeading);
            newEncoders = averageEncoder();
            double t0 = driveTimer.time();
            dt = t0 - lastTime;
            deltaY -= Math.sin(avgDiffHeading * Math.PI / 180.0) *(newEncoders - lastEncoders);
            integralDeltaY = 0.5 * (deltaY + lastDeltaY) * dt;
            lastDeltaY = deltaY;
            lastEncoders = newEncoders;
            lastDiffHeading = newDiffHeading;
            rDifferential = rDifferential = (deltaY * runWithHeadingKp + integralDeltaY * runWithHeadingKi)/100.0;

            remainingIntegral -= dt * rampPower;
            lastTime = t0;
            if (remainingIntegral <= 0.0) break;
        }

        for (int i=0; i<allMotors.length; i++)
            allMotors[i].setPower(0.0);
        logEncoders(driveTimer.time()-startTime,0.0);
        for (int p=0; p<(int) (0.4/deltaT); p++)
        {
            waitForTick(timeStepMsec);
            logEncoders(driveTimer.time()-startTime,0.0);
        }
    }

    void setDifferentialPowers(double power, double r)
    {
        double pPlus = (1.0 + r) * power;
        double pMinus = (1.0 - r) * power;
        allMotors[FRONTRIGHT].setPower(pPlus);
        allMotors[BACKRIGHT].setPower(pPlus);
        allMotors[FRONTLEFT].setPower(pMinus);
        allMotors[BACKLEFT].setPower(pMinus);
    }

    double averageEncoder()
    {
        double sum = 0.0;
        for (DcMotor m : allMotors)
            sum += m.getCurrentPosition();
        return sum / 4.0;
    }


    void logEncoders(double t)
    {
        String msg = String.format("drive: %7.4f %6d %6d %6d %6d",
                t,
                allMotors[0].getCurrentPosition(),
                allMotors[1].getCurrentPosition(),
                allMotors[2].getCurrentPosition(),
                allMotors[3].getCurrentPosition()
        );
        Log.i("foo",msg);
    }

    void logEncoders(double t, double power)
    {
        String msg = String.format("drive: %7.4f %6d %6d %6d %6d %7.3f %5.2f",
                t,
                allMotors[0].getCurrentPosition(),
                allMotors[1].getCurrentPosition(),
                allMotors[2].getCurrentPosition(),
                allMotors[3].getCurrentPosition(),
                power,
                getHeading()
        );
        Log.i("foo",msg);
    }

    void logEncoders(double t, double power, double r, double dy)
    {
        String msg = String.format("drive: %7.4f %6d %6d %6d %6d %7.3f %5.2f %8.4f %8.2f",
                t,
                allMotors[0].getCurrentPosition(),
                allMotors[1].getCurrentPosition(),
                allMotors[2].getCurrentPosition(),
                allMotors[3].getCurrentPosition(),
                power,
                getHeading(),
                r,
                dy
        );
        Log.i("foo",msg);
    }

    static final public int DRIVE = 0;
    static final public int STRAFE = 1;
    static final public int TURN = 2;
    static final public int TURNTOHEADING = 3;
    static final public int CHECKIMU = 4;
    static final public int SETHEADING = 5;
    static final public int SETPOWER = 6;
    static final public int WAIT = 7;
    static final public int APPROACHTO = 8;
    static final public int DISPLAY = 9;

    void RunProgram(int[] prog, LinearOpMode caller)
    {
        int  nSteps = prog.length/2;
        for (int iStep = 0; iStep<nSteps; iStep++)
        {
            int instruction = prog[2*iStep];
            int data = prog[2*iStep+1];
            switch (instruction)
            {
                case DRIVE:
                    Drive(data, caller);
                    break;
                case STRAFE:
                    Strafe((data*43)/29, caller);
                    break;
                case TURN:
                    Turn(data, caller);
                    break;
                case TURNTOHEADING:
                    //waitForTick(100);
                    TurnToHeading(data, caller);
                    break;
                case SETHEADING:
                    //waitForTick(100);
                    setHeading(data);
                    break;
                case SETPOWER:
                    runPower = data / 100.0;
                    break;
                case WAIT:
                    waitForTick((long)data);
                    break;
                case DISPLAY:
                    waitForTick(1000);
                    double dLeft = getFrontDistance();
                    double dRight = 0.0;// rev2M.getDistance(DistanceUnit.CM);
                    caller.telemetry.addData("heading", "%.1f",getHeading());
                    caller.telemetry.addData("ds",  "%.2f %.2f", dLeft, dRight);
                    caller.telemetry.update();
                    waitForTick(1000);
                    break;
                case APPROACHTO:
                    //waitForTick(1000);
                    ApproachTo(data, caller);
                    break;
                case CHECKIMU:
                    Quaternion q = imu.getQuaternionOrientation();
                    if (q.magnitude() < 0.9)
                    {
                        caller.telemetry.addData("Say","IMU died!!!");
                        caller.telemetry.update();
                        imu.initialize(imuParameters);
                        setRedLED(true);
                        waitForTick(2000);
                    }
            }
            caller.telemetry.addData("heading", "%.1f",getHeading());
            caller.telemetry.update();
        }
    }

    public void setHeading(double h)
    {
        Log.i("foo",String.format("setHeading: %.2f",h));
        checkAndResetIMU();
        headingOffset = 0.0;
        headingOffset = h - getHeadingWithLog();
    }

    public boolean checkAndResetIMU()
    {
        Quaternion q = imu.getQuaternionOrientation();
        double c = q.w;
        double s = q.z;
        double norm = Math.sqrt(c*c+s*s);
        if (norm < 0.5)
        {
            imu.initialize(imuParameters);
            Log.i("foo","IMU Reset!");
            waitForTick(500);
            return false;
        }
        return true;  // It was fine.
    }

    private double headingOffset = 0.0;

    // The heading is defined to be a clockwise angle measure from 'north'.
    // This is the negative of the intrinsic quaternion system.
    double getHeading()
    {
        Quaternion q = imu.getQuaternionOrientation();
        double c = q.w;
        double s = q.z;
        double norm = Math.sqrt(c*c+s*s);
        if (norm < 0.5)
        {
            //telemetry.addData("Say", "IMU died!");
            //telemetry.update();
            return 0.0;
        }
        // The atan2 returns the half-angle, so double it and convert to degrees.
        double angle = -Math.atan2(s,c) * 360.0 / Math.PI; // The atan gives us the half-angle.
        // Put the angle in the range from -180 to 180.
        return -normalizeAngle(angle + headingOffset);
    }

    double getHeadingWithLog()
    {
        Quaternion q = imu.getQuaternionOrientation();
        String msg = String.format("q: %.3f %.3f %.3f %.3f",q.w,q.x,q.y,q.z);
        Log.i("foo",msg);
        double c = q.w;
        double s = q.z;
        double norm = Math.sqrt(c*c+s*s);
        if (norm < 0.5)
        {
            Log.i("foo","IMU died");
            //telemetry.addData("Say", "IMU died!");
            //telemetry.update();
            return 0.0;
        }
        // The atan2 returns the half-angle, so double it and convert to degrees.
        double angle = -Math.atan2(s,c) * 360.0 / Math.PI; // The atan gives us the half-angle.
        // Put the angle in the range from -180 to 180.
        return -normalizeAngle(angle + headingOffset);
    }

    // Return an equivalent angle from -180 to 180.
    double normalizeAngle(double d)
    {
        return d-Math.floor((d + 180.0)/360.0)*360.0;
    }

    double diffHeading(double target)
    {
        double diff = target - getHeading();
        return normalizeAngle(diff);
//        diff = (diff + 720.0) % 360.0;
//        if (diff > 180.0) diff -= 360;
//        return diff;
    }

    public double getFrontDistance() // in cm
    {
        return Math.round(dsFront.getVoltage()/systemVoltage*1024.0);
    }

    public double getPotentiometer()
    {
        return Math.round(revPotentiometer.getVoltage()/systemVoltage*1024.0);
    }

    public void setBlueLED(boolean on)
    {
        dim.setLED(0,on);
    }

    public void setRedLED(boolean on)
    {
        dim.setLED(1,on);
    }

    private long lastPeriodTime = 0;
    public void resetTickPeriod()
    {
        period.reset();
        lastPeriodTime = 0;
    }

    /***
     *
     * waitForTick implements a periodic delay. However, this acts like a metronome with a regular
     * periodic tick.  This is used to compensate for varying processing times for each cycle.
     * The function looks at the elapsed cycle time, and sleeps for the remaining time interval.
     *
     * @param periodMs  Length of wait cycle in mSec.
     */
    public void waitForTick(long periodMs)
    {
        lastPeriodTime += periodMs; // Our estimate of when we return from this call.
        long  remaining = lastPeriodTime - (long)period.milliseconds();

        // sleep for the remaining portion of the regular cycle period.
        if (remaining > 0) {
            try {
                Thread.sleep(remaining);
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
            }
        }
    }
}
