/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * This OpMode uses the common HardwareK9bot class to define the devices on the robot.
 * All device access is managed through the HardwareK9bot class. (See this class for device names)
 * The code is structured as a LinearOpMode
 *
 * This particular OpMode executes a basic Tank Drive Teleop for the K9 bot
 * It raises and lowers the arm using the Gampad Y and A buttons respectively.
 * It also opens and closes the claw slowly using the X and B buttons.
 *
 * Note: the configuration of the servos is such that
 * as the arm servo approaches 0, the arm position moves up (away from the floor).
 * Also, as the claw servo approaches 0, the claw opens up (drops the game element).
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="awbot: Teleop Drive", group="awbot")
public class awbotTeleopTank extends LinearOpMode
{
    /* Declare OpMode members. */
    HardwareCwBot robot = new HardwareCwBot();

    @Override
    public void runOpMode() {
        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        telemetry.addData("Say", "Init robot...");    //
        telemetry.update();
        robot.simpleInit(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Hello Driver");    //
        telemetry.update();

        // Wait for the game to start (drive r presses PLAY)
        waitForStart();
        robot.resetTickPeriod();

        boolean aLastState = false;
        boolean aPressed = false;
        boolean bLastState = false;
        boolean bPressed = false;
        boolean yLastState = false;
        boolean yPressed = false;
        boolean xLastState = false;
        boolean xPressed = false;

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            //robot.backRight.setPower(gamepad1.right_bumper ? 1.0 : 0.0);
            //robot.backLeft.setPower(gamepad1.left_bumper ? 1.0 : 0.0);


            float x = gamepad1.left_stick_x;
            float y = -gamepad1.left_stick_y; // Negate to get +y forward.
            float rotation = gamepad1.right_stick_x;
            float speedControl = 0.5f*(1.0f + gamepad1.left_trigger);
            double biggestControl = Math.sqrt(x*x+y*y);
            double biggestWithRotation = Math.sqrt(x*x+y*y+rotation*rotation);

            double angle = Math.atan2(y,-x) - Math.PI/2.0;

            double[] powers = robot.getDrivePowersFromAngle(angle);
            double pow2 = 0.0;
            for (int i=0; i<robot.allMotors.length; i++)
            {
                double pow = powers[i]*biggestControl + rotation * robot.turnFactors[i];
                powers[i] = pow;
                pow2 += pow*pow;
            }

            if (biggestWithRotation != 0.0) {
                double scale = Math.sqrt(pow2);
                for (int i = 0; i < robot.allMotors.length; i++) {
                    robot.allMotors[i].setPower(
                            powers[i]/scale*biggestWithRotation*speedControl);
                }
            }
            else
            {
                for (int i = 0; i < robot.allMotors.length; i++)
                    robot.allMotors[i].setPower(0.0);
            }

            aPressed = gamepad1.a && !aLastState;
            aLastState = gamepad1.a;
            bPressed = gamepad1.b && !bLastState;
            bLastState = gamepad1.b;
            yPressed = gamepad1.y && !yLastState;
            yLastState = gamepad1.y;
            xPressed = gamepad1.x && !xLastState;
            xLastState = gamepad1.x;

            // YBA = PID
            if (yPressed)
            {
            }

            if (bPressed)
            {
            }

            if (xPressed)
            {
            }

            int encoderA = robot.frontLeft.getCurrentPosition();
            int encoderB = robot.backLeft.getCurrentPosition();
            int encoderC = robot.frontRight.getCurrentPosition();
            int encoderD = robot.backRight.getCurrentPosition();
            telemetry.addData("Encoders","%6d %6d %6d %6d", encoderA,encoderB,encoderC,encoderD);
            telemetry.update();

            robot.waitForTick(40);
        }
    }

    void ExerciseDistanceSensors()
    {
        robot.runPower = 0.25;
        for (int i=0; i<4; i++)
        {
            robot.allMotors[i].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.allMotors[i].setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        robot.setHeading(0);
        int nSteps = 3;
        int iLoops = 4;
        double length = 12.0;
        for (int ll=0; opModeIsActive() && ll<iLoops; ll++)
        {
            LogMeasurements();
            for (int i = 0; i < nSteps; i++) {
                robot.Drive(-HardwareCwBot.inches(length),this);
                LogMeasurements();
            }
            robot.TurnToHeading(0, this);
            LogMeasurements();
            if (!opModeIsActive()) break;
            for (int i = 0; i < nSteps; i++) {
                robot.Drive(HardwareCwBot.inches(length),this);
                LogMeasurements();
            }
            robot.TurnToHeading(0, this);
        }
    }

    void ApproachAndLogSensors()
    {
        robot.runPower = 0.25;
        for (int i=0; i<4; i++)
        {
            robot.allMotors[i].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.allMotors[i].setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        robot.setHeading(0);
        LogMeasurements();

        double lastLaserDistance = robot.rev2M.getDistance(DistanceUnit.CM);
        while (opModeIsActive() && lastLaserDistance > 10.0)
        {
            robot.Drive(HardwareCwBot.inches(2.0), this);
            robot.TurnToHeading(0, this);
            LogMeasurements();
            lastLaserDistance = robot.rev2M.getDistance(DistanceUnit.CM);
        }
    }

    void RetreatAndLogSensors()
    {
        double stepDistance = 8.0; // inches
        robot.runPower = 0.25;
        for (int i=0; i<4; i++)
        {
            robot.allMotors[i].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.allMotors[i].setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        robot.setHeading(0);
        LogMeasurements();

        double lastLaserDistance = robot.rev2M.getDistance(DistanceUnit.CM);
        while (opModeIsActive() && lastLaserDistance < 800.0)
        {
            robot.Drive(HardwareCwBot.inches(-stepDistance), this);
            robot.TurnToHeading(0, this);
            LogMeasurements();
            lastLaserDistance = robot.rev2M.getDistance(DistanceUnit.CM);
        }
    }

    void LogMeasurements()
    {
        robot.waitForTick(1000);
        int[] encoders = new int[4];
        for (int i=0; i<4; i++)
            encoders[i] = robot.allMotors[i].getCurrentPosition();
        double ultraSoundSensor = robot.getFrontDistance();
        double laserSensor =  robot.rev2M.getDistance(DistanceUnit.CM);
        robot.waitForTick(200);
        double heading = robot.getHeading();
        double ultraSoundSensor2 = robot.getFrontDistance();
        double laserSensor2 =  robot.rev2M.getDistance(DistanceUnit.CM);
        Log.i("foo",String.format("distance %6.1f %5d %5d %5d %5d %6.1f %6.1f %6.1f %6.1f",
                heading,
                encoders[0], encoders[1], encoders[2], encoders[3],
                ultraSoundSensor, laserSensor, ultraSoundSensor2, laserSensor2));
    }

    void TestAutoR()
    {
        robot.RunProgram(AutoPath.programOrbit,this);
    }
    void TestAutoC()
    {
        robot.RunProgram(AutoPath.programToAndFro,this);
    }
    void TestAutoL()
    {
        robot.RunProgram(AutoPath.programLeftGold,this);
    }

    int[] programTestDrive = new int[]
            {
                    HardwareCwBot.DRIVE, HardwareCwBot.inches(12.0),
                    HardwareCwBot.DRIVE, HardwareCwBot.inches(12.0),
                    HardwareCwBot.DRIVE, HardwareCwBot.inches(12.0)
            };
    int[] programTestStrafe = new int[]
            {
                    HardwareCwBot.STRAFE, HardwareCwBot.inches(12.0),
                    HardwareCwBot.STRAFE, HardwareCwBot.inches(12.0),
                    HardwareCwBot.STRAFE, HardwareCwBot.inches(12.0)
            };

    int[] programTestTurns = new int[]
            {
                    HardwareCwBot.TURN, HardwareCwBot.degrees(90.0),
                    HardwareCwBot.TURN, HardwareCwBot.degrees(90.0),
                    HardwareCwBot.TURN, HardwareCwBot.degrees(90.0),
                    HardwareCwBot.TURN, HardwareCwBot.degrees(90.0)
            };

}
