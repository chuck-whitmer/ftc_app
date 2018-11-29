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

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.Quaternion;

import java.io.FileWriter;
import java.io.IOException;
import java.io.PrintWriter;


@TeleOp(name="cwbot: Telop IMU", group="cwbot")
public class cwbotIMUTest extends LinearOpMode
{
    HardwareCwBot robot = new HardwareCwBot();
    Quaternion[] quats;
    Acceleration[] accels;
    final int nn = 1000;



    @Override
    public void runOpMode() {
        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        quats = new Quaternion[nn];
        accels = new Acceleration[nn];
        int iWrite = 0;
        int iTotal = 0;

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Hello IMU Driver");    //
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            if (gamepad1.right_bumper)
            {
                robot.Drive(robot.inches(-2.0), this);
            }
            if (gamepad1.left_bumper)
            {
                robot.Drive(robot.inches(2.0), this);
            }

            float x = gamepad1.right_stick_x;
            float y = -gamepad1.right_stick_y; // Negate to get +y forward.
            float rotation = gamepad1.left_stick_x;

            // A B

            float b = y - rotation;
            float a = y + rotation;

            float biggest = Math.max(Math.abs(a),Math.abs(b));
            if (biggest < 1.0f) biggest = 1.0f;

            robot.backRight.setPower(b/biggest);
            robot.backLeft.setPower(a/biggest);

            int encoderA = robot.backLeft.getCurrentPosition();
            int encoderB = robot.backRight.getCurrentPosition();

            Quaternion q = robot.imu.getQuaternionOrientation();
            Acceleration acc = robot.imu.getOverallAcceleration();





            quats[iWrite] = q;
            accels[iWrite] = acc;
            iTotal++;
            iWrite++;
            if (iWrite >= nn) iWrite = 0;

            robot.waitForTick(10);
        }


        try
        {
            FileWriter fw = new FileWriter("/sdcard/FIRST/mylog.txt");
            PrintWriter pw = new PrintWriter(fw);

            int n = Math.min(iTotal,nn);
            for (int i=0; i<n; i++)
            {
                Quaternion q = quats[i];
                Acceleration acc = accels[i];

                pw.printf("Q: %d %.5f %.5f %.5f %.5f A: %d %.6f %.6f %.6f\r\n",q.acquisitionTime,q.w,q.x,q.y,q.z,acc.acquisitionTime,acc.xAccel,acc.yAccel,acc.zAccel );
            }
            pw.flush();
            pw.close();
            fw.close();
        }
        catch (IOException e)
        {
            telemetry.addData("Say","File write failure");
            telemetry.update();
            robot.waitForTick(3000);
        }


    }
}
