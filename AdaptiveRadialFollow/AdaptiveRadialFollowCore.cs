/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. */

using System;
using System.Numerics;
using OpenTabletDriver.Plugin.Timing;
using OpenTabletDriver.Plugin.Attributes;
using OpenTabletDriver.Plugin.Output;
using OpenTabletDriver.Plugin.Tablet;

namespace AdaptiveRadialFollow
{
    public class AdaptiveRadialFollowCore
    {

        public double OuterRadius
        {
            get { return rOuter; }
            set { rOuter = System.Math.Clamp(value, 0.0f, 1000000.0f); }
        }
        private double rOuter = 0;

        public double InnerRadius
        {
            get { return rInner; }
            set { rInner = System.Math.Clamp(value, 0.0f, 1000000.0f); }
        }
        private double rInner = 0;

        public double SmoothingCoefficient
        {
            get { return smoothCoef; }
            set { smoothCoef = System.Math.Clamp(value, 0.0001f, 1.0f); }
        }
        private double smoothCoef;

        public double SoftKneeScale
        {
            get { return knScale; }
            set { knScale = System.Math.Clamp(value, 0.0f, 100.0f); }
        }
        private double knScale;

        public double SmoothingLeakCoefficient
        {
            get { return leakCoef; }
            set { leakCoef = System.Math.Clamp(value, 0.01f, 1.0f); }
        }
        private double leakCoef;

        public double VelocityDivisor
        {
            get { return vDiv; }
            set { vDiv = System.Math.Clamp(value, 0.01f, 1000000.0f); }
        }
        private double vDiv;

        public double MinimumRadiusMultiplier
        {
            get { return minMult; }
            set { minMult = System.Math.Clamp(value, 0.0f, 1.0f); }
        }
        private double minMult;

        public double RadialMultPower
        {
            get { return radPower; }
            set { radPower = System.Math.Clamp(value, 1.0f, 1000000.0f); }
        }
        private double radPower;

        public double MinimumSmoothingDivisor
        {
            get { return minSmooth; }
            set { minSmooth = System.Math.Clamp(value, 2.0f, 1000000.0f); }
        }
        public double minSmooth;

        public bool VelocityScalesKnee
        {
            get { return vKnee; }
            set { vKnee = value; }
        }
        private bool vKnee;

        public double RawAccelThreshold
        {
            get { return rawThreshold; }
            set { rawThreshold = System.Math.Clamp(value, -1000000.0f, 0.0f); }
        }
        public double rawThreshold;

        public bool ConsoleLogging
        {
            get { return cLog; }
            set { cLog = value; }
        }
        private bool cLog;

        public double AccelMultPower
        {
            get { return accPower; }
            set { accPower = System.Math.Clamp(value, 1.0f, 1000000.0f); }
        }
        public double accPower;

        public float SampleRadialCurve(IDeviceReport value, float dist) => (float)deltaFn(value, dist, xOffset(value), scaleComp(value));
        public double ResetMs = 1;
        public double GridScale = 1;

        Vector2 cursor;
        HPETDeltaStopwatch stopwatch = new HPETDeltaStopwatch(true);

        public Vector2 Filter(IDeviceReport value, Vector2 target)
        {
            UpdateReports(value);
           
            Vector2 direction = target - cursor;
            float distToMove = SampleRadialCurve(value, direction.Length());
            direction = Vector2.Normalize(direction);
            cursor = cursor + Vector2.Multiply(direction, distToMove);

            // Catch NaNs and pen redetection
            if (!(float.IsFinite(cursor.X) & float.IsFinite(cursor.Y) & stopwatch.Restart().TotalMilliseconds < 50))
                cursor = target;

            if (accel / (6 / vDiv) < rawThreshold)
            {
                lerpScale = Smootherstep(accel / (6 / vDiv), rawThreshold, rawThreshold - (1 / (6 / vDiv)));
                cursor = LerpedCursor((float)lerpScale, cursor, target);
            }

            if (cLog == true)
            {
                Console.WriteLine("Start of report ----------------------------------------------------");
                Console.WriteLine("Raw Velocity:");
                Console.WriteLine(vel);
                Console.WriteLine("Raw Acceleration:");
                Console.WriteLine(accel);
                Console.WriteLine("Accel Mult (this is an additional factor that multiplies velocity, should be close to or 0 on sharp decel, hovering around 1 when neutral, and close to or 2 on sharp accel. Only affected by power on radius scaling, so not shown.):");
                Console.WriteLine(accelMult);
                Console.WriteLine("Outside Radius:");
                Console.WriteLine(rOuterAdjusted(value, cursor, rOuter, rInner));
                Console.WriteLine("Inner Radius:");
                Console.WriteLine(rInnerAdjusted(value, cursor, rInner));
                Console.WriteLine("Smoothing Coefficient:");
                Console.WriteLine(smoothCoef / (1 + (Smoothstep(vel * accelMult, vDiv, 0) * (minSmooth - 1))));
                Console.WriteLine("Sharp Decel Lerp (With sharp decel, cursor is lerped between calculated value and raw report using this scale):");
                Console.WriteLine(lerpScale);

                if (vKnee == true)
                {
                    Console.WriteLine("Adjusted Soft Knee Scale:");
                    Console.WriteLine(((6 * (vel / vDiv) + 1) * accelMult) * knScale);
                }


                Console.WriteLine(RawAngle(lastLastReport, lastReport, currReport));
                Console.WriteLine("End of report ----------------------------------------------------");
            }

            lerpScale = 0;

            return cursor;
        }

        double xOffset(IDeviceReport value) => getXOffest(value);
        
        double scaleComp(IDeviceReport value) => getScaleComp(value);

        public Vector2 lastLastReport;

        public Vector2 lastReport;

        public Vector2 currReport;

        public Vector2 diff;

        public Vector2 seconddiff;

        public double accel;

        public double vel;

        public double accelMult;

        public double lerpScale;

        void UpdateReports(IDeviceReport value)
        {
            if (value is ITabletReport report)
            {
                lastLastReport = lastReport;

                lastReport = currReport;

                currReport = report.Position;

                diff = currReport - lastReport;

                seconddiff = lastReport - lastLastReport;

                vel =  Math.Sqrt(Math.Pow(diff.X, 2) + Math.Pow(diff.Y, 2)) / 100;

                accel = vel - Math.Sqrt(Math.Pow(seconddiff.X, 2) + Math.Pow(seconddiff.Y, 2)) / 100;

                accelMult = Smoothstep(accel, -1 / (6 / vDiv), 0) + Smoothstep(accel, 0, 1 / (6 / vDiv));
            }
        }
        
        /// Math functions
        
        public static double Dot(Vector2 a, Vector2 b)
        {
            return a.X * b.X + a.Y * b.Y;
        }
        public static double RawAngle(Vector2 vll, Vector2 vl, Vector2 vc)
        {
            Vector2 v1 = vll - vc;
            Vector2 v2 = vl - vc;

            double dotProduct = Dot(v1, v2);

            double mag1 = Math.Sqrt(v1.X * v1.X + v1.Y * v1.Y);
            double mag2 = Math.Sqrt(v2.X * v2.X + v2.Y * v2.Y);
        
            double cosine = Math.Clamp(dotProduct / (mag1 * mag2), -1, 1);

            return Math.Acos(cosine) * (180 / Math.PI);
        }

        double missedSnapness(double angle, double vel, double accel)
        {
            double snapAngle = Smoothstep(angle, 45, 90) - Smoothstep(angle, 90, 135);
            return 1000000000000;







        }

        double kneeFunc(double x) => x switch
        {
            < -3 => x,
            < 3 => Math.Log(Math.Tanh(Math.Exp(x)), Math.E),
            _ => 0,
        };

        public static double Smoothstep(double x, double start, double end) // yes i copied this from pp
        {
            x = Math.Clamp((x - start) / (end - start), 0.0, 1.0);

            return x * x * (3.0 - 2.0 * x);
        }

        public static double Smootherstep(double x, double start, double end) // this too
        {
            x = Math.Clamp((x - start) / (end - start), 0.0, 1.0);

            return x * x * x * (x * (6.0 * x - 15.0) + 10.0);
        }

        public static Vector2 LerpedCursor(float x, Vector2 cursor, Vector2 target)
        {
            x = Math.Clamp(x, 0.0f, 1.0f);
    
         return new Vector2
         (
             cursor.X + (target.X - cursor.X) * x,
             cursor.Y + (target.Y - cursor.Y) * x
         );
        }

        double kneeScaled(IDeviceReport value, double x) 
        
        {
            double velocity = 1;
            if (vKnee == true)
            {
            if (value is ITabletReport report)
            {
                velocity = (6 * (vel / vDiv) * accelMult) + 1;
            }
            }

            return knScale switch
            {
            > 0.0001f => (knScale * velocity) * kneeFunc(x / (knScale * velocity)) + 1,
            _ => x > 0 ? 1 : 1 + x,
            };
        }
        double inverseTanh(double x) => Math.Log((1 + x) / (1 - x), Math.E) / 2;

        double inverseKneeScaled(IDeviceReport value, double x) 
        {
            double velocity = 1;
            if (vKnee == true)
            {
            if (value is ITabletReport report)
            {
                velocity = (6 * (vel / vDiv) * accelMult) + 1;
            }
            }
         return (velocity * knScale) * Math.Log(inverseTanh(Math.Exp((x - 1) / (knScale * velocity))), Math.E);
        }

        double derivKneeScaled(IDeviceReport value, double x)
        {
            double velocity = 1;
            if (vKnee == true)
            {
            if (value is ITabletReport report)
            {
                velocity = (6 * (vel / vDiv) * accelMult) + 1;
            }
            }
            var e = Math.Exp(x / (knScale * velocity));
            var tanh = Math.Tanh(e);
            return (e - e * (tanh * tanh)) / tanh;
        }

        double getXOffest(IDeviceReport value) => inverseKneeScaled(value, 0);

        double getScaleComp(IDeviceReport value) => derivKneeScaled(value, getXOffest(value));

        public double rOuterAdjusted(IDeviceReport value, Vector2 cursor, double rOuter, double rInner)
        {
            if (value is ITabletReport report)
            {
                double velocity = vel * Math.Pow(accelMult, accPower);
                return Math.Max(Math.Min(Math.Pow(velocity / vDiv, radPower), 1), minMult) * Math.Max(rOuter, rInner + 0.0001f);
            }
            else
            return 0;
        }

        public double rInnerAdjusted(IDeviceReport value, Vector2 cursor, double rInner)
        {
             if (value is ITabletReport report)
            {
                double velocity = vel * Math.Pow(accelMult, accPower);
                return Math.Max(Math.Min(Math.Pow(velocity / vDiv, radPower), 1), minMult) * rInner;
            }
            else
            {
            return 0;
            }
        }

        double leakedFn(IDeviceReport value, double x, double offset, double scaleComp)
        => kneeScaled(value, x + offset) * (1 - leakCoef) + x * leakCoef * scaleComp;

        double smoothedFn(IDeviceReport value, double x, double offset, double scaleComp)
        {
            double velocity = 1;
            double LowVelocityUnsmooth = 1;
            if (value is ITabletReport report)
            {
                velocity = vel;
                LowVelocityUnsmooth = 1 + (Smoothstep(vel * accelMult, vDiv, 0) * (minSmooth - 1));
            }
        return leakedFn(value, x * (smoothCoef / LowVelocityUnsmooth) / scaleComp, offset, scaleComp);
        }

        double scaleToOuter(IDeviceReport value, double x, double offset, double scaleComp)
        {
            if (value is ITabletReport report)
            {
                return (rOuterAdjusted(value, cursor, rOuter, rInner) - rInnerAdjusted(value, cursor, rInner)) * smoothedFn(value, x / (rOuterAdjusted(value, cursor, rOuter, rInner) - rInnerAdjusted(value, cursor, rInner)), offset, scaleComp);
            }
            else
            {
            return (rOuter - rInner) * smoothedFn(value, x / (rOuter - rInner), offset, scaleComp);
            } 
        }

        double deltaFn(IDeviceReport value, double x, double offset, double scaleComp)
        {
            if (value is ITabletReport report)
            {
                return x > rInnerAdjusted(value, cursor, rInner) ? x - scaleToOuter(value, x - rInnerAdjusted(value, cursor, rInner), offset / 1, scaleComp / 1) - rInnerAdjusted(value, cursor, rInner) : 0;
            }
            else
            {
                return x > rInner ? x - scaleToOuter(value, x - rInner, offset, scaleComp) - rInnerAdjusted(value, cursor, rInner) : 0;
            }


        }
    }
}
