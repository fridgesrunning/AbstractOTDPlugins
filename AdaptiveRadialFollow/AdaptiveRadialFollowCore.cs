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

        public double RawAccelThreshold
        {
            get { return rawThreshold; }
            set { rawThreshold = System.Math.Clamp(value, -1000000.0f, 1000.0f); }
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

        public bool Advanced
        {
            get { return aToggle; }
            set { aToggle = value; }
        }
        public bool aToggle;

        public double rvt
        {
            get { return rawv; }
            set { rawv = value; }
        }
        public double rawv;

        public double aidx
        {
            get { return angidx; }
            set { angidx = value; }
        }
        public double angidx;

        public double xlerpconf
        {
            get { return explerpconf; }
            set { explerpconf = value; }
        }
        public double explerpconf;

        public double accelMultVelocityOverride
        {
            get { return amvDiv; }
            set { amvDiv = vDiv;
            if (aToggle == true)
            amvDiv = value;  }
        }
        public double amvDiv;

        public double spinCheckConfidence
        {
            get { return scConf; }
            set { scConf = value; }
        }
        public double scConf;

        public float SampleRadialCurve(IDeviceReport value, float dist) => (float)deltaFn(value, dist, xOffset(value), scaleComp(value));
        public double ResetMs = 1;
        public double GridScale = 1;

        public Vector2 Filter(IDeviceReport value, Vector2 target)
        {
                // Produce numbers (velocity, accel, etc)
            UpdateReports(value, target);

                // Self explanatory
            if (aToggle == true)
            ExperimentalBehavior();

            holdCursor = cursor;    // Don't remember why this is a thing

            Vector2 direction = target - cursor;
            float distToMove = SampleRadialCurve(value, direction.Length());    // Where all the magic happens

                // rawThreshold should be negative (or not.) Sets lerpScale to a smootherstep from accel = rawThreshold to accel = something lower
            if (accel / (6 / vDiv) < rawThreshold)
            lerpScale = Smootherstep(accel / (6 / vDiv), rawThreshold, rawThreshold - (1 / (6 / vDiv)));

            if ((aToggle == true) && (indexFactor - lastIndexFactor > (holdVel * explerpconf)))
            lerpScale = Math.Max(lerpScale, Smootherstep(indexFactor - lastIndexFactor, (holdVel * explerpconf), (holdVel * explerpconf) + (1 / (6 / rawv))));  // Don't exactly remember why this is the way it is but it looks like it works
            
            direction = Vector2.Normalize(direction);
            cursor = cursor + Vector2.Multiply(direction, distToMove);
            cursor = LerpedCursor((float)lerpScale, cursor, target);    // Jump to raw report if certain conditions are fulfilled

                // Catch NaNs and pen redetection
            if (!(float.IsFinite(cursor.X) & float.IsFinite(cursor.Y) & stopwatch.Restart().TotalMilliseconds < 50))
                cursor = target;


            if (cLog == true)
            {
                Console.WriteLine("Start of report ----------------------------------------------------");
                Console.WriteLine("Raw Velocity:");
                Console.WriteLine(holdVel);
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

                if (aToggle == true)
                {
                    Console.WriteLine("A bunch of random numbers...");
                    Console.WriteLine(jerk);
                    Console.WriteLine(snap);
                    Console.WriteLine(indexFactor);
                    Console.WriteLine((indexFactor - lastIndexFactor) / holdVel);
                    Console.WriteLine(spinCheck);
                    Console.WriteLine(sinceSnap);
                }
    
                Console.WriteLine("End of report ----------------------------------------------------");
            }

            lerpScale = 0;  // Reset value
            lastCursor = holdCursor;    // Don't remember why this is a thing

                // Reset possibly changed values
            if (aToggle == true)
            AdvancedReset();

            return cursor;
        }


            // Stats from reports
        void UpdateReports(IDeviceReport value, Vector2 target)
        {
            if (value is ITabletReport report)
            {
                last3Report = lastLastReport;
                lastLastReport = lastReport;
                lastReport = currReport;
                currReport = report.Position;

                diff = currReport - lastReport;
                seconddiff = lastReport - lastLastReport;
                thirddiff = lastLastReport - last3Report;

                lastVel = vel;
                vel =  Math.Sqrt(Math.Pow(diff.X, 2) + Math.Pow(diff.Y, 2)) / 100;
                holdVel = vel;

                lastAccel = accel;
                accel = vel - Math.Sqrt(Math.Pow(seconddiff.X, 2) + Math.Pow(seconddiff.Y, 2)) / 100;

                    // Has less use than it probably should.
                lastJerk = jerk;
                jerk = accel - lastAccel;

                snap = jerk - lastJerk;

                    // Angle index doesn't even use angles directly.
                angleIndexPoint = 2 * diff - seconddiff - thirddiff;
                lastIndexFactor = indexFactor;
                indexFactor = Math.Sqrt(Math.Pow(angleIndexPoint.X, 2) + Math.Pow(angleIndexPoint.Y, 2)) / 100;

                accelMult = Smoothstep(accel, -1 / (6 / amvDiv), 0) + Smoothstep(accel, 0, 1 / (6 / amvDiv));   // Usually 1, reaches 0 and 2 under sufficient deceleration and acceleration respecctively
                
            /// You can uncomment for advanced diagnostics.
            //    Console.WriteLine(vel);
            //    Console.WriteLine(accel);
            //    Console.WriteLine(jerk);
            //    Console.WriteLine(snap);
            //    Console.WriteLine("-----------");
            //    Console.WriteLine(angleIndex);
            //    Console.WriteLine(angleIndex - lastIndex);
            //    Console.WriteLine(rOuterAdjusted(value, cursor, rOuter, rInner));
            //    Console.WriteLine("-------------------------------------------");
            }
        }

            // A bunch of requirements where one can be met to do something funny.
        void ExperimentalBehavior()
        {

            sinceSnap += 1;
            if ((Math.Abs(indexFactor) > vel * 2 | (accel / vel > 0.35)) && (vel / rawv > 0.25))
            {
            //    Console.WriteLine("snapping?");
            //    Console.WriteLine(accel / vel);
                sinceSnap = 0;
            }

            last9Vel = last8Vel;

            last8Vel = last7Vel;

            last7Vel = last6Vel;

            last6Vel = last5Vel;

            last5Vel = last4Vel;

            last4Vel = last3Vel;

            last3Vel = last2Vel;

            last2Vel = lastVel;

            spinCheck = Math.Clamp(Math.Pow(vel / (rawv * scConf), 5), 0, 1) +
                        Math.Clamp(Math.Pow(lastVel / (rawv * scConf), 5), 0, 1) +
                        Math.Clamp(Math.Pow(last2Vel / (rawv * scConf), 5), 0, 1) +
                        Math.Clamp(Math.Pow(last3Vel / (rawv * scConf), 5), 0, 1) +
                        Math.Clamp(Math.Pow(last4Vel / (rawv * scConf), 5), 0, 1) +
                        Math.Clamp(Math.Pow(last5Vel / (rawv * scConf), 5), 0, 1) +
                        Math.Clamp(Math.Pow(last6Vel / (rawv * scConf), 5), 0, 1) +
                        Math.Clamp(Math.Pow(last7Vel / (rawv * scConf), 5), 0, 1) +
                        Math.Clamp(Math.Pow(last8Vel / (rawv * scConf), 5), 0, 1) +
                        Math.Clamp(Math.Pow(last9Vel / (rawv * scConf), 5), 0, 1);

            if ((spinCheck > 8) && sinceSnap > 30)
            {
            //    Console.WriteLine("spinning?");
                vel = 0;
                accel = -10 * rawThreshold;
            }

        //    if (indexFactor > Math.Max(1 / (6 / rawv), angidx * vel))
        //    {
        //        if (!((vel > rawv & lastVel > rawv) || 
        //        (accel > (1 / (6 / rawv)) & jerk > (1 / (6 / rawv)) & snap > (1 / (6 / rawv)))))
        //        {
        //        Console.WriteLine("OH MY GOD BRUH");
        //        Console.WriteLine(vel);
        //        }
        //    }

            if ((vel > rawv & lastVel > rawv) || 
                (accel > (1 / (6 / rawv)) & jerk > (1 / (6 / rawv)) & snap > (1 / (6 / rawv))) ||
                (indexFactor > Math.Max(1 / (6 / rawv), angidx * vel)))
            {
                vel *= 10 * vDiv;
                accelMult = 2;
            }
            

        }

        void AdvancedReset()
        {
            vel = Math.Sqrt(Math.Pow(diff.X, 2) + Math.Pow(diff.Y, 2)) / 100;
            accel = vel - Math.Sqrt(Math.Pow(seconddiff.X, 2) + Math.Pow(seconddiff.Y, 2)) / 100;   // This serves no use but might later on.
        }
        
        /// Math functions
        
        double kneeFunc(double x) => x switch
        {
            < -3 => x,
            < 3 => Math.Log(Math.Tanh(Math.Exp(x)), Math.E),
            _ => 0,
        };

        public static double Smoothstep(double x, double start, double end) // Copy pasted out of osu! pp. Thanks StanR 
        {
            x = Math.Clamp((x - start) / (end - start), 0.0, 1.0);

            return x * x * (3.0 - 2.0 * x);
        }

        public static double Smootherstep(double x, double start, double end) // this too
        {
            x = Math.Clamp((x - start) / (end - start), 0.0, 1.0);

            return x * x * x * (x * (6.0 * x - 15.0) + 10.0);
        }

        public static double Lerp(double x, double start, double end)
        {
            x = Math.Clamp(x, 0, 1);
            return start + (end - start) * x;
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
            return knScale switch
            {
                > 0.0001f => (knScale) * kneeFunc(x / (knScale)) + 1,
                _ => x > 0 ? 1 : 1 + x,
            };
        }
        
        double inverseTanh(double x) => Math.Log((1 + x) / (1 - x), Math.E) / 2;

        double inverseKneeScaled(IDeviceReport value, double x) 
        {
            double velocity = 1;
            return (velocity * knScale) * Math.Log(inverseTanh(Math.Exp((x - 1) / (knScale * velocity))), Math.E);
        }

        double derivKneeScaled(IDeviceReport value, double x)
        {
            var e = Math.Exp(x / (knScale));
            var tanh = Math.Tanh(e);
            return (e - e * (tanh * tanh)) / tanh;
        }

        double getXOffset(IDeviceReport value) => inverseKneeScaled(value, 0);

        double getScaleComp(IDeviceReport value) => derivKneeScaled(value, getXOffset(value));

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

        Vector2 cursor;
        Vector2 holdCursor;
        Vector2 lastCursor;
        HPETDeltaStopwatch stopwatch = new HPETDeltaStopwatch(true);

        double xOffset(IDeviceReport value) => getXOffset(value);
        
        double scaleComp(IDeviceReport value) => getScaleComp(value);

        public Vector2 last3Report;
        
        public Vector2 lastLastReport;

        public Vector2 lastReport;

        public Vector2 currReport;

        public Vector2 diff;

        public Vector2 seconddiff;

        public Vector2 thirddiff;

        public double vel;

        public double holdVel;

        public double lastVel;

        public double last2Vel;

        public double last3Vel;

        public double last4Vel;

        public double last5Vel;

        public double last6Vel;

        public double last7Vel;

        public double last8Vel;

        public double last9Vel;

        public double spinCheck;

        public double lastAccel;

        public double accel;

        public double lastJerk;

        public double jerk;

        public double snap;

        public double accelMult;

        public double lerpScale;

        public Vector2 angleIndexPoint;

        public double lastIndexFactor;

        public double indexFactor;

        public double angleIndex;

        public double sinceSnap;
    }
}
