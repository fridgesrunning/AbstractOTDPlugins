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

        public double xDivisor
        {
            get { return xDiv; }
            set { xDiv = System.Math.Clamp(value, 0.01f, 100.0f); }
        }
        private double xDiv;

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
            set { accPower = System.Math.Clamp(value, 1.0f, 100.0f); }
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
            set { rawv =  System.Math.Clamp(value, 0.01f, 1000000.0f); }
        }
        public double rawv;

        public double aidx
        {
            get { return angidx; }
            set { angidx = System.Math.Clamp(value, 0.1f, 1000000.0f); }
        }
        public double angidx;

        public double xlerpconf
        {
            get { return explerpconf; }
            set { explerpconf = System.Math.Clamp(value, 0.1f, 1000000.0f); }
        }
        public double explerpconf;

        public double accelMultVelocityOverride
        {
            get { return amvDiv; }
            set { amvDiv = vDiv;

            if (aToggle == true)
            amvDiv = System.Math.Clamp(value, 0.1f, 1000000.0f);  }
        }
        public double amvDiv;

        public double spinCheckConfidence
        {
            get { return scConf; }
            set { scConf = System.Math.Clamp(value, 0.1f, 1.0f); }
        }
        public double scConf;

        public bool groundedBehavior
        {
            get { return rToggle; }
            set { rToggle = value; }
        }
        public bool rToggle;

        public bool extratoggle1
        {
            get { return xt1; }
            set { xt1 = value; }
        }
        public bool xt1;

        public double extrastuffg
        {
            get { return xng; }
            set { xng = value; }
        }
        public double xng;

        public bool extratoggle2
        {
            get { return xt2; }
            set { xt2 = value; }
        }
        public bool xt2;

        public bool extratoggle3
        {
            get { return xt3; }
            set { xt3 = value; }
        }
        public bool xt3;

        public float SampleRadialCurve(IDeviceReport value, float dist) => (float)deltaFn(value, dist, xOffset(value), scaleComp(value));
        public double ResetMs = 1;
        public double GridScale = 1;

        public Vector2 Filter(IDeviceReport value, Vector2 target)
        {
            double holdTime = stopwatch.Restart().TotalMilliseconds;
                var consumeDelta = holdTime;
                if (consumeDelta < 150) {
                    reportMsAvg += ((consumeDelta - reportMsAvg) * 0.1f);
                }

            UpdateReports(value, target);

            if (aToggle == true) {
                AdvancedBehavior();
                if (rToggle == true) {
                    GroundedRadius(value, target); 
                }
            }
            holdCursor = cursor;

            Vector2 direction = target - cursor;
            float distToMove = SampleRadialCurve(value, direction.Length());

            if (accel0 / (6 / vDiv) < rawThreshold)
            lerpScale = Smootherstep(accel0 / (6 / vDiv), rawThreshold, rawThreshold - (1 / (6 / vDiv)));

            if ((aToggle == true) && (indexFactor0 - indexFactor1 > (hold1Vel0 * explerpconf)))
            lerpScale = Math.Max(lerpScale, Smootherstep(indexFactor0 - indexFactor1, (hold1Vel0 * explerpconf), (hold1Vel0 * explerpconf) + (1 / (6 / rawv))));
            
            direction = Vector2.Normalize(direction);
            cursor = cursor + Vector2.Multiply(direction, distToMove);
            cursor = Vector2.Lerp(cursor, target, (float)lerpScale);

            if (!(float.IsFinite(cursor.X) & float.IsFinite(cursor.Y) & holdTime < 50))
                cursor = target;


            if (cLog == true) {
                Console.WriteLine("Start of report ----------------------------------------------------");
                Console.WriteLine("Raw Velocity:");
                Console.WriteLine(hold1Vel0);
                Console.WriteLine("Raw Acceleration:");
                Console.WriteLine(accel0);
                Console.WriteLine("Accel Mult (this is an additional factor that multiplies velocity, should be close to or 0 on sharp decel, hovering around 1 when neutral, and close to or 2 on sharp accel0. Only affected by power on radius scaling, so not shown.):");
                Console.WriteLine(accelMult);
                Console.WriteLine("Outside Radius:");
                Console.WriteLine(rOuterAdjusted(value, cursor, rOuter, rInner));
                Console.WriteLine("Inner Radius:");
                Console.WriteLine(rInnerAdjusted(value, cursor, rInner));
                Console.WriteLine("Smoothing Coefficient:");
                Console.WriteLine(smoothCoef / (1 + (Smoothstep(vel0 * accelMult, vDiv, 0) * (minSmooth - 1))));
                Console.WriteLine("Sharp Decel Lerp (With sharp decel, cursor is lerped between calculated value and raw report using this scale):");
                Console.WriteLine(lerpScale);

                if (aToggle == true) {
                    Console.WriteLine("A bunch of random numbers...");
                    Console.WriteLine(jerk0);
                    Console.WriteLine(indexFactor0);
                    Console.WriteLine((indexFactor0 - indexFactor1) / hold1Vel0);
                    Console.WriteLine(spinCheck);
                    Console.WriteLine(sinceSnap);
                    Console.WriteLine(Math.Log((Math.Pow(vel1 / xng + 1, xng)) + 1));
                }
    
                Console.WriteLine("End of report ----------------------------------------------------");
            }

            lerpScale = 0;
            lastCursor = holdCursor;

            if (aToggle == true)
            AdvancedReset();

            

            return cursor;
        }

        void UpdateReports(IDeviceReport value, Vector2 target) {
            if (value is ITabletReport report) {

                pos3 = pos2;
                pos2 = pos1;
                pos1 = pos0;
                pos0 = report.Position;

                pressure1 = pressure0;
                pressure0 = report.Pressure;

                dir2 = dir1;
                dir1 = dir0;
                dir0 = pos0 - pos1;

                pointAccel2 = pointAccel1;
                pointAccel1 = pointAccel0;
                pointAccel0 = dir0 - dir1;

                vel1 = vel0;
                vel0 =  ((Math.Sqrt(Math.Pow(dir0.X / xDiv, 2) + Math.Pow(dir0.Y, 2)) / 12.5) / reportMsAvg);
                hold1Vel0 = vel0;

                accel1 = accel0;
                accel0 = vel0 - ((Math.Sqrt(Math.Pow(dir1.X / xDiv, 2) + Math.Pow(dir1.Y, 2)) / 12.5) / reportMsAvg);

                jerk1 = jerk0;
                jerk0 = accel0 - accel1;

                angleIndexPoint = 2 * dir0 - dir1 - dir2;
                indexFactor1 = indexFactor0;
                indexFactor0 = (Math.Sqrt(Math.Pow(angleIndexPoint.X / xDiv, 2) + Math.Pow(angleIndexPoint.Y, 2)) / 12.5) / reportMsAvg;

                if (accel0 < 0 && accel1 > 0) {
                    velPeakDir = dir1;
                    spinninglikely = 0;
                }

                if (accel0 > 0 && accel1 < 0) {
                    spinninglikely = ((vel1 + vel0) / (1.5 * rawv));
                }

                /*Console.WriteLine(Vector2.Distance(dir0, velPeakDir));
                Console.WriteLine(velPeakDir.Length());
                Console.WriteLine(dir0.Length());
                Console.WriteLine("----------------------"); */


                if ((accel0 > 0 && accel1 < 0 && pointAccel1.Length() + pointAccel0.Length() > 3)) {
                    dirFactor = 1.25 * Smootherstep(dir0.Length() / velPeakDir.Length(), 0.6, 0.4);
                    if (pointAccel0.Length() > 5) {
                        dirFactor *= Math.Max(0, Math.Pow(Vector2.Dot(Vector2.Normalize(pointAccel0), Vector2.Normalize(Vector2.Zero - velPeakDir)), 1));
                    }
                }
                else if (accel0 < 0) {
                    dirFactor = 1;
                    if (Vector2.Distance(dir0, velPeakDir) > velPeakDir.Length() * 0.75) {
                        dirFactor = 1.25 * Smootherstep(dir0.Length() / velPeakDir.Length(), 0.6, 0.4);
                        if (pointAccel0.Length() > 5) {
                        dirFactor *= Math.Max(0, Math.Pow(Vector2.Dot(Vector2.Normalize(pointAccel0), Vector2.Normalize(Vector2.Zero - velPeakDir)), 1));
                    }
                    }
                }

                if (xt1)
                accelMult = Smootherstep((accel0 * dirFactor), -1 / (6 / amvDiv), 0) + Smootherstep((accel0 * dirFactor) / Math.Log((Math.Pow(vel1 / xng + 1, xng)) + 1), 0, 1 / (6 / amvDiv));
                else accelMult = Smootherstep((accel0 * dirFactor), -1 / (6 / amvDiv), 0) + Smootherstep((accel0 * dirFactor), 0, 1 / (6 / amvDiv));
            }
        }

        void AdvancedBehavior()
        {

            sinceSnap += 1;
            if ((Math.Abs(indexFactor0) > vel0 * 2 | (accel0 / vel0 > 0.35)) && (vel0 / rawv > 0.25)) {
                sinceSnap = 0;
            }

            vel9 = vel8;

            vel8 = vel7;

            vel7 = vel6;

            vel6 = vel5;

            vel5 = vel4;

            vel4 = vel3;

            vel3 = vel2;

            vel2 = vel1;

            spinCheck = Math.Clamp(Math.Pow(vel0 / (rawv * scConf), 5), 0, 1) +
                        Math.Clamp(Math.Pow(vel1 / (rawv * scConf), 5), 0, 1) +
                        Math.Clamp(Math.Pow(vel2 / (rawv * scConf), 5), 0, 1) +
                        Math.Clamp(Math.Pow(vel3 / (rawv * scConf), 5), 0, 1) +
                        Math.Clamp(Math.Pow(vel4 / (rawv * scConf), 5), 0, 1) +
                        Math.Clamp(Math.Pow(vel5 / (rawv * scConf), 5), 0, 1) +
                        Math.Clamp(Math.Pow(vel6 / (rawv * scConf), 5), 0, 1) +
                        Math.Clamp(Math.Pow(vel7 / (rawv * scConf), 5), 0, 1) +
                        Math.Clamp(Math.Pow(vel8 / (rawv * scConf), 5), 0, 1) +
                        Math.Clamp(Math.Pow(vel9 / (rawv * scConf), 5), 0, 1);

            if ( // (vel0 > rawv & vel1 > rawv) || 
                (accel0 > (1 / (6 / rawv) * 0.5 + 0.5 * Smoothstep(DotNorm(pointAccel0, dir1), 1, 0.5)) & jerk0 > (1 / (6 / rawv) * 0.5 + 0.5 * Smoothstep(DotNorm(pointAccel0, dir1), 1, 0.5))) ||
                (indexFactor0 > Math.Max(1 / (6 / rawv), angidx * vel0)))
            {
                vel0 *= 10 * vDiv;
                accelMult = 2;
            }

            hold2Vel0 = vel0;

            if ((distanceGround < rOuter) & (vel0 > rawv & vel1 > rawv)) {
                vel0 *= 10 * vDiv;
                accelMult = 2;
            }

            if ((spinCheck > 8) && sinceSnap > 30 || spinninglikely > 1) {
                vel0 = 0;
                accel0 = -10 * rawThreshold;
            }

            if (xt2) {
                if (pressure0 > 0 && pressure1 == 0) {
                    emergency = 3;
                    vel0 = vDiv / 10;
                    accelMult = 0;
                    accel0 = 0;
                }
                else if (pressure1 > 0 && pressure0 == 0) {
                    emergency = 3;
                    vel0 = vDiv / 10;
                    accelMult = 0;
                    accel0 = 0;
                }
                else if (emergency > 0) {
                    emergency--;
                    vel0 = vDiv / 10;
                    accelMult = 0;
                    accel0 = 0;
                }                
            }

            if (xt3) {
                if (jerk1 + jerk0 < 0 && vel0 < rawv / 6)
                accelMult = 0;
            }
        }

        void GroundedRadius(IDeviceReport value, Vector2 target) {
            if (hold2Vel0 * Math.Pow(accelMult, accPower) < vDiv) {
                radiusGroundCount = 0;
                distanceGround = 0;
            }
                else radiusGroundCount += 1;

            if (accelMult < 1.99) {
                sinceAccelPeak = 0;
            }
            else sinceAccelPeak += 1;

            if (hold2Vel0 * Math.Pow(accelMult, accPower) >= vDiv) {
                if ((radiusGroundCount <= 1) || 
                (vel0 > rawv & vel1 > rawv) && 
                ((accel0 > (1 / (6 / rawv)) & jerk0 > (1 / (6 / rawv))) ||
                (indexFactor0 > Math.Max(1 / (6 / rawv), angidx * vel0)) ||
                (sinceAccelPeak > 0)))
                {
                    groundedPoint = cursor;
                }
                groundedDist = target - groundedPoint;
                distanceGround = Math.Sqrt(Math.Pow(groundedDist.X, 2) + Math.Pow(groundedDist.Y, 2));
                    
            }
            if (distanceGround > rOuter) {
                vel0 = 0;
                accel0 = -10 * rawThreshold;
            }
        }

        void AdvancedReset() {
            vel0 =  ((Math.Sqrt(Math.Pow(dir0.X / xDiv, 2) + Math.Pow(dir0.Y, 2)) / 12.5) / reportMsAvg);
            accel0 = vel0 - ((Math.Sqrt(Math.Pow(dir1.X / xDiv, 2) + Math.Pow(dir1.Y, 2)) / 12.5) / reportMsAvg);   // This serves no use but might later on.
        }
        
        /// Math functions
        
        double kneeFunc(double x) => x switch {
            < -3 => x,
            < 3 => Math.Log(Math.Tanh(Math.Exp(x)), Math.E),
            _ => 0,
        };

        public static double Smoothstep(double x, double start, double end) {
            x = Math.Clamp((x - start) / (end - start), 0.0, 1.0);

            return x * x * (3.0 - 2.0 * x);
        }

        public static double Smootherstep(double x, double start, double end) {
            x = Math.Clamp((x - start) / (end - start), 0.0, 1.0);

            return x * x * x * (x * (6.0 * x - 15.0) + 10.0);
        }

        public static double Lerp(double x, double start, double end) {
            x = Math.Clamp(x, 0, 1);
            return start + (end - start) * x;
        }

        double kneeScaled(IDeviceReport value, double x) {
            return knScale switch {
                > 0.0001f => (knScale) * kneeFunc(x / (knScale)) + 1,
                _ => x > 0 ? 1 : 1 + x,
            };
        }
        
        double inverseTanh(double x) => Math.Log((1 + x) / (1 - x), Math.E) / 2;

        double inverseKneeScaled(IDeviceReport value, double x) {
            double velocity = 1;
            return (velocity * knScale) * Math.Log(inverseTanh(Math.Exp((x - 1) / (knScale * velocity))), Math.E);
        }

        double derivKneeScaled(IDeviceReport value, double x) {
            var e = Math.Exp(x / (knScale));
            var tanh = Math.Tanh(e);
            return (e - e * (tanh * tanh)) / tanh;
        }

        double getXOffset(IDeviceReport value) => inverseKneeScaled(value, 0);

        double getScaleComp(IDeviceReport value) => derivKneeScaled(value, getXOffset(value));

        public double rOuterAdjusted(IDeviceReport value, Vector2 cursor, double rOuter, double rInner) {
            if (value is ITabletReport report) {
                double velocity = vel0 * Math.Pow(accelMult, accPower);
                return Math.Max(Math.Min(Math.Pow(velocity / vDiv, radPower), 1), minMult) * Math.Max(rOuter, rInner + 0.0001f);
            }
            else {
                return 0;
            }
        }

        public double rInnerAdjusted(IDeviceReport value, Vector2 cursor, double rInner) {
            if (value is ITabletReport report) {
                double velocity = vel0 * Math.Pow(accelMult, accPower);
                return Math.Max(Math.Min(Math.Pow(velocity / vDiv, radPower), 1), minMult) * rInner;
            }
            else {
                return 0;
            }
        }

        double leakedFn(IDeviceReport value, double x, double offset, double scaleComp)
        => kneeScaled(value, x + offset) * (1 - leakCoef) + x * leakCoef * scaleComp;

        double smoothedFn(IDeviceReport value, double x, double offset, double scaleComp) {
            double velocity = 1;
            double LowVelocityUnsmooth = 1;
            if (value is ITabletReport report) {
                velocity = vel0;
                LowVelocityUnsmooth = 1 + (Smoothstep(vel0 * accelMult, vDiv, 0) * (minSmooth - 1));
            }

            return leakedFn(value, x * (smoothCoef / LowVelocityUnsmooth) / scaleComp, offset, scaleComp);
        }

        double scaleToOuter(IDeviceReport value, double x, double offset, double scaleComp) {
            if (value is ITabletReport report) {
                return (rOuterAdjusted(value, cursor, rOuter, rInner) - rInnerAdjusted(value, cursor, rInner)) * smoothedFn(value, x / (rOuterAdjusted(value, cursor, rOuter, rInner) - rInnerAdjusted(value, cursor, rInner)), offset, scaleComp);
            }
            else {
                return (rOuter - rInner) * smoothedFn(value, x / (rOuter - rInner), offset, scaleComp);
            } 
        }

        double deltaFn(IDeviceReport value, double x, double offset, double scaleComp) {
            if (value is ITabletReport report) {
                return x > rInnerAdjusted(value, cursor, rInner) ? x - scaleToOuter(value, x - rInnerAdjusted(value, cursor, rInner), offset / 1, scaleComp / 1) - rInnerAdjusted(value, cursor, rInner) : 0;
            }
            else {
                return x > rInner ? x - scaleToOuter(value, x - rInner, offset, scaleComp) - rInnerAdjusted(value, cursor, rInner) : 0;
            }
        }

        double DotNorm(Vector2 a, Vector2 b) {
            return Vector2.Dot(Vector2.Normalize(a), Vector2.Normalize(b));
        }

        Vector2 cursor;
        Vector2 holdCursor;
        Vector2 lastCursor;
        HPETDeltaStopwatch stopwatch = new HPETDeltaStopwatch(true);

        double xOffset(IDeviceReport value) => getXOffset(value);
        
        double scaleComp(IDeviceReport value) => getScaleComp(value);

        public Vector2 pos0, pos1, pos2, pos3, dir0, dir1, dir2;
        public Vector2 groundedPoint, groundedDist, angleIndexPoint;
        public double vel0, hold1Vel0, hold2Vel0, vel1, vel2, vel3, vel4, vel5, vel6, vel7, vel8, vel9;
        public double accel0, accel1;
        public double jerk0, jerk1;
        public double accelMult, lerpScale;
        public double sinceAccelPeak;
        public double reportMsAvg = 5;
        public Vector2 pointAccel0, pointAccel1, pointAccel2;
        public double distanceGround, indexFactor0, indexFactor1, radiusGroundCount, radiusGroundPosition, spinCheck, angleIndex, sinceSnap;
        public Vector2 velPeakDir;
        public double dirFactor;
        public Vector2 cluster;
        public double spinninglikely;
        uint pressure0, pressure1, emergency;

        /*public Vector2 pos3;   
        public Vector2 pos2;
        public Vector2 pos1;
        public Vector2 pos0;
        public Vector2 dir0;
        public Vector2 dir1;
        public Vector2 dir2;
        public double vel0;
        public double hold1Vel0;
        public double hold2Vel0;
        public double vel1;
        public double vel2;
        public double vel3;
        public double vel4;
        public double vel5;
        public double vel6;
        public double vel7;
        public double vel8;
        public double vel9;
        public double spinCheck;
        public double accel1;
        public double accel0;
        public double jerk1;
        public double jerk0;
        public double accelMult;
        public double lerpScale;
        public Vector2 angleIndexPoint;
        public double indexFactor1;
        public double indexFactor0;
        public double angleIndex;
        public double sinceSnap;
        public double radiusGroundCount;
        public double radiusGroundPosition;
        public Vector2 groundedPoint;
        public Vector2 groundedDist;
        public double distanceGround;
        public double doubt;
        public double sinceAccelPeak;
        private double reportMsAvg = 5; */
    }
}
