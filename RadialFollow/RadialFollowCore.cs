/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. */

using System;
using System.Numerics;
using OpenTabletDriver.Plugin.Timing;
using OpenTabletDriver.Plugin.Attributes;
using OpenTabletDriver.Plugin.Output;
using OpenTabletDriver.Plugin.Tablet;

namespace RadialFollow
{
    public class RadialFollowCore
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
            set { vDiv = System.Math.Clamp(value, 0.0f, 1000000.0f);}
        }
        private double vDiv;

        public double MinimumRadiusMultiplier
        {
            get { return minMult; }
            set { minMult = System.Math.Clamp(value, 0.0f, 1.0f);}
        }
        private double minMult;

        public bool VelocityScalesKnee
        {
            get { return vKnee; }
            set { vKnee = value; }
        }
        private bool vKnee;

        public float SampleRadialCurve(IDeviceReport value, float dist) => (float)deltaFn(value, dist, xOffset(value), scaleComp(value));
        public double ResetMs = 1;
        public double GridScale = 1;

        Vector2 cursor;
        HPETDeltaStopwatch stopwatch = new HPETDeltaStopwatch(true);

        public Vector2 Filter(IDeviceReport value, Vector2 target)
        {
            Vector2 direction = target - cursor;
            float distToMove = SampleRadialCurve(value, direction.Length());
            direction = Vector2.Normalize(direction);
            cursor = cursor + Vector2.Multiply(direction, distToMove);

            // Catch NaNs and pen redetection
            if (!(float.IsFinite(cursor.X) & float.IsFinite(cursor.Y) & stopwatch.Restart().TotalMilliseconds < 50))
                cursor = target;

            return cursor;
        }

        double xOffset(IDeviceReport value) => getXOffest(value);
        
        double scaleComp(IDeviceReport value) => getScaleComp(value);
        
        //, scaleComp;
      //  void updateDerivedParams(IDeviceReport value)
      //  {
      //      if (knScale > 0.0001f)
      //      {
      //          xOffset = getXOffest(value);
       //         scaleComp = getScaleComp(value);
       //     }
       //     else // Calculating them with functions would cause / by 0
       //     {
       //         xOffset = -1;
      //          scaleComp = 1;
       //     }
      //  }
        

        /// Math functions

        double kneeFunc(double x) => x switch
        {
            < -3 => x,
            < 3 => Math.Log(Math.Tanh(Math.Exp(x)), Math.E),
            _ => 0,
        };

        double kneeScaled(IDeviceReport value, double x) 
        
        {
            double velocity = 1;
            if (vKnee == true)
            {
            if (value is ITabletReport report)
            {
                Vector2 pos = report.Position / 100;
                Vector2 dih = pos - cursor;
                velocity = Math.Sqrt(Math.Pow(dih.X, 2) + Math.Pow(dih.Y, 2)) + 1;
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
                Vector2 pos = report.Position / 100;
                Vector2 dih = pos - cursor;
                velocity = Math.Sqrt(Math.Pow(dih.X, 2) + Math.Pow(dih.Y, 2)) + 1;
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
                Vector2 pos = report.Position / 100;
                Vector2 dih = pos - cursor;
                velocity = Math.Sqrt(Math.Pow(dih.X, 2) + Math.Pow(dih.Y, 2)) + 1;
            }
            }
            var e = Math.Exp(x / (knScale * velocity));
            var tanh = Math.Tanh(e);
            return (e - e * (tanh * tanh)) / tanh;
        }

        double getXOffest(IDeviceReport value) => inverseKneeScaled(value, 0);

        double getScaleComp(IDeviceReport value) => derivKneeScaled(value, getXOffest(value));



        public double rOuterAdjusted(IDeviceReport value, Vector2 cursor, double rOuter, double rInner) ///=> Math.Sqrt((Math.Pow(pos.X / 15200 - cursor.X / 2560, 2)) + (Math.Pow(pos.Y / 8550 - cursor.Y / 1440, 2))) / Math.Sqrt(Math.Pow(cursor.X, 2) + Math.Pow(cursor.Y, 2)) * GridScale * Math.Max(rOuter, rInner + 0.0001f);
        {
            if (value is ITabletReport report)
            {
                Vector2 pos = report.Position / 100;
                Vector2 dih = pos - cursor;
                double velocity = Math.Sqrt(Math.Pow(dih.X, 2) + Math.Pow(dih.Y, 2));
                return Math.Max(Math.Min(velocity / vDiv, 1), minMult) * Math.Max(rOuter, rInner + 0.0001f);
            }
            else
            return 0;
        }
        public double rInnerAdjusted(IDeviceReport value, Vector2 cursor, double rInner) ///=> Math.Sqrt((Math.Pow(pos.X / 15200 - cursor.X / 2560, 2)) + (Math.Pow(pos.Y / 8550 - cursor.Y / 1440, 2))) / Math.Sqrt(Math.Pow(cursor.X, 2) + Math.Pow(cursor.Y, 2)) * GridScale * rInner;
        {
             if (value is ITabletReport report)
            {
                Vector2 pos = report.Position / 100;
                Vector2 dih = pos - cursor;
                double velocity = Math.Sqrt(Math.Pow(dih.X, 2) + Math.Pow(dih.Y, 2));
                Console.WriteLine(velocity / 5);
                return Math.Max(Math.Min(velocity / vDiv, 1), minMult) * rInner;
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
                Vector2 pos = report.Position / 100;
                Vector2 dih = pos - cursor;
                velocity = Math.Sqrt(Math.Pow(dih.X, 2) + Math.Pow(dih.Y, 2));
                LowVelocityUnsmooth = velocity < vDiv ?  1 + (Math.Abs(velocity - vDiv) / vDiv) : 1;
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
        //=> (rOuterAdjusted - rInnerAdjusted) * smoothedFn(x / (rOuterAdjusted - rInnerAdjusted), offset, scaleComp);

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
        //=> x > rInnerAdjusted ? x - scaleToOuter(x - rInnerAdjusted, offset, scaleComp) - rInnerAdjusted : 0;
    }
}
