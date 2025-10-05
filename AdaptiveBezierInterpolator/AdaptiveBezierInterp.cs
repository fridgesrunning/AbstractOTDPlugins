using System;
using System.Numerics;
using OpenTabletDriver.Plugin.Attributes;
using OpenTabletDriver.Plugin.Output;
using OpenTabletDriver.Plugin.Tablet;
using OpenTabletDriver.Plugin.Timing;

namespace AdaptiveBezierInterpolator
{
    [PluginName("AdaptiveBezierInterpolator")]
    public class AdaptiveBezierInterp : AsyncPositionedPipelineElement<IDeviceReport>
    {
        public AdaptiveBezierInterp() : base()
        {
        }

        public override PipelinePosition Position => PipelinePosition.PostTransform;

        [Property("Pre-interpolation smoothing factor"), DefaultPropertyValue(1.0f), ToolTip
        (
            "Sets the factor of pre-interpolation simple exponential smoothing (aka EMA weight).\n\n" +
            "Possible values are 0.01 .. 1\n" +
            "Factor of 1 means no smoothing is applied, smaller values add smoothing."
        )]
        public float SmoothingFactor
        {
            get { return emaWeight; }
            set { emaWeight = System.Math.Clamp(value, 0.0f, 1.0f); }
        }
        private float emaWeight;

        [Property("Tilt smoothing factor"), DefaultPropertyValue(1.0f), ToolTip
        (
            "For tablets that report tilt, sets the factor of simple exponential smoothing (aka EMA weight) for tilt values.\n\n" +
            "Possible values are 0.01 .. 1\n" +
            "Factor of 1 means no smoothing is applied, smaller values add smoothing."
        )]
        public float TiltSmoothingFactor
        {
            get { return tiltWeight; }
            set { tiltWeight = System.Math.Clamp(value, 0.0f, 1.0f); }
        }
        private float tiltWeight;

        [Property("Velocity Divisor (HOVER OVER THE TEXTBOX!!!)"), DefaultPropertyValue(100.0f), ToolTip
        (
            "Enable ARF with your preferred settings (default behavior heavily recommended), but if you have an inside radius of 0, then consider increasing it slightly if you want to use this.\n" +
            "Enable the checkbox below.\n" +
            "Apply + save. Close OTD. Open OTD Daemon. Wave your cursor around.\n" +
            "There should be a bunch of reports. These are all distance traveled since the last report filtered by ARF, maybe in pixels?\n" +
            "There should be a bunch of zeroes, then a big number.\n" +
            "Flow aim your cursor around. These should be smaller numbers.\n" +
            "Choose a value that is in between these numbers."
        )]
        public float VelocityDivisor
        {
            get { return vDiv; }
            set { vDiv = System.Math.Clamp(value, 0.1f, 1000000.0f); }
        }
        private float vDiv;

       [BooleanProperty("Console Logging", ""), DefaultPropertyValue(false), ToolTip
        (
            "Reports velocity."
        )]
        public bool ConsoleLogging
        {
            get { return cLog; }
            set { cLog = value; }
        }
        public bool cLog;

        [BooleanProperty("Reverse Ground", ""), DefaultPropertyValue(false), ToolTip
        (
            "Don't know what the true effect of this is. Small chance to bug. Try and see if you like it.\n" +
            "Not recommended if you have a smooth cursortrail. Looks bad."
        )]
        public bool ReverseGround
        {
            get { return rgToggle; }
            set { rgToggle = value; }
        }
        public bool rgToggle;

        protected override void UpdateState()
        {
            float alpha = (float)(reportStopwatch.Elapsed.TotalSeconds * Frequency / reportMsAvg);
     //       alpha = (float)Math.Pow(alpha, Math.Pow(Math.Abs(accel / vDiv), 2) + 1);

            if (State is ITiltReport tiltReport)
            {
                tiltReport.Tilt = Vector2.Lerp(previousTiltTraget, tiltTraget, alpha);
                State = tiltReport;
            }

            if (State is ITabletReport report && PenIsInRange())
            {
                var res = new Vector3(0, 0, 0);
                alpha = (float)Math.Clamp(alpha, 0, 1);
                if (rgToggle && groundedIndex > 0.5)
                {
                    if (groundedClock > 0.5)
                        res = Vector3.Lerp(controlPoint, controlPointNext, alpha * 0.35f);
                        else res = Vector3.Lerp(Vector3.Lerp(lastControlPoint, controlPoint, alpha), target, 0.4f + alpha * 0.25f);

                }
                else
                {
                var lerp1 = Vector3.Lerp(Vector3.Lerp(previousTarget, controlPoint, lastGround), controlPoint, alpha);
                var lerp2 = Vector3.Lerp(controlPoint, target, alpha);
                res = Vector3.Lerp(lerp1, lerp2, (float)Math.Pow(alpha, 1 - (float)Smootherstep(velocity, vDiv, 0) / 2));
                }
                report.Position = new Vector2(res.X, res.Y);
                report.Pressure = report.Pressure == 0 ? 0 : (uint)(res.Z);
                State = report;
                OnEmit();
            }
        }

        protected override void ConsumeState()
        {
            if (State is ITiltReport tiltReport)
            {
                if (!vec2IsFinite(tiltTraget)) tiltTraget = tiltReport.Tilt;
                previousTiltTraget = tiltTraget;
                tiltTraget += tiltWeight * (tiltReport.Tilt - tiltTraget);
            }

            if (State is ITabletReport report)
            {
                var consumeDelta = (float)reportStopwatch.Restart().TotalMilliseconds;
                if (consumeDelta < 150)
                    reportMsAvg += ((consumeDelta - reportMsAvg) * 0.1f);



                lastLastEmaTarget = lastEmaTarget;
                lastEmaTarget = emaTarget;
                emaTarget = vec2IsFinite(emaTarget) ? emaTarget : report.Position;
                emaTarget += emaWeight * (report.Position - emaTarget);

                lastVelocity = velocity;
                velocity = (float)Math.Sqrt(Math.Pow(emaTarget.X - lastEmaTarget.X, 2) + Math.Pow(emaTarget.Y - lastEmaTarget.Y, 2));
                accel = velocity - (float)Math.Sqrt(Math.Pow(lastEmaTarget.X - lastLastEmaTarget.X, 2) + Math.Pow(lastEmaTarget.Y - lastLastEmaTarget.Y, 2));

                if (cLog)
                Console.WriteLine(velocity);

                if (rgToggle)
                {
                    lastGround = 0;
                    if (groundedIndex > 0.5 && groundedClock < 0.5)
                    lastGround = 1;
                
                    groundedIndex = 0;
                    groundedClock = 0;

                    if ((Math.Abs(accel) > vDiv))
                    {
                        if (accel > vDiv)
                        {
                            groundedPoint = lastEmaTarget;
                            groundedTarget = new Vector3(groundedPoint, 0);
                            groundedClock = 1;
                        }
                        groundedIndex = 1;
                    }

                }

                lastControlPoint = controlPoint;
                controlPoint = controlPointNext;
                controlPointNext = new Vector3(emaTarget, report.Pressure);


                previousTarget = target;
                target = Vector3.Lerp(controlPoint, controlPointNext, 0.5f + (float)Smootherstep(velocity, vDiv, 0) / 2);
                
                
            }
            else OnEmit();
        }

        private Vector2 emaTarget, tiltTraget, previousTiltTraget, lastEmaTarget, lastLastEmaTarget, groundedPoint;
        private Vector3 controlPointNext, controlPoint, lastControlPoint, target, previousTarget, groundedTarget;
        float velocity, accel, lastVelocity, groundedIndex, groundedClock, lastGround;
        private HPETDeltaStopwatch reportStopwatch = new HPETDeltaStopwatch();
        private float reportMsAvg = 5;

        private bool vec2IsFinite(Vector2 vec) => float.IsFinite(vec.X) & float.IsFinite(vec.Y);

        public static double Smootherstep(double x, double start, double end)
        {
            x = Math.Clamp((x - start) / (end - start), 0.0, 1.0);

            return x * x * x * (x * (6.0 * x - 15.0) + 10.0);
        }
    }
}
