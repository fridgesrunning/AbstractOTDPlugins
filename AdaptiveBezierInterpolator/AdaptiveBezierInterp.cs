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
            set { emaWeight = System.Math.Clamp(value, 0.0f, 1.0f);
            holdEma = emaWeight; }
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
            "Choose a value that is in between these numbers.\n" +
            "Discord for help: shreddism"
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

        [BooleanProperty("Reverse Ground (Enables Below Settings)", ""), DefaultPropertyValue(false), ToolTip
        (
            "When grounding is detected, attempts to use the next two reports to interpolate, instead of default behavior,\n" +
            "and allows alpha to be manipulated in this situation.\n\n" +
            "Alpha is an approximated value from 0 - 1 that goes from 0 to 1 in a report to interpolate between positions.\n" +
            "alpha 0 is the startpoint for the first detected report with grounding. alpha1 is the multiplier given to alpha during this report. (0.5 is effectively 1 but we're interping through 2 reports so its fine)\n" +
            "alpha 2 and alpha 3 are like alpha 0 and alpha 1 respectively, but for the second report with grounded detection.\n" +
            "On the report after this, the normal behavior is changed ever so slightly in hopes of not bugging out, but if you use good settings you shouldn't care about this.\n\n" +
            "Using defaults (0.0 0.5 0.5 0.5), the first alpha starts at 0, finishes at a normal value, then the second starts at a normal value then ends at a normal value.\n" +
            "This appears to be less 'smooth' than default behavior when the checkbox is disabled, but it gives a 'crisper' look.\n" +
            "Other settings that work are (1 0 1 0) for lowest latency, (0 1 1 0) for low latency but smooth, (0 0 1 0) for a fat snap, and (0 0 0 1) for an extra report of hang + 1-report transition.\n\n" +
            "If it bugs out on smooth cursortrail (test with one) then your settings are probably bad. If not, your tablet is reporting too unevenly. Leave this disabled.\n" +
            "If you can't tell a difference, then leave disabled."
        )]
        public bool ReverseGround
        {
            get { return rgToggle; }
            set { rgToggle = value; }
        }
        public bool rgToggle;

        [Property("alpha 0"), DefaultPropertyValue(0.0f), ToolTip
        (
            "alpha"
        )]
        public float AlphaThresholdZero
        {
            get { return aa0; }
            set { aa0 = System.Math.Clamp(value, 0.0f, 1.0f); }
        }
        private float aa0;

        [Property("alpha 1"), DefaultPropertyValue(0.5f), ToolTip
        (
            "alpha"
        )]
        public float AlphaThresholdOne
        {
            get { return aa1; }
            set { aa1 = System.Math.Clamp(value, 0.0f, 1.0f); }
        }
        private float aa1;

        [Property("alpha 2"), DefaultPropertyValue(0.5f), ToolTip
        (
            "alpha"
        )]
        public float AlphaThresholdTwo
        {
            get { return aa2; }
            set { aa2 = System.Math.Clamp(value, 0.0f, 1.0f); }
        }
        private float aa2;

        [Property("alpha 3"), DefaultPropertyValue(0.5f), ToolTip
        (
            "alpha"
        )]
        public float AlphaThresholdThree
        {
            get { return aa3; }
            set { aa3 = System.Math.Clamp(value, 0.0f, 1.0f); }
        }
        private float aa3;

        [BooleanProperty("Wire Update To Consume", ""), DefaultPropertyValue(true), ToolTip
        (
            "Does the same thing as TemporalResampler's 'extraFrames' option. This should be fine."
        )]
        public bool ExtraFrames
        {
            get { return xToggle; }
            set { xToggle = value; }
        }
        public bool xToggle;

        [BooleanProperty("Dot AntiLatency", ""), DefaultPropertyValue(false), ToolTip
        (
            "Very rudimentary, tries to reduce percieved latency on jumps if using smaller radii. Needs Velocity Divisor to be configured correctly.\n" +
            "I mean, so does everything else, but this too."
        )]
        public bool DotBoost
        {
            get { return dToggle; }
            set { dToggle = value; }
        }
        public bool dToggle;

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
                float holdTime = (float)reportStopwatch.Restart().TotalMilliseconds;
                var consumeDelta = holdTime;
                if (consumeDelta < 150)
                    reportMsAvg += ((consumeDelta - reportMsAvg) * 0.1f);



                lastLastEmaTarget = lastEmaTarget;
                lastEmaTarget = emaTarget;
                emaTarget = vec2IsFinite(emaTarget) ? emaTarget : report.Position;
                emaTarget += emaWeight * (report.Position - emaTarget);

                lastVelocity = velocity;
                velocity = (float)Math.Sqrt(Math.Pow(emaTarget.X - lastEmaTarget.X, 2) + Math.Pow(emaTarget.Y - lastEmaTarget.Y, 2)) * (8 / holdTime);
                accel = velocity - (float)Math.Sqrt(Math.Pow(lastEmaTarget.X - lastLastEmaTarget.X, 2) + Math.Pow(lastEmaTarget.Y - lastLastEmaTarget.Y, 2)) * (8 / holdTime);

                if (cLog)
                Console.WriteLine(velocity);

                    lastGround = 0;
                    emaWeight = holdEma;
                    if (groundedIndex > 0.5)
                    {
                        
                        if (groundedClock > 0.5)
                        {
                            holdGround = 1;
                        }
                        else 
                        {
                            lastGround = 1;
                            holdGround = 0;
                            dotVel = velocity;
                        }
                    }
                    groundedIndex = holdGround;
                    groundedClock = 0;

                    if ((Math.Abs(accel) > vDiv))
                    {
                        if (accel > vDiv)
                        {
                            groundedPoint = lastEmaTarget;
                            groundedTarget = new Vector3(groundedPoint, 0);
                            if (groundCount < 1)
                            {
                            groundDirection = Vector2.Normalize(emaTarget - lastEmaTarget);
                            groundedClock = 1;
                            }
                        }
                        groundedIndex = 1;
                    }

                    if (groundedIndex > 0)
                    {
                        groundCount +=1;
                    }
                    else groundCount = 0;
                    
                    if (groundCount > 2)
                    {
                        groundedIndex = 0;
                        groundedClock = 0;
                        groundCount = 0;
                        dotVel = velocity * 2 + 1000000.0f;
                    }

                    if (groundedIndex < 0.5)
                    {
                        if ((velocity > 0.9 * dotVel) && (velocity > vDiv / 4))
                        {   
                            holdDot = (float)Vector2.Dot(groundDirection, Vector2.Normalize(emaTarget - lastEmaTarget));
                            dotCount = 1;
                        }
                        else 
                        {
                            if (dotCount > 1)
                            dotCount = 0;

                            if (dotCount > 0)
                            {
                                dotCount = 2;
                            }
                            else
                            {
                            groundDirection = new Vector2(0, 0);
                            holdDot = 0;
                            }
                        }
                    }

                    if ((velocity < vDiv / 50) && (lastVelocity < vDiv / 50))  // rare failsafe for pen replacement
                    dotCount = 0;

                if (groundCount > 2)
                    {
                    groundedIndex = 0;
                    groundCount = 0;
                    }

                    // I know what you're thinking. This idiot put the same code twice. But it didn't work until I did that. Maybe I just didn't replace the build but I'm keeping it here as a symbol of rememberance for the horrible mess above

                lastControlPoint = controlPoint;
                controlPoint = controlPointNext;
                controlPointNext = new Vector3(emaTarget, report.Pressure);

                previousTarget = target;

                if (dToggle)
                dotAntiLatency = (holdDot * (dotCount / 2));

                target = Vector3.Lerp(controlPoint, controlPointNext, Math.Min(0.5f + ((float)Smootherstep(velocity, vDiv, 0) / 2) + dotAntiLatency, 1));

                if (xToggle)
                UpdateState();
                
            }
            else OnEmit();
        }

        protected override void UpdateState()
        {
            float alpha = (float)(reportStopwatch.Elapsed.TotalSeconds * Frequency / reportMsAvg);

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
                        res = Vector3.Lerp(controlPoint, Vector3.Lerp(target, controlPointNext, aa0 + alpha * aa1), aa0 + alpha * aa1);
                        else
                         {
                            res = Vector3.Lerp(Vector3.Lerp(Vector3.Lerp(previousTarget, controlPoint, aa2 + alpha * aa3), target, aa2 + alpha * aa3), controlPointNext, aa0 + aa1);
                            res = Vector3.Lerp(res, controlPointNext, (float)Smootherstep(velocity / vDiv, 0.1, 0));
                         }
                }
                else
                {
                var lerp1 = Vector3.Lerp(Vector3.Lerp(previousTarget, controlPoint, lastGround), controlPoint, alpha);
                var lerp2 = Vector3.Lerp(controlPoint, target, alpha);
                res = Vector3.Lerp(lerp1, lerp2, (float)Math.Pow(alpha, 1 - Math.Min(((float)Smootherstep(velocity, vDiv, 0) / 2) + dotAntiLatency, 1)));
                }
                report.Position = new Vector2(res.X, res.Y);
                report.Pressure = report.Pressure == 0 ? 0 : (uint)(res.Z);
                State = report;
                OnEmit();
            }
        }

        private Vector2 emaTarget, tiltTraget, previousTiltTraget, lastEmaTarget, lastLastEmaTarget, groundedPoint, groundDirection;
        private Vector3 controlPointNext, controlPoint, lastControlPoint, target, previousTarget, groundedTarget;
        float velocity, accel, lastVelocity, groundedIndex, groundedClock, lastGround, holdEma, groundCount, holdGround, holdDot, dotCount, dotVel, dotAntiLatency;
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
