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

        [Property("Velocity Divisor"), DefaultPropertyValue(5.0f), ToolTip
        (
            "Make it the same. If you don't know what this means, what are you doing?"
        )]
        public float VelocityDivisor
        {
            get { return vDiv; }
            set { vDiv = System.Math.Clamp(10 * value, 1.0f, 1000000.0f); }
        }
        private float vDiv;

        [Property("Aggressiveness"), DefaultPropertyValue(0.75f), ToolTip
        (
            "For testing purposes. Leave it at 0.75 unless you know what you're doing (read source code)"
        )]
        public float Aggressiveness
        {
            get { return lerpMax; }
            set { lerpMax = System.Math.Clamp(value, 0.0f, 1.0f); }
        }
        private float lerpMax;

        protected override void UpdateState()
        {
            float alpha = (float)(reportStopwatch.Elapsed.TotalSeconds * Frequency / reportMsAvg);
            alpha = (float)Math.Pow(alpha, Math.Pow(2, -1 * Math.Max(accel / (vDiv / 4), 0)));

            if (State is ITiltReport tiltReport)
            {
                tiltReport.Tilt = Vector2.Lerp(previousTiltTraget, tiltTraget, alpha);
                State = tiltReport;
            }

            if (State is ITabletReport report && PenIsInRange())
            {
                var lerp1 = Vector3.Lerp(previousTarget, controlPoint, alpha);
                var lerp2 = Vector3.Lerp(controlPoint, target, alpha);
                var res = Vector3.Lerp(lerp1, lerp2, alpha);
                res = Vector3.Lerp(res, controlPointNext, (float)Math.Clamp(Math.Pow(((accel + 1 * (6 / vDiv)) * (velocity / 10)) / (6 * vDiv), 9), 0, lerpMax));
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

                velocity = (float)Math.Sqrt(Math.Pow(emaTarget.X - lastEmaTarget.X, 2) + Math.Pow(emaTarget.Y - lastEmaTarget.Y, 2));
                accel = velocity - (float)Math.Sqrt(Math.Pow(lastEmaTarget.X - lastLastEmaTarget.X, 2) + Math.Pow(lastEmaTarget.Y - lastLastEmaTarget.Y, 2));

                controlPoint = controlPointNext;
                controlPointNext = new Vector3(emaTarget, report.Pressure);

                previousTarget = target;
                target = Vector3.Lerp(controlPoint, controlPointNext, (float)Math.Pow(0.5, Math.Pow(2, -1 * Math.Max(accel + (velocity / 2) / (vDiv / 4), 0))));
                
            }
            else OnEmit();
        }

        private Vector2 emaTarget, tiltTraget, previousTiltTraget, lastEmaTarget, lastLastEmaTarget;
        private Vector3 controlPointNext, controlPoint, target, previousTarget;
        float velocity, accel;
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
