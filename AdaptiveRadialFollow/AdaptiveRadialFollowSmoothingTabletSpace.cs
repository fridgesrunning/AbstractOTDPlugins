using System;
using System.Numerics;
using OpenTabletDriver.Plugin.Attributes;
using OpenTabletDriver.Plugin.Output;
using OpenTabletDriver.Plugin.Tablet;

namespace AdaptiveRadialFollow
{
    [PluginName("Adaptive Radial Follow Smoothing (Tablet Space)")]
    public class AdaptiveRadialFollowSmoothingTabletSpace : IPositionedPipelineElement<IDeviceReport>
    {
        public AdaptiveRadialFollowSmoothingTabletSpace() : base() { }
        public PipelinePosition Position => PipelinePosition.PreTransform;

        [Property("X Divisor (Important) (Hover Over The Textbox)"), DefaultPropertyValue(1.0d), ToolTip
        (
            "https://www.desmos.com/calculator/pw0r8hvezt\n\n" +
            "Accounts for the discrepancy between tablet aspect ratio and screen aspect ratio.\n" +
            "Default value is 1.0\n" +
            "Discord for help: shreddism"
        )]
        public double xDivisor
        {
            get => radialCore.xDivisor;
            set { radialCore.xDivisor = value; }
        }

        [Property("Outer Radius (Important)"), DefaultPropertyValue(10.0d), Unit("mm"), ToolTip
        (
            "https://www.desmos.com/calculator/pw0r8hvezt\n\n" +
            "This scales with cursor velocity by default. \n\n" +
            "Outer radius defines the max distance the cursor can lag behind the actual reading.\n\n" +
            "Unit of measurement is millimetres.\n" +
            "The value should be >= 0 and inner radius.\n" +
            "If smoothing leak is used, defines the point at which smoothing will be reduced,\n" +
            "instead of hard clamping the max distance between the tablet position and a cursor.\n\n" +
            "That was the original description. This should be equal to inner radius in most cases.\n\n" +
            "Default value is 10.0 mm"
        )]
        public double OuterRadius
        {
            get => radialCore.OuterRadius;
            set { radialCore.OuterRadius = value; }
        }

        [Property("Inner Radius (Important)"), DefaultPropertyValue(10.0d), Unit("mm"), ToolTip
        (
            "https://www.desmos.com/calculator/pw0r8hvezt\n\n" +
            "This scales with cursor velocity by default. \n\n" +
            "Inner radius defines the max distance the tablet reading can deviate from the cursor without moving it.\n" +
            "This effectively creates a deadzone in which no movement is produced.\n\n" +
            "Unit of measurement is millimetres.\n" +
            "The value should be >= 0 and <= outer radius.\n\n" +
            "That was the original description. This should be equal to outer radius in most cases.\n\n" +
            "Default value is 10.0 mm"
        )]
        public double InnerRadius
        {
            get => radialCore.InnerRadius;
            set { radialCore.InnerRadius = value; }
        }

        [Property("Initial Smoothing Coefficient"), DefaultPropertyValue(0.5d), ToolTip
        (
            "Smoothing coefficient determines how fast or slow the cursor will descend from the outer radius to the inner.\n\n" +
            "Possible value range is 0.0001..1, higher values mean more smoothing (slower descent to the inner radius).\n\n" +
            "Default value is 0.5"
        )]
        public double SmoothingCoefficient
        {
            get => radialCore.SmoothingCoefficient;
            set { radialCore.SmoothingCoefficient = value; }
        }

        [Property("Soft Knee Scale"), DefaultPropertyValue(1.0d), ToolTip
        (
            "Soft knee scale determines how soft the transition between smoothing inside and outside the outer radius is.\n\n" +
            "Possible value range is 0..100, higher values mean softer transition.\n" +
            "The effect is somewhat logarithmic, i.e. most of the change happens closer to zero.\n\n" +
            "Default value is 1"
        )]
        public double SoftKneeScale
        {
            get => radialCore.SoftKneeScale;
            set { radialCore.SoftKneeScale = value; }
        }

        [Property("Smoothing Leak Coefficient"), DefaultPropertyValue(0.0d), ToolTip
        (
            "Smoothing leak coefficient allows for input smoothing to continue past outer radius at a reduced rate.\n\n" +
            "Possible value range is 0..1, 0 means no smoothing past outer radius, 1 means 100% of the smoothing gets through.\n\n" +
            "Note that this probably shouldn't be above 0.\n\n" +
            "Default value is 0.0"
        )]
        public double SmoothingLeakCoefficient
        {
            get => radialCore.SmoothingLeakCoefficient;
            set { radialCore.SmoothingLeakCoefficient = value; }
        }

        [Property("Velocity Divisor (Important)"), DefaultPropertyValue(5.0d), ToolTip
        (
            "https://www.desmos.com/calculator/pw0r8hvezt\n\n" +
            "Radius will be multiplied by the cursor's velocity divided by this number up to 1 * the radius value.\n\n" +
            "Unit of measurement is be millimeters per report. A full area CTL-480 user could use anywhere from 5-12 depending on other settings/environment,\n" +
            "so you will likely need to play around with this.\n" +
            "Default value is 5.0"
        )]
        public double VelocityDivisor
        {
            get => radialCore.VelocityDivisor;
            set { radialCore.VelocityDivisor = value; }
        }

        [Property("Minimum Radius Multiplier"), DefaultPropertyValue(0.0d), ToolTip
        (
            "As radius scales by velocity, it might be useful for there to still be some radius even if the velocity is low.\n\n" +
            "Possible value range is 0..1, 0 means the radius will become small as to be effectively 0, 1 means no velocity scaling which I don't recommend.\n\n" +
            "Default value is 0.0"
        )]
        public double MinimumRadiusMultiplier
        {
            get => radialCore.MinimumRadiusMultiplier;
            set { radialCore.MinimumRadiusMultiplier = value; }
        }

        [Property("Radial Mult Power"), DefaultPropertyValue(9.0d), ToolTip
        (
            "Velocity / the velocity divisor returns a radial multiplier, which is raised to this power.\n\n" +
            "Possible value range is 1 and up, 1 means radius will scale linearly with velocity up to 1 * radius, 2 means it will be squared, 3 means it will be cubed, and so on.\n" +
            "Numbers that low are just for explanation, I recommend going higher.\n" + 
            "Default value is 9.0"
        )]
        public double RadialMultPower
        {
            get => radialCore.RadialMultPower;
            set { radialCore.RadialMultPower = value; }
        }

        [Property("Minimum Smoothing Divisor"), DefaultPropertyValue(15.0d), ToolTip
        (
            "As velocity along with an acceleration factor becomes lower than max radius threshold,\n" +
            "initial smoothing coefficient approaches being divided by this number * some constant. It might be slightly more complex than that but you don't have to worry about it.\n\n" +
            "Possible value range is 2 and up.\n\n" +
            "Default value is 15.0"
        )]
        public double MinimumSmoothingDivisor
        {
            get => radialCore.MinimumSmoothingDivisor;
            set { radialCore.MinimumSmoothingDivisor = value; }
        }

        [Property("Raw Accel Threshold"), DefaultPropertyValue(-0.5d), ToolTip
        (
            "If decel (negative value) is sharp enough, then cursor starts to approach snapping to raw report. Velocity divisor adjusts for this.\n" +
            "You can put this above 0 if you feel like it, but be aware that this overrides most other processing.\n" + 
            "Look in the console for the Sharp Decel Lerp value (read the option below) if you want to do that.\n\n" +
            "Default value is -0.5"
        )]
        public double RawAccelThreshold
        {
            get => radialCore.RawAccelThreshold;
            set { radialCore.RawAccelThreshold = value; }
        }

        [BooleanProperty("Console Logging", ""), DefaultPropertyValue(false), ToolTip
        (
            "Each report, info will be printed in console.\n\n" +
            "If the rate of prints exceeds report rate, then that is bad and this filter is not working. Screenshot your area, reset all settings, then re-enable this filter first.\n" +
            "You can use this to make sure that your parameters and thresholds are right.\n\n" +
            "Default value is false"
        )]
        public bool ConsoleLogging
        {
            get => radialCore.ConsoleLogging;
            set { radialCore.ConsoleLogging = value; }
        }

        [Property("Accel Mult Power"), DefaultPropertyValue(9.0d), ToolTip
        (
            "Enable Console Logging above and look at the console. This specific setting affects only radius scaling. but is pretty important.\n\n" +
            "Default value is 9.0"
        )]
        public double AccelMultPower
        {
            get => radialCore.AccelMultPower;
            set { radialCore.AccelMultPower = value; }
        }

        [BooleanProperty("2.0 behavior (below options)", ""), DefaultPropertyValue(true), ToolTip
        (
            "Enables the options below, and some other behavioral changes.\n\n" +
            "Default value is true"
        )]
        public bool Advanced
        {
            get => radialCore.Advanced;
            set { radialCore.Advanced = value; }
        }

        [Property("Raw Velocity Threshold (Important)"), DefaultPropertyValue(5.0d), ToolTip
        (
            "https://www.desmos.com/calculator/pw0r8hvezt\n\n" +
            "Regardless of acceleration, being above this velocity for 2 consecutive reports will override and max out radius.\n" +
            "Only active if 2.0 behavior is enabled.\n\n" +
            "Default value is 5.0"
        )]
        public double rvt
        {
            get => radialCore.rvt;
            set { radialCore.rvt = value; }
        }

        [Property("Angle Index Confidence"), DefaultPropertyValue(1.5d), ToolTip
        (
            "Controls angle index confidence. Higher is weaker. Gets buggy around 1 and below. Usually best to leave this alone.\n" +
            "Only active if 2.0 behavior is enabled.\n\n" +
            "Default value is 1.5"
        )]
        public double aidx
        {
            get => radialCore.aidx;
            set { radialCore.aidx = value; }
        }

        [Property("Angle Index Decel Confidence"), DefaultPropertyValue(3.0d), ToolTip
        (
            "No idea how to describe this well. It's similar to above but acts in the same vein as raw accel threshold. Usually best to leave this alone.\n" +
            "Only active if 2.0 behavior is enabled.\n\n" +
            "Default value is 3.0"
        )]
        public double xlerpconf
        {
            get => radialCore.xlerpconf;
            set { radialCore.xlerpconf = value; }
        }

        [Property("Accel Mult Velocity Override (Important)"), DefaultPropertyValue(5.0d), ToolTip
        (
            "https://www.desmos.com/calculator/pw0r8hvezt\n\n" +
            "Velocity divisor plays a role in accel mult calculation. This is a manual override. Nothing changes if this is the same as the velocity divisor\n" +
            "Only active if 2.0 behavior is enabled.\n\n" +
            "Default value is 5.0"
        )]
        public double accelMultVelocityOverride
        {
            get => radialCore.accelMultVelocityOverride;
            set { radialCore.accelMultVelocityOverride = value; }
        }

        [Property("Spin Check Confidence"), DefaultPropertyValue(0.75d), ToolTip
        (
            "Checks if raw velocity has been above ((this number) * raw velocity threshold) enough with no snaps.\n" +
            "Only active if 2.0 behavior is enabled.\n\n" +
            "Default value is 0.75"
        )]
        public double spinCheckConfidence
        {
            get => radialCore.spinCheckConfidence;
            set { radialCore.spinCheckConfidence = value; }
        }

        [BooleanProperty("Grounded Radius (Default Behavior)", ""), DefaultPropertyValue(true), ToolTip
        (
            "Radius behavior where the position of the first radius max in a series of consecutive radius max reports dictates the center of the radius.\n" +
            "Only active if 2.0 behavior is enabled.\n\n" +
            "Default value is true"
        )]
        public bool groundedBehavior
        {
            get => radialCore.groundedBehavior;
            set { radialCore.groundedBehavior = value; }
        }

        public event Action<IDeviceReport> Emit;

        public void Consume(IDeviceReport value)
        {
            if (value is ITabletReport report)
            {
                report.Position = radialCore.Filter(value, report.Position * mmScale) / mmScale;
                value = report;
            }
            Emit?.Invoke(value);
        }

        AdaptiveRadialFollowCore radialCore = new AdaptiveRadialFollowCore();

        [TabletReference]
        public TabletReference TabletReference
        {
            set
            {
                var digitizer = value.Properties.Specifications.Digitizer;
                mmScale = new Vector2
                {
                    X = digitizer.Width / digitizer.MaxX,
                    Y = digitizer.Height / digitizer.MaxY
                };
            }
        }
        private Vector2 mmScale = Vector2.One;
    }
}
