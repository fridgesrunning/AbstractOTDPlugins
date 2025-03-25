﻿using System;
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

        [Property("Outer Radius"), DefaultPropertyValue(2.0d), Unit("mm"), ToolTip
        (
            "This scales with cursor velocity by default. \n\n" +
            "Outer radius defines the max distance the cursor can lag behind the actual reading.\n\n" +
            "Unit of measurement is millimetres.\n" +
            "The value should be >= 0 and inner radius.\n" +
            "If smoothing leak is used, defines the point at which smoothing will be reduced,\n" +
            "instead of hard clamping the max distance between the tablet position and a cursor.\n\n" +
            "Default value is 1.0 mm"
        )]
        public double OuterRadius
        {
            get => radialCore.OuterRadius;
            set { radialCore.OuterRadius = value; }
        }

        [Property("Inner Radius"), DefaultPropertyValue(1.0d), Unit("mm"), ToolTip
        (
            "This scales with cursor velocity by default. \n\n" +
            "Inner radius defines the max distance the tablet reading can deviate from the cursor without moving it.\n" +
            "This effectively creates a deadzone in which no movement is produced.\n\n" +
            "Unit of measurement is millimetres.\n" +
            "The value should be >= 0 and <= outer radius.\n\n" +
            "Default value is 0.0 mm"
        )]
        public double InnerRadius
        {
            get => radialCore.InnerRadius;
            set { radialCore.InnerRadius = value; }
        }

        [Property("Initial Smoothing Coefficient"), DefaultPropertyValue(0.95d), ToolTip
        (
            "Smoothing coefficient determines how fast or slow the cursor will descend from the outer radius to the inner.\n\n" +
            "Possible value range is 0.0001..1, higher values mean more smoothing (slower descent to the inner radius).\n\n" +
            "Default value is 0.95"
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
            "Smoothing leak coefficient allows for input smooting to continue past outer radius at a reduced rate.\n\n" +
            "Possible value range is 0..1, 0 means no smoothing past outer radius, 1 means 100% of the smoothing gets through.\n\n" +
            "Note that this probably shouldn't be above 0 unless you want the Minimum Smoothing Divisor to be below 2 for more smoothing.\n\n" +
            "Default value is 0.0"
        )]
        public double SmoothingLeakCoefficient
        {
            get => radialCore.SmoothingLeakCoefficient;
            set { radialCore.SmoothingLeakCoefficient = value; }
        }

        [Property("Velocity Divisor"), DefaultPropertyValue(5.0d), ToolTip
        (
            "Radius will be multiplied by the cursor's velocity divided by this number up to 1 * the radius value.\n\n" +
            "Unit of measurement may be millimeters per report rate. A full area CTL-480 user could use 5, so you will likely need a lesser value than this.\n\n" +
            "Default value is 5.0"
        )]
        public double VelocityDivisor
        {
            get => radialCore.VelocityDivisor;
            set { radialCore.VelocityDivisor = value; }
        }

        [Property("Minimum Radius Multiplier"), DefaultPropertyValue(0.1d), ToolTip
        (
            "As radius scales by velocity, it might be useful for there to still be some radius even if the velocity is low.\n\n" +
            "Possible value range is 0..1, 0 means the radius will become small as to be effectively 0, 1 means no velocity scaling which I don't recommend.\n\n" +
            "Default value is 0.1"
        )]
        public double MinimumRadiusMultiplier
        {
            get => radialCore.MinimumRadiusMultiplier;
            set { radialCore.MinimumRadiusMultiplier = value; }
        }

        [Property("Radial Mult Power"), DefaultPropertyValue(2.0d), ToolTip
        (
            "Velocity / the velocity divisor returns a radial multiplier, which is raised to this power.\n\n" +
            "Possible value range is 1 and up, 1 means radius will scale linearly with velocity up to 1 and 2 means it will be squared.\n\n" +
            "2 is generally good for both supporting massive radii for extreme snap and keeping radii low when flowing at the same time.\n\n" +
            "Default value is 2.0"
        )]
        public double RadialMultPower
        {
            get => radialCore.RadialMultPower;
            set { radialCore.RadialMultPower = value; }
        }

        [Property("Minimum Smoothing Divisor"), DefaultPropertyValue(5.0d), ToolTip
        (
            "As velocity along with an acceleration factor becomes lower than max radius threshold,\n" +
            "initial smoothing coefficient approaches being divided by this number * some constant. I checked in console and it's more complex than that but you don't have to worry about it\n\n" +
            "Possible value range is 2 and up.\n\n" +
            "Default value is 5.0"
        )]
        public double MinimumSmoothingDivisor
        {
            get => radialCore.MinimumSmoothingDivisor;
            set { radialCore.MinimumSmoothingDivisor = value; }
        }
        
        [BooleanProperty("Velocity Scales Knee", ""), DefaultPropertyValue(false), ToolTip
        (
            "Only check this if you know what you're doing (if you read the tooltip)\n\n" +
            "Should the soft knee scale be multiplied by 1 + velocity?\n" +
            "If enabled, it's recommended to use a low value for the soft knee scale, like 0.2.\n\n" +
            "Default value is false"
        )]
        public bool VelocityScalesKnee
        {
            get => radialCore.VelocityScalesKnee;
            set { radialCore.VelocityScalesKnee = value; }
        }

        [Property("Raw Accel Threshold"), DefaultPropertyValue(-0.35d), ToolTip
        (
            "I'm really too lazy to do these tooltips seriously.\n" +
            "if sharp enough decel, then cursor starts to approach snapping to raw report\n" +
            "Default value is -0.35\n\n" +
            "Please go into process lasso and set the priority of OTD to realtime. This filter is pretty performance intensive due to shit code."
        )]
        public double RawAccelThreshold
        {
            get => radialCore.RawAccelThreshold;
            set { radialCore.RawAccelThreshold = value; }
        }

        [BooleanProperty("Console Logging", ""), DefaultPropertyValue(false), ToolTip
        (
            "Each report, things will be printed.\n\n" +
            "If the rate of prints exceeds report rate, then that is bad and this filter is not working as well as it could be. read the readme\n" +
            "You can use this to make sure that your parameters and thresholds are right.\n\n" +
            "Default value is false"
        )]
        public bool ConsoleLogging
        {
            get => radialCore.ConsoleLogging;
            set { radialCore.ConsoleLogging = value; }
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
