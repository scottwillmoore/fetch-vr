using RosMessageTypes.BuiltinInterfaces;

public static class ROSExtensions
{
    private static readonly double secondsPerNanosecond = 1e-9;

    public static double ToDouble(this DurationMsg durationMessage)
    {
        var duration = (double)durationMessage.sec;
        duration += secondsPerNanosecond * (double)durationMessage.nanosec;
        return duration;
    }
}

