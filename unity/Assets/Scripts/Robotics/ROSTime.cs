// https://github.com/ros/ros_comm/blob/melodic-devel/clients/rospy/src/rospy/rostime.py

using RosMessageTypes.BuiltinInterfaces;
using RosMessageTypes.Rosgraph;
using System;
using Unity.Robotics.ROSTCPConnector;

public class ROSTime
{
    private static readonly long ticksPerNanosecond = TimeSpan.TicksPerMillisecond / 1000;
    private static readonly long ticksPerSecond = TimeSpan.TicksPerSecond;

    private static readonly DateTime epochTime = new DateTime(1970, 1, 1, 0, 0, 0, DateTimeKind.Utc);

    private static ROSTime instance;

    private ROSConnection rosConnection;

    private TimeMsg latestTime;

    public static ROSTime GetOrCreateInstance()
    {
        if (instance == null)
        {
            instance = new ROSTime();
        }

        return instance;
    }

    private ROSTime()
    {
        rosConnection = ROSConnection.GetOrCreateInstance();
        rosConnection.Subscribe<ClockMsg>("clock", ClockCallback);
    }

    public TimeMsg Now()
    {
        if (this.latestTime == null)
        {
            var currentTime = System.DateTime.UtcNow;
            var elapsedTime = currentTime - epochTime;

            var elapsedSeconds = elapsedTime.Ticks / ticksPerSecond;
            var elapsedNanoseconds = (elapsedTime.Ticks % ticksPerSecond) / ticksPerNanosecond;

            return new TimeMsg((uint)elapsedSeconds, (uint)elapsedNanoseconds);
        }
        else
        {
            return this.latestTime;
        }
    }

    private void ClockCallback(ClockMsg message)
    {
        latestTime = message.clock;
    }
}

