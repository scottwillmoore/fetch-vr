//Do not edit! This file was generated by Unity-ROS MessageGeneration.
using System;
using System.Linq;
using System.Collections.Generic;
using System.Text;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;

namespace RosMessageTypes.Moveit
{
    [Serializable]
    public class MoveItErrorCodesMsg : Message
    {
        public const string k_RosMessageName = "moveit_msgs/MoveItErrorCodes";
        public override string RosMessageName => k_RosMessageName;

        public int val;
        //  overall behavior
        public const int SUCCESS = 1;
        public const int FAILURE = 99999;
        public const int PLANNING_FAILED = -1;
        public const int INVALID_MOTION_PLAN = -2;
        public const int MOTION_PLAN_INVALIDATED_BY_ENVIRONMENT_CHANGE = -3;
        public const int CONTROL_FAILED = -4;
        public const int UNABLE_TO_AQUIRE_SENSOR_DATA = -5;
        public const int TIMED_OUT = -6;
        public const int PREEMPTED = -7;
        //  planning & kinematics request errors
        public const int START_STATE_IN_COLLISION = -10;
        public const int START_STATE_VIOLATES_PATH_CONSTRAINTS = -11;
        public const int GOAL_IN_COLLISION = -12;
        public const int GOAL_VIOLATES_PATH_CONSTRAINTS = -13;
        public const int GOAL_CONSTRAINTS_VIOLATED = -14;
        public const int INVALID_GROUP_NAME = -15;
        public const int INVALID_GOAL_CONSTRAINTS = -16;
        public const int INVALID_ROBOT_STATE = -17;
        public const int INVALID_LINK_NAME = -18;
        public const int INVALID_OBJECT_NAME = -19;
        //  system errors
        public const int FRAME_TRANSFORM_FAILURE = -21;
        public const int COLLISION_CHECKING_UNAVAILABLE = -22;
        public const int ROBOT_STATE_STALE = -23;
        public const int SENSOR_INFO_STALE = -24;
        //  kinematics errors
        public const int NO_IK_SOLUTION = -31;

        public MoveItErrorCodesMsg()
        {
            this.val = 0;
        }

        public MoveItErrorCodesMsg(int val)
        {
            this.val = val;
        }

        public static MoveItErrorCodesMsg Deserialize(MessageDeserializer deserializer) => new MoveItErrorCodesMsg(deserializer);

        private MoveItErrorCodesMsg(MessageDeserializer deserializer)
        {
            deserializer.Read(out this.val);
        }

        public override void SerializeTo(MessageSerializer serializer)
        {
            serializer.Write(this.val);
        }

        public override string ToString()
        {
            return "MoveItErrorCodesMsg: " +
            "\nval: " + val.ToString();
        }

#if UNITY_EDITOR
        [UnityEditor.InitializeOnLoadMethod]
#else
        [UnityEngine.RuntimeInitializeOnLoadMethod]
#endif
        public static void Register()
        {
            MessageRegistry.Register(k_RosMessageName, Deserialize);
        }
    }
}