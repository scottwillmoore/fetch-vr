//Do not edit! This file was generated by Unity-ROS MessageGeneration.
using System;
using System.Linq;
using System.Collections.Generic;
using System.Text;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;

namespace RosMessageTypes.Moveit
{
    [Serializable]
    public class ApplyPlanningSceneResponse : Message
    {
        public const string k_RosMessageName = "moveit_msgs/ApplyPlanningScene";
        public override string RosMessageName => k_RosMessageName;

        public bool success;

        public ApplyPlanningSceneResponse()
        {
            this.success = false;
        }

        public ApplyPlanningSceneResponse(bool success)
        {
            this.success = success;
        }

        public static ApplyPlanningSceneResponse Deserialize(MessageDeserializer deserializer) => new ApplyPlanningSceneResponse(deserializer);

        private ApplyPlanningSceneResponse(MessageDeserializer deserializer)
        {
            deserializer.Read(out this.success);
        }

        public override void SerializeTo(MessageSerializer serializer)
        {
            serializer.Write(this.success);
        }

        public override string ToString()
        {
            return "ApplyPlanningSceneResponse: " +
            "\nsuccess: " + success.ToString();
        }

#if UNITY_EDITOR
        [UnityEditor.InitializeOnLoadMethod]
#else
        [UnityEngine.RuntimeInitializeOnLoadMethod]
#endif
        public static void Register()
        {
            MessageRegistry.Register(k_RosMessageName, Deserialize, MessageSubtopic.Response);
        }
    }
}
