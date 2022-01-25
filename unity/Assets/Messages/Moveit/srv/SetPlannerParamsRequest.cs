//Do not edit! This file was generated by Unity-ROS MessageGeneration.
using System;
using System.Linq;
using System.Collections.Generic;
using System.Text;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;

namespace RosMessageTypes.Moveit
{
    [Serializable]
    public class SetPlannerParamsRequest : Message
    {
        public const string k_RosMessageName = "moveit_msgs/SetPlannerParams";
        public override string RosMessageName => k_RosMessageName;

        //  Name of planning config
        public string planner_config;
        //  Optional name of planning group (set global defaults if empty)
        public string group;
        //  parameters as key-value pairs
        public PlannerParamsMsg @params;
        //  replace params or augment existing ones?
        public bool replace;

        public SetPlannerParamsRequest()
        {
            this.planner_config = "";
            this.group = "";
            this.@params = new PlannerParamsMsg();
            this.replace = false;
        }

        public SetPlannerParamsRequest(string planner_config, string group, PlannerParamsMsg @params, bool replace)
        {
            this.planner_config = planner_config;
            this.group = group;
            this.@params = @params;
            this.replace = replace;
        }

        public static SetPlannerParamsRequest Deserialize(MessageDeserializer deserializer) => new SetPlannerParamsRequest(deserializer);

        private SetPlannerParamsRequest(MessageDeserializer deserializer)
        {
            deserializer.Read(out this.planner_config);
            deserializer.Read(out this.group);
            this.@params = PlannerParamsMsg.Deserialize(deserializer);
            deserializer.Read(out this.replace);
        }

        public override void SerializeTo(MessageSerializer serializer)
        {
            serializer.Write(this.planner_config);
            serializer.Write(this.group);
            serializer.Write(this.@params);
            serializer.Write(this.replace);
        }

        public override string ToString()
        {
            return "SetPlannerParamsRequest: " +
            "\nplanner_config: " + planner_config.ToString() +
            "\ngroup: " + group.ToString() +
            "\n@params: " + @params.ToString() +
            "\nreplace: " + replace.ToString();
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
