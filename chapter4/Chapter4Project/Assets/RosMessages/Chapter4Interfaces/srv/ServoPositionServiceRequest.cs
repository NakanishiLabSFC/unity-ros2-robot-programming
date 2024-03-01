//Do not edit! This file was generated by Unity-ROS MessageGeneration.
using System;
using System.Linq;
using System.Collections.Generic;
using System.Text;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;

namespace RosMessageTypes.Chapter4Interfaces
{
    [Serializable]
    public class ServoPositionServiceRequest : Message
    {
        public const string k_RosMessageName = "chapter4_interfaces/ServoPositionService";
        public override string RosMessageName => k_RosMessageName;

        public int position;

        public ServoPositionServiceRequest()
        {
            this.position = 0;
        }

        public ServoPositionServiceRequest(int position)
        {
            this.position = position;
        }

        public static ServoPositionServiceRequest Deserialize(MessageDeserializer deserializer) => new ServoPositionServiceRequest(deserializer);

        private ServoPositionServiceRequest(MessageDeserializer deserializer)
        {
            deserializer.Read(out this.position);
        }

        public override void SerializeTo(MessageSerializer serializer)
        {
            serializer.Write(this.position);
        }

        public override string ToString()
        {
            return "ServoPositionServiceRequest: " +
            "\nposition: " + position.ToString();
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
