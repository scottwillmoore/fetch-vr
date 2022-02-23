# Unity

## VR

**_Everything you wanted to know about XR in Unity but were afraid to ask..._**

To develop in Unity with an Oculus Quest 2 I would read both the [Oculus documentation](https://developer.oculus.com/documentation/unity/) and the [Unity documentation](https://docs.unity3d.com/Manual/VROverview.html) for VR development. As the landscape of VR development has evolved drastically over the last several years, it is important to check the dates of articles on the internet and double check that any advice provided is still applicable.

### SDKs

It is important to understand that every XR platform orginally provided their own SDK to interact with their VR devices. For example, Oculus provide both an [Oculus Mobile SDK](https://developer.oculus.com/documentation/native/android/mobile-intro/) and [Oculus PC SDK](https://developer.oculus.com/documentation/native/pc/pcsdk-intro/), Steam provide the [OpenVR SDK](https://github.com/ValveSoftware/openvr). However, as XR hardware matured the [OpenXR](https://www.khronos.org/openxr/) standard emerged, which provides a unified abstraction to interact with any XR devices. It has become an industry standard and is supported by most XR devices. Despite the adoption of the OpenXR standard, XR platforms do develop unique features that are sometimes not initially supported through the OpenXR APIs.

### Unity XR Plug-in Framework

As far as I am aware, most likely prior to the widespread adoption of OpenXR, Unity developed their own XR abstraction known as the [Unity XR Plug-in Framework](https://docs.unity3d.com/Manual/XRPluginArchitecture.html). This abstraction is now the most popular way to interface with XR devices from within Unity. In addition to many XR platforms, it also supports OpenXR and therefore will also support any XR platforms that support the OpenXR standard.

### Unity XR Interaction Toolkit

The benefit of the XR support built-in to Unity is that there are many other projects that extend it with additional features. For example, Unity provide an additonal package named the [XR Interaction Toolkit](https://docs.unity3d.com/Packages/com.unity.xr.interaction.toolkit@2.0/manual/index.html) which adds many new features. An import feature is the ability to interact with Unity UI systems.

Oculus also provide the [Oculus Integration](https://developer.oculus.com/documentation/unity/unity-import/) package for Unity. This is the method discussed and recommended in the Oculus documentation. It does provide features that are not available with the Unity XR Plug-in Framework and provides the neccessary tools to publish your app to the Unity store. However, the package imports directly into your project, rather than as a Unity package and is over 1 GB. For most use cases it is not needed and I would not recommend it. You may want to wish to extract components from the package such as the Oculus Touch controller meshes.

### Mixed Reality Toolkit

Microsoft develop the [Mixed Reality Toolkit](https://docs.microsoft.com/en-us/windows/mixed-reality/mrtk-unity/?view=mrtkunity-2021-05) which is a Unity package that in a similar manner to the XR Interaction Toolkit. I believe future versions of the Mixed Reality Toolkit aim to improve integration with the XR Interaction Toolkit. I personally have never used the Mixed Reality Toolkit, however it appears to provide a large amount of high quality features for developers to use.

In summary, just use the Unity XR Plug-in Framework that is now built-in to Unity. You may wish to add in additional packages such as the Oculus Integration, XR Interaction Toolkit or the Mixed Reality Tookit for their additional features. I would avoid using any of the Oculus specific components that are provided by the Oculus Integration packages.

## ROS

**_Everything you wanted to know about ROS integration in Unity but were afraid to ask..._**

There are several methods to integrate ROS into a Unity project. However, each method comes with a set of pros and cons which should be evaluated.

### Rosbridge

An easy way to integrate ROS with any frontend is to use [Rosbridge](http://wiki.ros.org/rosbridge_suite) which provides a JSON-based protocol to communicate with ROS. Most popular programming languages have a library that implements the Rosbridge protocol. A problem with all Rosbridge-based applications is performance, whereby the cost to serialize and deserialize from JSON can be inefficient. This is not a problem for simple messages, however for larger messages such as images and point clouds it can become an issue.

### ROS#

A popular project for C# is [ROS#](https://github.com/siemens/ros-sharp) which provides a set of libraries for interacting with ROS. It uses Rosbridge to communicate with ROS nodes, and therefore suffers from the limitations of Rosbridge. In addition, it provides utilities for the generation of C# classes for ROS messages and the ability to import a URDFs into your Unity scenes.

### Unity Robotics

A newer project, which is developed by Unity is the [Unity Robotics Hub](https://github.com/Unity-Technologies/Unity-Robotics-Hub) repository. They provide their own TCP-based communication protocol which is much more efficient that the Rosbridge protocol. They also provide a URDF importer that integrates with Unity [articulation bodies](https://docs.unity3d.com/Manual/class-ArticulationBody.html) and a visualizer that allows many common ROS message types to be viewed from within Unity.

I would recommend the official Unity packages for a new project, however it is not without its drawbacks...

First, the use of articulation bodies appears to be primarly targetted for physics simulation within Unity. It is not well documented on how to use articulation bodies without physics, for visualisation purposes. The entire articulation body API is in my opinion awkward to use, and if you disable physics it no longer works. Instead, the best way is to move your imported URDF to a Unity layer that has all collisions disabled, and set the joint friction to positive infinity.

Second, the way ROS messages have been implemented underneath is not ideal. However, in most cases there is no need to interact or extend these low level primitives. Except, when you wish to use ROS actions, which are not supported yet. In this case, we had to implement our own action client which can be found in this repository.

### Iviz

An interesting project I found was [Iviz](https://github.com/KIT-ISAS/iviz), an attempt to create an Rviz-like application in AR. The interesting component of Iviz is that they implemented their ROS communication from scratch, by implementing the actual ROS protocol used by ROS nodes. This means that no additional packages or translation layers are required to be installed in ROS, the application can simply interact as another ROS node on the network. This may be useful for a truely standalone application.
