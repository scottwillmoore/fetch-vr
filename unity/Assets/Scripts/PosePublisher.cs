using RosMessageTypes.Geometry;
using RosMessageTypes.Moveit;
using RosMessageTypes.Shape;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using Unity.Robotics.ROSTCPConnector;
using UnityEngine;

public class PosePublisher : MonoBehaviour
{
    private ROSConnection rosConnection;
    private ROSTime rosTime;

    [SerializeField] private string frameId;
    [SerializeField] private GameObject frameReference;

    [Header("Pose Topic Publisher")]

    [SerializeField] private string poseTopicName;
    [SerializeField] private float poseTopicFrequency = 10.0f;

    private float poseTopicPeriod;

    private float timeElapsed;

    [Header("Motion Plan Service")]

    [SerializeField] private string motionPlanServiceName;

    [Space(8)]
    [SerializeField] private OVRInput.RawButton motionPlanButton;

    [Space(8)]
    [SerializeField] private GameObject motionPlanMarker;

    [Space(8)]
    [SerializeField] private string groupName;
    [SerializeField] private string linkName;

    [Space(8)]
    [SerializeField] private int planningAttempts = 1;
    [SerializeField] private double allowedPlanningTime = 5.0;

    [Space(8)]
    [SerializeField] private double maxVelocityScalingFactor = 1.0;
    [SerializeField] private double maxAccelerationScalingFactor = 1.0;

    [Space(8)]
    [SerializeField] private double positionTolerance = 1e-3;
    [SerializeField] private double orientationTolerance = 1e-3;

    private bool waitForMotionPlan;
    private bool motionPlanSuccess;

    private RobotTrajectoryMsg latestRobotTrajectory;

    [Header("Execute Trajectory Action")]

    [SerializeField] private string executeTrajectoryActionName;

    [Space(8)]
    [SerializeField] private OVRInput.RawButton executeTrajectoryButton;

    private SimpleActionClient<
            ExecuteTrajectoryAction,
            ExecuteTrajectoryActionGoal,
            ExecuteTrajectoryActionResult,
            ExecuteTrajectoryActionFeedback,
            ExecuteTrajectoryGoal,
            ExecuteTrajectoryResult,
            ExecuteTrajectoryFeedback
        > executeTrajectoryClient;

    private bool waitForExecuteTrajectory;

#if UNITY_EDITOR
    [UnityEditor.InitializeOnLoadMethod]
#else
    [UnityEngine.RuntimeInitializeOnLoadMethod]
#endif
    public static void Register()
    {
        MessageRegistry.Register(ExecuteTrajectoryAction.k_RosMessageName, ExecuteTrajectoryAction.Deserialize);
        MessageRegistry.Register(ExecuteTrajectoryActionGoal.k_RosMessageName, ExecuteTrajectoryActionGoal.Deserialize);
        MessageRegistry.Register(ExecuteTrajectoryActionResult.k_RosMessageName, ExecuteTrajectoryActionResult.Deserialize);
        MessageRegistry.Register(ExecuteTrajectoryActionFeedback.k_RosMessageName, ExecuteTrajectoryActionFeedback.Deserialize);
    }

    public void Start()
    {
        rosConnection = ROSConnection.GetOrCreateInstance();
        rosTime = ROSTime.GetOrCreateInstance();

        rosConnection.RegisterPublisher<PoseStampedMsg>(poseTopicName);
        poseTopicPeriod = 1.0f / poseTopicFrequency;

        rosConnection.RegisterRosService<GetMotionPlanRequest, GetMotionPlanResponse>(motionPlanServiceName);

        executeTrajectoryClient = new SimpleActionClient<
                ExecuteTrajectoryAction,
                ExecuteTrajectoryActionGoal,
                ExecuteTrajectoryActionResult,
                ExecuteTrajectoryActionFeedback,
                ExecuteTrajectoryGoal,
                ExecuteTrajectoryResult,
                ExecuteTrajectoryFeedback
            >(executeTrajectoryActionName);
    }

    public void Update()
    {
        var latestTime = rosTime.Now();

        var relativePosition = frameReference.transform.InverseTransformPoint(gameObject.transform.position);
        var relativeRotation = Quaternion.Inverse(gameObject.transform.rotation) * frameReference.transform.rotation;

        var pose = new PoseStampedMsg();
        pose.header.frame_id = frameId;
        pose.header.stamp = latestTime;
        pose.pose.orientation = relativeRotation.To<FLU>();
        pose.pose.position = relativePosition.To<FLU>();

        timeElapsed += Time.deltaTime;
        if (timeElapsed > poseTopicPeriod)
        {
            rosConnection.Publish(poseTopicName, pose);
            timeElapsed = 0;
        }

        var meshRenderer = motionPlanMarker.GetComponent<MeshRenderer>();
        var transform = motionPlanMarker.GetComponent<Transform>();

        if (OVRInput.GetDown(motionPlanButton))
        {
            if (waitForMotionPlan)
            {
                // TODO: Feedback: A motion plan is in progress
            }
            else
            {
                meshRenderer.material.SetColor("_Color", Color.blue);
                transform.position = relativePosition;
                transform.rotation = relativeRotation;

                var request = new GetMotionPlanRequest();
                request.motion_plan_request = CreateMotionPlanRequest(pose);

                waitForMotionPlan = true;
                motionPlanSuccess = false;

                rosConnection.SendServiceMessage<GetMotionPlanResponse>(motionPlanServiceName, request, MotionPlanCallback);

                Debug.Log(request.ToString());
            }
        }

        if (OVRInput.GetDown(executeTrajectoryButton))
        {
            if (motionPlanSuccess)
            {
                if (waitForExecuteTrajectory)
                {
                    // TODO: Feedback: An executed trajectory is in progress
                }
                else
                {
                    waitForExecuteTrajectory = true;

                    var request = new ExecuteKnownTrajectoryRequest();

                    var goal = new ExecuteTrajectoryGoal();
                    goal.trajectory = latestRobotTrajectory;

                    executeTrajectoryClient.Send(goal, ExecuteTrajectoryCallback);

                    Debug.Log(request.ToString());
                }
            }
            else
            {
                // TODO: Feedback: There is no successful plan to execute
            }
        }
    }

    private void MotionPlanCallback(GetMotionPlanResponse response)
    {
        Debug.Log(response.ToString());

        waitForMotionPlan = false;

        var errorCode = response.motion_plan_response.error_code.val;
        motionPlanSuccess = errorCode == MoveItErrorCodesMsg.SUCCESS;

        var meshRenderer = motionPlanMarker.GetComponent<MeshRenderer>();
        var transform = motionPlanMarker.GetComponent<Transform>();

        if (motionPlanSuccess)
        {
            latestRobotTrajectory = response.motion_plan_response.trajectory;

            meshRenderer.material.SetColor("_Color", Color.green);

            // TODO: Update transform to match the response destination
            // transform.position = ...;
            // transform.rotation = ...;
        }
        else
        {
            meshRenderer.material.SetColor("_Color", Color.red);
        }
    }

    private void ExecuteTrajectoryCallback(ExecuteTrajectoryResult result)
    {
        Debug.Log("Result: Error Code: " + result.error_code);

        waitForExecuteTrajectory = false;
    }

    private MotionPlanRequestMsg CreateMotionPlanRequest(PoseStampedMsg pose)
    {
        // http://docs.ros.org/en/api/moveit_msgs/html/msg/WorkspaceParameters.html
        var workspaceParameters = new WorkspaceParametersMsg();
        workspaceParameters.header.frame_id = frameId;
        workspaceParameters.header.stamp = pose.header.stamp;
        // TODO: Allow the workspace to be specified
        workspaceParameters.min_corner.x = -1.0;
        workspaceParameters.min_corner.y = -0.0;
        workspaceParameters.min_corner.z = -1.0;
        workspaceParameters.max_corner.x = +1.0;
        workspaceParameters.max_corner.y = +2.0;
        workspaceParameters.max_corner.z = +1.0;

        // http://docs.ros.org/en/api/moveit_msgs/html/msg/PositionConstraint.html
        var positionConstraint = new PositionConstraintMsg();
        positionConstraint.header = pose.header;
        positionConstraint.link_name = linkName;
        // constraint.target_point_offset = ...;
        positionConstraint.constraint_region.primitives = new SolidPrimitiveMsg[1];
        positionConstraint.constraint_region.primitives[0] = new SolidPrimitiveMsg();
        positionConstraint.constraint_region.primitives[0].type = SolidPrimitiveMsg.SPHERE;
        positionConstraint.constraint_region.primitives[0].dimensions = new double[1];
        positionConstraint.constraint_region.primitives[0].dimensions[SolidPrimitiveMsg.SPHERE_RADIUS] = positionTolerance;
        positionConstraint.constraint_region.primitive_poses = new PoseMsg[1];
        positionConstraint.constraint_region.primitive_poses[0] = new PoseMsg();
        positionConstraint.constraint_region.primitive_poses[0].position = pose.pose.position;
        // position_constraint.constraint_region.primitive_poses[0].orientation = ...;
        positionConstraint.weight = 1.0f;

        // http://docs.ros.org/en/api/moveit_msgs/html/msg/OrientationConstraint.html
        var orientationConstraint = new OrientationConstraintMsg();
        orientationConstraint.header = pose.header;
        orientationConstraint.link_name = linkName;
        orientationConstraint.orientation = pose.pose.orientation;
        orientationConstraint.absolute_x_axis_tolerance = orientationTolerance;
        orientationConstraint.absolute_y_axis_tolerance = orientationTolerance;
        orientationConstraint.absolute_z_axis_tolerance = orientationTolerance;
        // orientation_constraint.parameterization = ...; 
        orientationConstraint.weight = 1.0f;


        // http://docs.ros.org/en/api/moveit_msgs/html/msg/Constraints.html
        var goalConstraint = new ConstraintsMsg();
        // goal_constraint.name = ...;
        goalConstraint.position_constraints = new PositionConstraintMsg[1];
        goalConstraint.position_constraints[0] = positionConstraint;
        goalConstraint.orientation_constraints = new OrientationConstraintMsg[1];
        goalConstraint.orientation_constraints[0] = orientationConstraint;

        // http://docs.ros.org/en/api/moveit_msgs/html/msg/MotionPlanRequest.html
        var request = new MotionPlanRequestMsg();
        request.workspace_parameters = workspaceParameters;
        request.start_state.is_diff = true;
        request.goal_constraints = new ConstraintsMsg[1];
        request.goal_constraints[0] = goalConstraint;
        // request.path_constraints = ...;
        // request.trajectory_constraints = ...;
        // request.reference_trajectories = ...;
        // request.pipeline_id = ...;
        // request.planner_id = ...;
        request.group_name = groupName;
        request.num_planning_attempts = planningAttempts;
        request.allowed_planning_time = allowedPlanningTime;
        request.max_velocity_scaling_factor = maxVelocityScalingFactor;
        request.max_acceleration_scaling_factor = maxAccelerationScalingFactor;
        // request.cartesian_speed_end_effector_link = ...;
        // request.max_cartesian_speed = ...;

        /*
        // http://docs.ros.org/en/api/moveit_msgs/html/msg/PlanningOptions.html
        var planning_options = new PlanningOptionsMsg();
        planning_options.planning_scene_diff.is_diff = true;
        planning_options.planning_scene_diff.robot_state.is_diff = true;
        planning_options.plan_only = true;
        planning_options.look_around = false;
        // planning_options.look_around_attempts = ...;
        // planning_options.max_safe_execution_cost = ...;
        planning_options.replan = false;
        // planning_options.replan_attempts = ...;
        // planning_options.replan_delay = ...;

        var move_group_goal = new MoveGroupGoal();
        move_group_goal.request = request;
        move_group_goal.planning_options = planning_options;
        */

        return request;
    }
}
