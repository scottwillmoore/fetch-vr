using RosMessageTypes.Geometry;
using RosMessageTypes.Moveit;
using RosMessageTypes.Shape;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using Unity.Robotics.ROSTCPConnector;
using UnityEngine.InputSystem;
using UnityEngine.UI;
using UnityEngine;

public class MoveGroupController : MonoBehaviour
{
    private enum State
    {
        HasNoPlan,
        WaitingForPlan,
        HasFailedPlan,
        HasSuccessfulPlan,
        ExecutingPlan,
    }

    private enum GripperState
    {
        Open,
        Closed,
    }

    private ROSConnection rosConnection;
    private ROSTime rosTime;

    private TFSystem tfSystem;

    [Header("Actions")]

    [SerializeField] private InputAction planTrajectoryAction;
    [SerializeField] private InputAction executeTrajectoryAction;
    [SerializeField] private InputAction toggleGripperAction;

    [Header("Feedbacks")]

    [SerializeField] private TintMaterial gripperIndicator;
    [SerializeField] private Text feedbackText;

    [Header("Settings")]

    [SerializeField] private string frameId;

    [SerializeField] private string planTrajectoryActionName = "/move_group";

    private ROSActionClient<MoveGroupGoal, MoveGroupResult, MoveGroupFeedback> planTrajectoryActionClient;

    [SerializeField] private string executeTrajectoryActionName = "/execute_trajectory";

    private ROSActionClient<ExecuteTrajectoryGoal, ExecuteTrajectoryResult, ExecuteTrajectoryFeedback> executeTrajectoryActionClient;

    [SerializeField] private string armGroupName;
    [SerializeField] private string gripperGroupName;

    [SerializeField] private string targetLinkName;

    [SerializeField] private int planningAttempts = 1;
    [SerializeField] private double allowedPlanningTime = 5.0;

    [SerializeField] private double maxVelocityScalingFactor = 1.0;
    [SerializeField] private double maxAccelerationScalingFactor = 1.0;

    [SerializeField] private double positionTolerance = 1e-3;
    [SerializeField] private double orientationTolerance = 1e-3;
    [SerializeField] private double jointTolerance = 1e-3;

    private State state;
    private RobotTrajectoryMsg latestTrajectory;

    public void Start()
    {
        rosConnection = ROSConnection.GetOrCreateInstance();
        rosTime = ROSTime.GetOrCreateInstance();

        tfSystem = TFSystem.GetOrCreateInstance();

        planTrajectoryActionClient = new ROSActionClient<MoveGroupGoal, MoveGroupResult, MoveGroupFeedback>(planTrajectoryActionName);
        executeTrajectoryActionClient = new ROSActionClient<ExecuteTrajectoryGoal, ExecuteTrajectoryResult, ExecuteTrajectoryFeedback>(executeTrajectoryActionName);

        planTrajectoryAction.Enable();
        executeTrajectoryAction.Enable();
        toggleGripperAction.Enable();

        planTrajectoryAction.performed += (inputAction) => PlanTrajectory();
        executeTrajectoryAction.performed += (inputAction) => ExecuteTrajectory();
        toggleGripperAction.performed += (inputAction) => ToggleGripper();

        gripperIndicator.gameObject.SetActive(false);
    }

    private void PlanTrajectory()
    {
        var errorPrefix = "Cannot plan trajectory: ";

        switch (state)
        {
            case State.HasNoPlan:
            case State.HasFailedPlan:
            case State.HasSuccessfulPlan:
                state = State.WaitingForPlan;

                feedbackText.text = "Attempting to plan trajectory...";

                gripperIndicator.gameObject.SetActive(true);
                gripperIndicator.transform.position = gameObject.transform.position;
                gripperIndicator.transform.rotation = gameObject.transform.rotation;
                var tintColor = Color.blue;
                tintColor.a = 0.5f;
                gripperIndicator.SetTintColor(tintColor);

                var pose = GetCurrentPose();
                var goal = GetPlanGoal(pose);
                planTrajectoryActionClient.Send(goal, (resultState, result) =>
                {
                    switch (resultState)
                    {
                        case ResultState.Aborted:
                        case ResultState.Lost:
                        case ResultState.Preempted:
                        case ResultState.Recalled:
                        case ResultState.Rejected:
                            state = State.HasFailedPlan;

                            feedbackText.text = "Plan has failed!";

                            tintColor = Color.red;
                            tintColor.a = 0.5f;
                            gripperIndicator.SetTintColor(tintColor);

                            break;

                        case ResultState.Succeeded:
                            state = State.HasSuccessfulPlan;
                            latestTrajectory = result.planned_trajectory;

                            feedbackText.text = "Plan has succeeded!";

                            tintColor = Color.green;
                            tintColor.a = 0.5f;
                            gripperIndicator.SetTintColor(tintColor);

                            break;
                    }
                });
                break;

            case State.WaitingForPlan:
                feedbackText.text = errorPrefix + "Waiting for plan to be confirmed.";
                break;

            case State.ExecutingPlan:
                feedbackText.text = errorPrefix + "Execution in progress.";
                break;
        }
    }

    private void ExecuteTrajectory()
    {
        var errorPrefix = "Cannot execute trajectory: ";

        switch (state)
        {
            case State.HasNoPlan:
                feedbackText.text = errorPrefix + "There is no plan.";
                break;

            case State.WaitingForPlan:
                feedbackText.text = errorPrefix + "Waiting for the plan to be confirmed.";
                break;

            case State.HasFailedPlan:
                feedbackText.text = errorPrefix + "The plan cannot be achieved.";
                break;

            case State.HasSuccessfulPlan:
                state = State.ExecutingPlan;

                feedbackText.text = "Attempting to execute trajectory...";

                gripperIndicator.gameObject.SetActive(false);

                var goal = new ExecuteTrajectoryGoal();
                goal.trajectory = latestTrajectory;
                executeTrajectoryActionClient.Send(goal, (resultState, result) =>
                {
                    switch (resultState)
                    {
                        case ResultState.Aborted:
                        case ResultState.Lost:
                        case ResultState.Preempted:
                        case ResultState.Recalled:
                        case ResultState.Rejected:
                            state = State.HasNoPlan;
                            feedbackText.text = "Attempting to execute trajectory...";
                            break;

                        case ResultState.Succeeded:
                            state = State.HasNoPlan;
                            feedbackText.text = "Execution has succeeded!";
                            break;
                    }
                });
                break;

            case State.ExecutingPlan:
                feedbackText.text = errorPrefix + "Execution in progress.";
                break;
        }
    }

    private void ToggleGripper()
    {
        Debug.Log("Toggle Gripper");
        var errorPrefix = "Cannot plan trajectory: ";

        switch (state)
        {
            case State.HasNoPlan:
            case State.HasFailedPlan:
            case State.HasSuccessfulPlan:
                state = State.WaitingForPlan;

                var fingersState = GetGripperState();
                var goal = GetGripperPlanGoal(fingersState);

                planTrajectoryActionClient.Send(goal, (resultState, result) =>
                {
                    switch (resultState)
                    {
                        case ResultState.Aborted:
                        case ResultState.Lost:
                        case ResultState.Preempted:
                        case ResultState.Recalled:
                        case ResultState.Rejected:
                            state = State.HasFailedPlan;
                            Debug.LogError("Has failed plan!");
                            break;

                        case ResultState.Succeeded:
                            state = State.HasSuccessfulPlan;
                            Debug.Log("Has successful plan!");
                            latestTrajectory = result.planned_trajectory;
                            ExecuteTrajectory();
                            break;
                    }
                });
                break;

            case State.WaitingForPlan:
                Debug.LogError(errorPrefix + "Waiting for the plan to be confirmed.");
                break;

            case State.ExecutingPlan:
                Debug.LogError(errorPrefix + "Execution in progress.");
                break;
        }
    }

    private PoseStampedMsg GetCurrentPose()
    {
        var referenceObject = tfSystem.GetTransformObject(frameId);

        var relativePosition = referenceObject.transform.InverseTransformPoint(gameObject.transform.position);
        var relativeRotation = referenceObject.transform.InverseTransformRotation(gameObject.transform.rotation);

        var currentPose = new PoseStampedMsg();
        currentPose.header.stamp = rosTime.Now();
        currentPose.header.frame_id = frameId;

        currentPose.pose.position = relativePosition.To<FLU>();
        currentPose.pose.orientation = relativeRotation.To<FLU>();

        return currentPose;
    }

    private bool GetGripperState()
    {
        var gripperLink = gameObject.transform.Find("gripper_link").gameObject;
        var leftFinger = gripperLink.transform.Find("l_gripper_finger_link").gameObject;
        var fingersPosition = leftFinger.GetComponent<ArticulationBody>().jointPosition[0];
        var fingersOpen = fingersPosition > 0.048f;

        return fingersOpen;
    }

    private MoveGroupGoal GetPlanGoal(PoseStampedMsg pose)
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
        positionConstraint.link_name = targetLinkName;
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
        orientationConstraint.link_name = targetLinkName;
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
        request.group_name = armGroupName;
        request.num_planning_attempts = planningAttempts;
        request.allowed_planning_time = allowedPlanningTime;
        request.max_velocity_scaling_factor = maxVelocityScalingFactor;
        request.max_acceleration_scaling_factor = maxAccelerationScalingFactor;
        // request.cartesian_speed_end_effector_link = ...;
        // request.max_cartesian_speed = ...;

        // http://docs.ros.org/en/api/moveit_msgs/html/msg/PlanningOptions.html
        var planningOptions = new PlanningOptionsMsg();
        planningOptions.planning_scene_diff.is_diff = true;
        planningOptions.planning_scene_diff.robot_state.is_diff = true;
        planningOptions.plan_only = true;
        planningOptions.look_around = false;
        // planning_options.look_around_attempts = ...;
        // planning_options.max_safe_execution_cost = ...;
        planningOptions.replan = false;
        // planning_options.replan_attempts = ...;
        // planning_options.replan_delay = ...;

        var moveGroupGoal = new MoveGroupGoal();
        moveGroupGoal.request = request;
        moveGroupGoal.planning_options = planningOptions;

        return moveGroupGoal;
    }

    private void SetFeedback(string feedback)
    {
        feedbackText.text = feedback;
    }

    private MoveGroupGoal GetGripperPlanGoal(bool fingersOpen)
    {
        var jointGoal = 0f;
        if (!fingersOpen)
        {
            jointGoal = 0.5f;
            Debug.Log("Opening gripper");
        }
        else
        {
            Debug.Log("Closing gripper");
        }

        // http://docs.ros.org/en/api/moveit_msgs/html/msg/WorkspaceParameters.html
        var workspaceParameters = new WorkspaceParametersMsg();
        workspaceParameters.header.frame_id = frameId;
        workspaceParameters.header.stamp = rosTime.Now();
        // TODO: Allow the workspace to be specified
        workspaceParameters.min_corner.x = -1.0;
        workspaceParameters.min_corner.y = -0.0;
        workspaceParameters.min_corner.z = -1.0;
        workspaceParameters.max_corner.x = +1.0;
        workspaceParameters.max_corner.y = +2.0;
        workspaceParameters.max_corner.z = +1.0;

        // http://docs.ros.org/en/api/moveit_msgs/html/msg/JointConstraint.html
        var leftJointConstraint = new JointConstraintMsg();
        leftJointConstraint.joint_name = "l_gripper_finger_joint";
        leftJointConstraint.position = jointGoal;
        leftJointConstraint.tolerance_above = jointTolerance;
        leftJointConstraint.tolerance_below = jointTolerance;
        leftJointConstraint.weight = 1.0f;
        var rightJointConstraint = new JointConstraintMsg();
        rightJointConstraint.joint_name = "r_gripper_finger_joint";
        rightJointConstraint.position = jointGoal;
        rightJointConstraint.tolerance_above = jointTolerance;
        rightJointConstraint.tolerance_below = jointTolerance;
        rightJointConstraint.weight = 1.0f;

        // http://docs.ros.org/en/api/moveit_msgs/html/msg/Constraints.html
        var goalConstraint = new ConstraintsMsg();
        // goal_constraint.name = ...;
        // goalConstraint.position_constraints = new PositionConstraintMsg[1];
        // goalConstraint.position_constraints[0] = positionConstraint;
        // goalConstraint.orientation_constraints = new OrientationConstraintMsg[1];
        // goalConstraint.orientation_constraints[0] = orientationConstraint;
        goalConstraint.joint_constraints = new JointConstraintMsg[2];
        goalConstraint.joint_constraints[0] = leftJointConstraint;
        goalConstraint.joint_constraints[1] = rightJointConstraint;

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
        request.group_name = gripperGroupName;
        request.num_planning_attempts = planningAttempts;
        request.allowed_planning_time = allowedPlanningTime;
        request.max_velocity_scaling_factor = maxVelocityScalingFactor;
        request.max_acceleration_scaling_factor = maxAccelerationScalingFactor;
        // request.cartesian_speed_end_effector_link = ...;
        // request.max_cartesian_speed = ...;

        // http://docs.ros.org/en/api/moveit_msgs/html/msg/PlanningOptions.html
        var planningOptions = new PlanningOptionsMsg();
        planningOptions.planning_scene_diff.is_diff = true;
        planningOptions.planning_scene_diff.robot_state.is_diff = true;
        planningOptions.plan_only = true;
        planningOptions.look_around = false;
        // planning_options.look_around_attempts = ...;
        // planning_options.max_safe_execution_cost = ...;
        planningOptions.replan = false;
        // planning_options.replan_attempts = ...;
        // planning_options.replan_delay = ...;

        var moveGroupGoal = new MoveGroupGoal();
        moveGroupGoal.request = request;
        moveGroupGoal.planning_options = planningOptions;

        return moveGroupGoal;
    }
}
