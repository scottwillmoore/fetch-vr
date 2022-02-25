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

    [SerializeField] private InputAction planTrajectoryAction;
    [SerializeField] private InputAction executeTrajectoryAction;
    [SerializeField] private InputAction toggleGripperAction;

    [Space(8.0f)]

    [SerializeField] private TintMaterial gripperIndicator;
    [SerializeField] private Text feedbackText;

    [Space(8.0f)]

    [SerializeField] private string referenceFrameId;

    [SerializeField] private string planTrajectoryActionName = "/move_group";
    [SerializeField] private string executeTrajectoryActionName = "/execute_trajectory";

    [Space(8.0f)]

    [SerializeField] private string armGroupName;
    [SerializeField] private string targetLinkName;
    [SerializeField] private Transform targetLinkTransform;

    [Space(8.0f)]

    [SerializeField] private bool useVisibilityConstraint;
    [SerializeField] private double visibilityConstraintWeight;
    [SerializeField] private string sensorFrameId;
    [SerializeField] private Transform targetVisibilityTransform;

    [Space(8.0f)]

    [SerializeField] private string gripperGroupName;
    [SerializeField] private ArticulationBody fingerArticulationBody;
    [SerializeField] private float fingerJointThreshold;

    [Space(8.0f)]

    [SerializeField] private int planningAttempts = 1;
    [SerializeField] private double allowedPlanningTime = 5.0;

    [SerializeField] private double maxVelocityScalingFactor = 1.0;
    [SerializeField] private double maxAccelerationScalingFactor = 1.0;

    [SerializeField] private double positionTolerance = 1e-3;
    [SerializeField] private double orientationTolerance = 1e-3;
    [SerializeField] private double jointTolerance = 1e-3;

    private ROSConnection rosConnection;
    private ROSTime rosTime;

    private TFSystem tfSystem;

    private ROSActionClient<MoveGroupGoal, MoveGroupResult, MoveGroupFeedback> planTrajectoryActionClient;
    private ROSActionClient<ExecuteTrajectoryGoal, ExecuteTrajectoryResult, ExecuteTrajectoryFeedback> executeTrajectoryActionClient;

    private State state;
    private RobotTrajectoryMsg latestTrajectory;

    public void Start()
    {
        rosConnection = ROSConnection.GetOrCreateInstance();
        rosTime = ROSTime.GetOrCreateInstance();

        tfSystem = TFSystem.GetOrCreateInstance();

        planTrajectoryActionClient = new ROSActionClient<MoveGroupGoal, MoveGroupResult, MoveGroupFeedback>(planTrajectoryActionName);
        executeTrajectoryActionClient = new ROSActionClient<ExecuteTrajectoryGoal, ExecuteTrajectoryResult, ExecuteTrajectoryFeedback>(executeTrajectoryActionName);

        state = State.HasNoPlan;

        planTrajectoryAction.Enable();
        planTrajectoryAction.performed += (inputAction) => PlanTrajectory();

        executeTrajectoryAction.Enable();
        executeTrajectoryAction.performed += (inputAction) => ExecuteTrajectory();

        toggleGripperAction.Enable();
        toggleGripperAction.performed += (inputAction) => ToggleGripper();

        gripperIndicator.gameObject.SetVisability(false);
        feedbackText.text = "Welcome!";
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

                gripperIndicator.gameObject.SetVisability(true);
                gripperIndicator.transform.position = gameObject.transform.position;
                gripperIndicator.transform.rotation = gameObject.transform.rotation;
                var tintColor = Color.blue;
                tintColor.a = 0.5f;
                gripperIndicator.SetTintColor(tintColor);

                var pose = GetRelativePose(referenceFrameId, targetLinkTransform);
                var goal = GetTrajectoryPlanGoal(pose);

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

                            // var errorId = result.error_code.val;
                            // var errorMessage = result.error_code.GetErrorMessage();
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

                gripperIndicator.gameObject.SetVisability(false);

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
        var errorPrefix = "Cannot toggle gripper: ";

        switch (state)
        {
            case State.HasNoPlan:
            case State.HasFailedPlan:
            case State.HasSuccessfulPlan:
                state = State.WaitingForPlan;

                var gripperState = GetGripperState();
                var goalGripperState = gripperState switch
                {
                    GripperState.Open => GripperState.Closed,
                    GripperState.Closed => GripperState.Open,
                    _ => GripperState.Open,
                };
                var goal = GetGripperPlanGoal(goalGripperState);

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

    private PoseStampedMsg GetRelativePose(string referenceFrameId, Transform relativeTransform)
    {
        var referenceObject = tfSystem.GetTransformObject(referenceFrameId);

        var relativePosition = referenceObject.transform.InverseTransformPoint(transform.position);
        var relativeRotation = referenceObject.transform.InverseTransformRotation(transform.rotation);

        var pose = new PoseStampedMsg();
        pose.header.stamp = rosTime.Now();
        pose.header.frame_id = this.referenceFrameId;
        pose.pose.position = relativePosition.To<FLU>();
        pose.pose.orientation = relativeRotation.To<FLU>();

        return pose;
    }

    private GripperState GetGripperState()
    {
        var fingerJoint = fingerArticulationBody.jointPosition[0];
        var isOpen = fingerJoint < fingerJointThreshold;
        return isOpen switch
        {
            true => GripperState.Open,
            false => GripperState.Closed,
        };
    }

    private WorkspaceParametersMsg GetWorkspaceParameters()
    {
        // http://docs.ros.org/en/api/moveit_msgs/html/msg/WorkspaceParameters.html
        var workspaceParameters = new WorkspaceParametersMsg();
        workspaceParameters.header.frame_id = referenceFrameId;
        workspaceParameters.header.stamp = rosTime.Now();
        // TODO: Allow the workspace to be specified
        workspaceParameters.min_corner.x = -1.0;
        workspaceParameters.min_corner.y = -0.0;
        workspaceParameters.min_corner.z = -1.0;
        workspaceParameters.max_corner.x = +1.0;
        workspaceParameters.max_corner.y = +2.0;
        workspaceParameters.max_corner.z = +1.0;
        return workspaceParameters;
    }

    private MotionPlanRequestMsg GetPartialMotionPlanRequest()
    {
        // http://docs.ros.org/en/api/moveit_msgs/html/msg/MotionPlanRequest.html
        var request = new MotionPlanRequestMsg();
        request.workspace_parameters = GetWorkspaceParameters();
        request.start_state.is_diff = true;
        // request.goal_constraints = ...;
        // request.path_constraints = ...;
        // request.trajectory_constraints = ...;
        // request.reference_trajectories = ...;
        // request.pipeline_id = ...;
        // request.planner_id = ...;
        // request.group_name = ...;
        request.num_planning_attempts = planningAttempts;
        request.allowed_planning_time = allowedPlanningTime;
        request.max_velocity_scaling_factor = maxVelocityScalingFactor;
        request.max_acceleration_scaling_factor = maxAccelerationScalingFactor;
        // request.cartesian_speed_end_effector_link = ...;
        // request.max_cartesian_speed = ...;
        return request;
    }

    private PlanningOptionsMsg GetPlanningOptions()
    {
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
        return planningOptions;
    }

    private MoveGroupGoal GetPartialMoveGroupGoal()
    {
        var moveGroupGoal = new MoveGroupGoal();
        moveGroupGoal.request = GetPartialMotionPlanRequest();
        moveGroupGoal.planning_options = GetPlanningOptions();
        return moveGroupGoal;
    }

    private MoveGroupGoal GetTrajectoryPlanGoal(PoseStampedMsg pose)
    {
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
        positionConstraint.weight = 1.0;

        // http://docs.ros.org/en/api/moveit_msgs/html/msg/OrientationConstraint.html
        var orientationConstraint = new OrientationConstraintMsg();
        orientationConstraint.header = pose.header;
        orientationConstraint.link_name = targetLinkName;
        orientationConstraint.orientation = pose.pose.orientation;
        orientationConstraint.absolute_x_axis_tolerance = orientationTolerance;
        orientationConstraint.absolute_y_axis_tolerance = orientationTolerance;
        orientationConstraint.absolute_z_axis_tolerance = orientationTolerance;
        // orientation_constraint.parameterization = ...; 
        orientationConstraint.weight = 1.0;

        // http://docs.ros.org/en/api/moveit_msgs/html/msg/VisibilityConstraint.html
        var visibilityConstraint = new VisibilityConstraintMsg();
        visibilityConstraint.target_radius = 0.1;
        visibilityConstraint.target_pose = GetRelativePose(referenceFrameId, targetVisibilityTransform);
        visibilityConstraint.cone_sides = 8;
        visibilityConstraint.sensor_pose.header.frame_id = sensorFrameId;
        visibilityConstraint.sensor_pose.header.stamp = rosTime.Now();
        // visibilityConstraint.sensor_pose.pose = ...;
        visibilityConstraint.max_view_angle = 0.0;
        visibilityConstraint.max_range_angle = 0.0;
        visibilityConstraint.sensor_view_direction = VisibilityConstraintMsg.SENSOR_X;
        visibilityConstraint.weight = visibilityConstraintWeight;

        // http://docs.ros.org/en/api/moveit_msgs/html/msg/Constraints.html
        var goalConstraint = new ConstraintsMsg();
        // goal_constraint.name = ...;
        // goalConstraint.joint_constraints = ...;
        goalConstraint.position_constraints = new PositionConstraintMsg[1];
        goalConstraint.position_constraints[0] = positionConstraint;
        goalConstraint.orientation_constraints = new OrientationConstraintMsg[1];
        goalConstraint.orientation_constraints[0] = orientationConstraint;
        if (useVisibilityConstraint)
        {
            goalConstraint.visibility_constraints = new VisibilityConstraintMsg[1];
            goalConstraint.visibility_constraints[0] = visibilityConstraint;
        }

        var moveGroupGoal = GetPartialMoveGroupGoal();
        moveGroupGoal.request.group_name = armGroupName;
        moveGroupGoal.request.goal_constraints = new ConstraintsMsg[1];
        moveGroupGoal.request.goal_constraints[0] = goalConstraint;

        return moveGroupGoal;
    }

    private MoveGroupGoal GetGripperPlanGoal(GripperState gripperState)
    {
        var jointPosition = gripperState switch
        {
            GripperState.Open => 0.0,
            GripperState.Closed => 0.5,
            _ => 0.0,
        };

        // http://docs.ros.org/en/api/moveit_msgs/html/msg/JointConstraint.html
        var leftJointConstraint = new JointConstraintMsg();
        leftJointConstraint.joint_name = "l_gripper_finger_joint";
        leftJointConstraint.position = jointPosition;
        leftJointConstraint.tolerance_above = jointTolerance;
        leftJointConstraint.tolerance_below = jointTolerance;
        leftJointConstraint.weight = 1.0f;

        // http://docs.ros.org/en/api/moveit_msgs/html/msg/JointConstraint.html
        var rightJointConstraint = new JointConstraintMsg();
        rightJointConstraint.joint_name = "r_gripper_finger_joint";
        rightJointConstraint.position = jointPosition;
        rightJointConstraint.tolerance_above = jointTolerance;
        rightJointConstraint.tolerance_below = jointTolerance;
        rightJointConstraint.weight = 1.0;

        // http://docs.ros.org/en/api/moveit_msgs/html/msg/Constraints.html
        var goalConstraint = new ConstraintsMsg();
        // goal_constraint.name = ...;
        goalConstraint.joint_constraints = new JointConstraintMsg[2];
        goalConstraint.joint_constraints[0] = leftJointConstraint;
        goalConstraint.joint_constraints[1] = rightJointConstraint;
        // goalConstraint.position_constraints = ...;
        // goalConstraint.orientation_constraints = ...;
        // goalConstraint.visibility_constraints = ...;

        var moveGroupGoal = GetPartialMoveGroupGoal();
        moveGroupGoal.request.group_name = gripperGroupName;
        moveGroupGoal.request.goal_constraints = new ConstraintsMsg[1];
        moveGroupGoal.request.goal_constraints[0] = goalConstraint;

        return moveGroupGoal;
    }
}
