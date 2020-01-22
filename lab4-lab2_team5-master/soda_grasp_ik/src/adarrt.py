#!/usr/bin/env python

import time
import random
import adapy
import numpy as np
from numpy.random import choice
import rospy


class AdaRRT():
    """
    Rapidly-Exploring Random Trees (RRT) for the ADA controller.
    """
    joint_lower_limits = np.array([-3.14, 1.57, 0.33, -3.14, 0, 0])
    joint_upper_limits = np.array([3.14, 5.00, 5.00, 3.14, 3.14, 3.14])

    class Node():
        """
        A node for a doubly-linked tree structure.
        """
        def __init__(self, state, parent):
            self.state = np.asarray(state)
            """
            :param state: np.array of a state in the search space.
            :param parent: parent Node object.
            """

            self.parent = parent
            self.children = []

        def __iter__(self):
            """
            Breadth-first iterator.
            """
            nodelist = [self]
            while nodelist:
                node = nodelist.pop(0)
                nodelist.extend(node.children)
                yield node

        def __repr__(self):
            return 'Node({})'.format(', '.join(map(str, self.state)))

        def add_child(self, state):
            """
            Adds a new child at the given state.
            :param state: np.array of new child node's stateea
            :returns: child Node object.
            """
            child = AdaRRT.Node(state=state, parent=self)
            self.children.append(child)
            return child

    def __init__(self,
             start_state,
             goal_state,
             ada,
             joint_lower_limits=None,
             joint_upper_limits=None,
             ada_collision_constraint=None,
             step_size=0.25,
             goal_precision=1.0,
             max_iter=10000):
        """
        :param start_state: Array representing the starting state.
        :param goal_state: Array representing the goal state.
        :param ada: libADA instance.
        :param joint_lower_limits: List of lower bounds of each joint.
        :param joint_upper_limits: List of upper bounds of each joint.
        :param ada_collision_constraint: Collision constraint object.
        :param step_size: Distance between nodes in the RRT.
        :param goal_precision: Maximum distance between RRT and goal before
            declaring completion.
        :param sample_near_goal_prob:
        :param sample_near_goal_range:
        :param max_iter: Maximum number of iterations to run the RRT before
            failure.
        """
        self.start = AdaRRT.Node(start_state, None)
        self.goal = AdaRRT.Node(goal_state, None)
        self.ada = ada
        self.joint_lower_limits = joint_lower_limits or AdaRRT.joint_lower_limits
        self.joint_upper_limits = joint_upper_limits or AdaRRT.joint_upper_limits
        self.ada_collision_constraint = ada_collision_constraint
        self.step_size = step_size
        self.goal_precision = goal_precision
        self.max_iter = max_iter
        self.goal_sample_range = 0.05

    def build(self):
        """
        Build an RRT.
        In each step of the RRT:
            1. Sample a random point.
            2. Find its nearest neighbor.
            3. Attempt to create a new node in the direction of sample from its
                nearest neighbor.
            4. If we have created a new node, check for completion.
        Once the RRT is complete, add the goal node to the RRT and build a path
        from start to goal.
        :returns: A list of states that create a path from start to
            goal on success. On failure, returns None.
        """
        for k in range(self.max_iter):
            pd = [0.2,0.8]
            #function_names = [self._get_random_sample_near_goal,self._get_nearest_neighbor]
            draw = choice([0,1],1,p=pd)
            if draw[0]==0:
                sample_node=self._get_random_sample_near_goal()
            else:
                sample_node=self._get_random_sample()
            
            
            near_nei=self._get_nearest_neighbor(sample_node)
            new_node=self._extend_sample(sample_node, near_nei)
            #print("did not get")
            # FILL in your code here

            if new_node and self._check_for_completion(new_node):
                print("got")
                new_node.add_child(self.goal)
                path=self._trace_path_from_start(node=new_node)
                path.append(self.goal.state)
                return path

                # FILL in your code here

        print("Failed to find path from {0} to {1} after {2} iterations!".format(
            self.start.state, self.goal.state, self.max_iter))

    def _get_random_sample_near_goal(self):
        ran_vec = []
        for i in range(len(self.joint_lower_limits)):
            ran_vec.append(random.uniform(self.goal.state[i] - 0.05, self.goal.state[i] + 0.05))
        return np.array(ran_vec)

    def _get_random_sample(self):
        ran_vec=[]
        for i in range(len(self.joint_lower_limits)):
            ran_vec.append(random.uniform(self.joint_lower_limits[i],self.joint_upper_limits[i]))
        return np.array(ran_vec)
        """
        Uniformly samples the search space.
        :returns: A vector representing a randomly sampled point in the search
        space.
        """
    # FILL in your code here

    def _get_nearest_neighbor(self, sample):
        distances = [np.linalg.norm(node.state-sample) for node in self.start.__iter__()]

        if len(distances)==0:
            return self.start

        minimum=min(distances)
        for node in self.start.__iter__():
            if(np.linalg.norm(node.state-sample)==minimum):
                return node
        """
        Finds the closest node to the given sample in the search space,
        excluding the goal node.
        :param sample: The target point to find the closest neighbor to.
        :returns: A Node object for the closest neighbor.
        """
    # FILL in your code here

    def _extend_sample(self, sample, neighbor):
        new_node=neighbor.state+(((sample-neighbor.state)/np.linalg.norm(sample-neighbor.state))*self.step_size)

        if self._check_for_collision(new_node):
            return None

        neighbor.add_child(new_node)
        return AdaRRT.Node(new_node, neighbor)
    """
    Adds a new node to the RRT between neighbor and sample, at a distance
    step_size away from neighbor. The new node is only created if it will
    not collide with any of the collision objects (see
    RRT._check_for_collision)
    :param sample: target point
    :param neighbor: closest existing node to sample
    :returns: The new Node object. On failure (collision), returns None.
    """
    # FILL in your code here

    def _check_for_completion(self, node):
        if np.linalg.norm(node.state-self.goal.state)<=self.goal_precision:
            return True
        return False
    """
    Check whether node is within self.step_size distance of the goal.
    :param node: The target Node
    :returns: Boolean indicating node is close enough for completion.
    """
    # FILL in your code here

    def _trace_path_from_start(self, node=None):
        if node==None:
            list_states=[self.goal.state]
            parents=self.goal.parent

            while(parents!=None):
                list_states.append(parents.state)
                parents=parents.parent

            return list_states[::-1]

        list_states=[node.state]
        parents=node.parent

        while(parents!=None):
            list_states.append(parents.state)
            parents=parents.parent

        return list_states[::-1]
    """
    Traces a path from start to node, if provided, or the goal otherwise.
    :param node: The target Node at the end of the path. Defaults to
        self.goal
    :returns: A list of states (not Nodes!) beginning at the start state and
        ending at the goal state.
    """
    # FILL in your code here

    def _check_for_collision(self, sample):

        """
        Checks if a sample point is in collision with any collision object.
        :returns: A boolean value indicating that sample is in collision.
        """
        if self.ada_collision_constraint is None:
            return False
        return self.ada_collision_constraint.is_satisfied(
        self.ada.get_arm_state_space(),
        self.ada.get_arm_skeleton(), sample)


def main():
    sim = True

    # instantiate an ada
    ada = adapy.Ada(True)

    armHome = [-1.5, 3.22, 1.23, -2.19, 1.8, 1.2]
    goalConfig = [-1.72, 4.44, 2.02, -2.04, 2.66, 1.39]
    delta = 0.25
    eps = 0.2

    if sim:
        ada.set_positions(goalConfig)

    # launch viewer
    viewer = ada.start_viewer("dart_markers/simple_trajectories", "map")

    # add objects to world
    canURDFUri = "package://pr_assets/data/objects/can.urdf"
    sodaCanPose = [0.25, -0.35, 0.0, 0, 0, 0, 1]
    tableURDFUri = "package://pr_assets/data/furniture/uw_demo_table.urdf"
    tablePose = [0.3, 0.0, -0.7, 0.707107, 0, 0, 0.707107]
    world = ada.get_world()
    can = world.add_body_from_urdf(canURDFUri, sodaCanPose)
    table = world.add_body_from_urdf(tableURDFUri, tablePose)

    # add collision constraints
    collision_free_constraint = ada.set_up_collision_detection(
        ada.get_arm_state_space(),
        ada.get_arm_skeleton(),
        [can, table])
    full_collision_constraint = ada.get_full_collision_constraint(
        ada.get_arm_state_space(),
        ada.get_arm_skeleton(),
        collision_free_constraint)

    # easy goal
    adaRRT = AdaRRT(
    start_state=np.array(armHome),
    goal_state=np.array(goalConfig),
    ada=ada,
    ada_collision_constraint=full_collision_constraint,
    step_size=delta,
    goal_precision=eps)

    rospy.sleep(1.0)

    path = adaRRT.build()
    if path is not None:
        print("Path waypoints:")
        print(np.asarray(path))
        waypoints = []
        for i, waypoint in enumerate(path):
            waypoints.append((0.0 + i, waypoint))

        t0 = time.clock()
        traj = ada.compute_smooth_joint_space_path(
            ada.get_arm_state_space(), waypoints)
        t = time.clock() - t0
        print(str(t) + "seconds elapsed")
        raw_input('Press ENTER to execute trajectory and exit')
        ada.execute_trajectory(traj)
        rospy.sleep(1.0)

if __name__ == '__main__':
    main()
