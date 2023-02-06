using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;



[RequireComponent(typeof(DroneController))]
public class DroneAI : MonoBehaviour
{

    private DroneController m_Drone; // the car controller we want to use
    private Rigidbody m_Drone_rb;
    public GameObject terrain_manager_game_object;
    TerrainManager terrain_manager;

    private List<Node> graph = new List<Node>();
    private List<Node> rough_path = new List<Node>();
    private List<Node> my_path = new List<Node>();
    Vector3 start_pos;
    Vector3 goal_pos;

    private int itr_num = 0;
    private int pathpoint_index = 1;
    private float x_low, x_high, z_low, z_high;
    private int x_N, z_N;
    private float[,] traversability;
    private float[,] traversability_larger_obstacle;

    private bool stop = false;
    private float start_time;
    private bool is_hit = false;
    private float hit_time = 0;

    // hyper-parameter
    private float stepSize = 1f;
    private float goalThreshold = 4f;
    private float radius = 5f;
    private int co = 3;

    private Vector3 drone_pos_cur = new Vector3();
    private Vector3 drone_pos_pre = new Vector3();


    private void Start()
    {
        // get the drone controller
        m_Drone = GetComponent<DroneController>();
        terrain_manager = terrain_manager_game_object.GetComponent<TerrainManager>();
        m_Drone_rb = GetComponent<Rigidbody>();

        start_pos = terrain_manager.myInfo.start_pos;
        goal_pos = terrain_manager.myInfo.goal_pos;
        x_low = terrain_manager.myInfo.x_low;
        x_high = terrain_manager.myInfo.x_high;
        z_low = terrain_manager.myInfo.z_low;
        z_high = terrain_manager.myInfo.z_high;
        x_N = terrain_manager.myInfo.x_N;
        z_N = terrain_manager.myInfo.z_N;
        traversability = terrain_manager.myInfo.traversability;

        drone_pos_cur = start_pos;
        Node start_node = new Node(start_pos);
        graph.Add(start_node);
        Debug.Log("start_pos: " + start_pos);
        Node lastNode = null;

        traversability_larger_obstacle = CreateNewMap();

        while (true)
        {
            itr_num += 1;
            // Debug.Log("------------------- Iteration: " + itr_num + " -------------------");
            Vector3 randomPoint = GenerateRandomPoint();
            // Debug.Log("randomPoint: " + randomPoint);

            Node nearestNode = FindNearestNode(randomPoint);
            // Debug.Log("nearestNode: " + nearestNode.pos);

            Vector3 newPoint = Steer(randomPoint, nearestNode.pos);
            Debug.Log("newPoint: " + newPoint);

            if (CollisionFree(newPoint, nearestNode.pos))
            {
                List<Node> neighbours = FindNeighbours(newPoint);
                var parentTuple = ChooseParent(neighbours, newPoint, nearestNode.pos);
                float cost = parentTuple.Item1;
                Node parent = parentTuple.Item2;
                Node newNode = new Node(newPoint, parent, cost);
                if (parent != null)
                {
                    rewire(neighbours, newNode);
                    graph.Add(newNode);
                }
                else
                {
                    //newNode.parent = nearestNode;
                    graph.Add(newNode);
                }

                lastNode = newNode;
                Debug.Log("lastNode: " + lastNode.pos);

                if (ReachedGoal(newNode))
                {
                    Debug.Log("Found goal!");
                    break;
                }
            }
        }

        while (lastNode != null)
        {
            rough_path.Add(lastNode);
            if (lastNode.parent != null) Debug.Log("lastnode.parent: " + lastNode.parent.pos);
            lastNode = lastNode.parent;
        }
        rough_path.Reverse();
        my_path = rough_path;//GetCenterPath(start_node);
        Debug.Log("path length: " + my_path.Count);
        for (int i = 0; i < my_path.Count - 1; i++)
        {
            Debug.DrawLine(my_path[i].pos, my_path[i + 1].pos, Color.red, 100f);
        }

        start_time = Time.time;
    }


    private float[,] CreateNewMap()
    {

        float[,] traversability_larger_obstacle = new float[x_N * co, z_N * co];
        int start_i = terrain_manager.myInfo.get_i_index(start_pos.x, co);
        int start_j = terrain_manager.myInfo.get_j_index(start_pos.z, co);
        int goal_i = terrain_manager.myInfo.get_i_index(goal_pos.x, co);
        int goal_j = terrain_manager.myInfo.get_j_index(goal_pos.z, co);

        for (int i = 0; i < x_N; i++)
        {
            for (int j = 0; j < z_N; j++)
            {
                if (traversability[i, j] == 1f)
                {
                    for (int k = 0; k < co; k++)
                    {
                        for (int p = 0; p < co; p++)
                        {
                            traversability_larger_obstacle[i * co + k, j * co + p] = 1f;
                        }
                    }
                }
            }
        }

        for (int i = 0; i < x_N * co; i++)
        {
            for (int j = 0; j < z_N * co; j++)
            {
                // Debug.Log("newpos: " + (int)Math.Floor((float)i / (float)co) + " " + (int)Math.Floor((float)j / (float)co));
                int tmp_i = (int)Math.Floor((float)i / (float)co);
                int tmp_j = (int)Math.Floor((float)j / (float)co);
                if (traversability[tmp_i, tmp_j] == 1f)
                {
                    if (i - 1 >= 0) traversability_larger_obstacle[i - 1, j] = 1f; //left
                    if (i + 1 <= x_N * co - 1) traversability_larger_obstacle[i + 1, j] = 1f; //right
                    if (j - 1 >= 0) traversability_larger_obstacle[i, j - 1] = 1f; //down
                    if (j + 1 <= z_N * co - 1) traversability_larger_obstacle[i, j + 1] = 1f; //up
                    if (i - 1 >= 0 && j - 1 >= 0) traversability_larger_obstacle[i - 1, j - 1] = 1f; //low left
                    if (i - 1 >= 0 && j + 1 <= z_N * co - 1) traversability_larger_obstacle[i - 1, j + 1] = 1f; //upper left
                    if (i + 1 <= x_N * co - 1 && j - 1 >= 0) traversability_larger_obstacle[i + 1, j - 1] = 1f; //low right
                    if (i + 1 <= x_N * co - 1 && j + 1 <= z_N * co - 1) traversability_larger_obstacle[i + 1, j + 1] = 1f; //upper right
                }
            }
        }

        traversability_larger_obstacle[start_i, start_j] = 0f;
        traversability_larger_obstacle[start_i - 1, start_j] = 0f;
        traversability_larger_obstacle[start_i + 1, start_j] = 0f;
        traversability_larger_obstacle[start_i - 2, start_j] = 0f;
        traversability_larger_obstacle[start_i + 2, start_j] = 0f;
        traversability_larger_obstacle[start_i - 3, start_j] = 0f;
        traversability_larger_obstacle[start_i + 3, start_j] = 0f;
        traversability_larger_obstacle[goal_i, goal_j] = 0f;
        /*
        float x_step = (x_high - x_low) / x_N / co;
        float z_step = (z_high - z_low) / z_N / co;
        for (int i = 0; i < x_N * co; i++)
        {
            for (int j = 0; j < z_N * co; j++)
            {
                if (traversability_larger_obstacle[i, j] > 0.5f)
                {
                    GameObject cube = GameObject.CreatePrimitive(PrimitiveType.Cube);
                    cube.transform.position = new Vector3(terrain_manager.myInfo.get_x_pos(i, co), 0.0f, terrain_manager.myInfo.get_z_pos(j, co));
                    cube.transform.localScale = new Vector3(x_step, 15.0f, z_step);
                }

            }
        }
        */
        return traversability_larger_obstacle;
    }

    private List<Node> GetCenterPath(Node start_node)
    {
        Node goal_node = rough_path[rough_path.Count - 1];
        int last_i = terrain_manager.myInfo.get_i_index(start_pos.x);
        int last_j = terrain_manager.myInfo.get_j_index(start_pos.z);
        my_path.Add(start_node);
        foreach (Node node in rough_path)
        {
            int i = terrain_manager.myInfo.get_i_index(node.pos.x);
            int j = terrain_manager.myInfo.get_j_index(node.pos.z);

            if (i != last_i || j != last_j)
            {
                Debug.Log("node pos:" + i + " " + j);

                float x = terrain_manager.myInfo.get_x_pos(i);
                float z = terrain_manager.myInfo.get_z_pos(j);
                my_path.Add(new Node(new Vector3(x, 0, z), node.parent, node.cost));
                last_i = i;
                last_j = j;
            }
        }
        my_path.Add(goal_node);
        return my_path;

    }


    private Vector3 GenerateRandomPoint()
    {
        // Generate a random point within the environment bounds
        float x = UnityEngine.Random.Range(x_low, x_high);
        float z = UnityEngine.Random.Range(z_low, z_high);
        return new Vector3(x, 0, z);
    }

    private Node FindNearestNode(Vector3 point)
    {
        // Use a nearest-neighbor algorithm to find the nearest node in the my_path
        Node nearestNode = graph[0];
        float nearestDist = float.MaxValue;
        for (int i = 0; i < graph.Count; i++)
        {
            float dist = Vector3.Distance(point, graph[i].pos);
            if (dist < nearestDist)
            {
                nearestDist = dist;
                nearestNode = graph[i];
            }
        }
        return nearestNode;
    }

    private Vector3 Steer(Vector3 randomPoint, Vector3 nearestPoint)
    {
        // Generate a new point along the line between the random point and the nearest point
        Vector3 direction = (randomPoint - nearestPoint).normalized;
        return nearestPoint + direction * stepSize;
    }

    private bool CollisionFree(Vector3 newPoint, Vector3 oldPoint)
    {
        int newPoint_x = terrain_manager.myInfo.get_i_index(newPoint.x, co); //x
        int newPoint_z = terrain_manager.myInfo.get_j_index(newPoint.z, co); //z
                                                                             // float grid_center_x = terrain_manager.myInfo.get_x_pos(i);
                                                                             // float grid_center_z = terrain_manager.myInfo.get_z_pos(j);
                                                                             // Debug.Log("i: " + i);
                                                                             // Debug.Log("j: " + j);
                                                                             // Debug.Log("grid_center_x: " + grid_center_x);
                                                                             // Debug.Log("grid_center_z: " + grid_center_z);
                                                                             // Debug.Log("size_x: " + traversability_larger_obstacle.GetLength(0));
                                                                             // Debug.Log("size_y: " + traversability_larger_obstacle.GetLength(1));
                                                                             // Debug.Log("newPoint_x: " + newPoint_x);
                                                                             // Debug.Log("newPoint_z: " + newPoint_z);
        float obstacle = traversability_larger_obstacle[newPoint_x, newPoint_z];// 1.0 if there is an obstacle on point and otherwise 0.0 
                                                                                // Debug.Log("obstacle: " + obstacle);
        if (obstacle == 1.0)
        {
            // Debug.Log("here");
            return false;
        }
        return true;
    }

    private List<Node> FindNeighbours(Vector3 newPoint)
    {
        List<Node> neighbours = new List<Node>();
        for (int i = 0; i < graph.Count; i++)
        {
            Vector3 vertex = graph[i].pos;
            float dist = Vector3.Distance(newPoint, vertex);
            if (dist < radius)
            {
                neighbours.Add(graph[i]);
            }
        }
        return neighbours;
    }

    private Tuple<float, Node> ChooseParent(List<Node> neighbours, Vector3 newPoint, Vector3 nearestPoint)
    {
        float minCost = float.MaxValue;
        Node parent = null;
        Debug.Log("number of neighbours: " + neighbours.Count);
        foreach (Node neighbour in neighbours)
        {
            float cost = Vector3.Distance(newPoint, neighbour.pos) + neighbour.cost;
            if (cost < minCost)
            {
                minCost = cost;
                parent = neighbour;
            }
        }
        return new Tuple<float, Node>(minCost, parent);
    }

    private void rewire(List<Node> neighbours, Node newNode)
    {
        foreach (Node neighbour in neighbours)
        {
            float dist_neighbour_newNode = Vector3.Distance(newNode.pos, neighbour.pos);
            if (dist_neighbour_newNode + newNode.cost < neighbour.cost)
            {
                if (CollisionFree(newNode.pos, neighbour.pos))
                {
                    neighbour.parent = newNode;
                    neighbour.cost = dist_neighbour_newNode + newNode.cost;

                }
            }
        }
    }
   
    private bool ReachedGoal(Node newNode)
    {
        // Check if the new node is within a certain distance of the goal
        return Vector3.Distance(newNode.pos, goal_pos) <= goalThreshold;
    }


    private void FixedUpdate()
    {
        Debug.Log("------------------- PathPoint Index: " + pathpoint_index + " -------------------");
        // Execute your path here
        // ...

        // this is how you access information about the terrain
        int i = terrain_manager.myInfo.get_i_index(transform.position.x);
        int j = terrain_manager.myInfo.get_j_index(transform.position.z);
        float grid_center_x = terrain_manager.myInfo.get_x_pos(i);
        float grid_center_z = terrain_manager.myInfo.get_z_pos(j);

        Debug.DrawLine(transform.position, new Vector3(grid_center_x, 0f, grid_center_z), Color.white, 1f);

        // this is how you control the car
        
        float speed = 10f;
        Vector3 next_pos = my_path[pathpoint_index].pos;
        Vector3 drone_pos = transform.position;
        Vector3 acceleration = next_pos - drone_pos + m_Drone_rb.velocity * (5f - m_Drone_rb.velocity.magnitude);
        float min_dis = float.MaxValue;
        int min_idx = pathpoint_index;

        for (int idx = pathpoint_index; idx < my_path.Count; idx++)
        {
            float drone_pathpoint_dis = Vector3.Distance(my_path[idx].pos, drone_pos);
            if (drone_pathpoint_dis < min_dis && idx - pathpoint_index <= 2)
            {
                min_dis = drone_pathpoint_dis;
                min_idx = idx;
            }
        }

        pathpoint_index = min_idx;
        next_pos = my_path[pathpoint_index].pos;
        Debug.DrawLine(transform.position, my_path[pathpoint_index].pos, Color.yellow);


        float dist_to_next = Vector3.Distance(drone_pos, next_pos);
        if (dist_to_next < 5f && pathpoint_index + 1 < my_path.Count)
        {
            pathpoint_index += 1;
        }
        int sharp_turn = 0; //1:(30,50) 2:>50
        if (pathpoint_index < my_path.Count - 6)
        {
            for (int lookahead_idx = 0; lookahead_idx < 4; lookahead_idx++)
            {
                Vector3 cur_path = my_path[pathpoint_index + lookahead_idx].pos - my_path[pathpoint_index + lookahead_idx - 1].pos;
                Vector3 next_path = my_path[pathpoint_index + lookahead_idx + 1].pos - my_path[pathpoint_index + lookahead_idx].pos;
                if (Vector3.Angle(cur_path, next_path) > 50f)
                {
                    sharp_turn = 2;
                    break;
                }
                else if (Vector3.Angle(cur_path, next_path) > 30f)
                {
                    sharp_turn = 1;
                    break;
                }
            }
        }
        if (sharp_turn == 2)
        {
            m_Drone.Move(0f, 0f);
        }
        else if(sharp_turn == 1)
        {
            m_Drone.Move(0.3f *acceleration.x, 0.3f *acceleration.z);
        }
        else
            m_Drone.Move(acceleration.x, acceleration.z);
        /*
        if (m_Drone_rb.velocity.magnitude == 0)
            m_Drone.Move(3 * acceleration.x, 3 * acceleration.z);
        else if (m_Drone_rb.velocity.magnitude <= 1)
            m_Drone.Move(10 * acceleration.x, 10 * acceleration.z);
        else
            m_Drone.Move(acceleration.x, acceleration.z);
        */
        //desired acceleration first component
        //desired acceleration second component
        /*
        drone_pos_pre = drone_pos_cur;
        drone_pos_cur = new Vector3(transform.position.x, 0, transform.position.z);
        Vector3 next_pos = my_path[pathpoint_index].pos;
        float dist_to_next = Vector3.Distance(drone_pos_cur, next_pos);
        if (dist_to_next < 3f && pathpoint_index + 1 < my_path.Count)
        {
            pathpoint_index += 1;
        }

        Vector3 v_cur = drone_pos_cur - drone_pos_pre;

        Vector3 v_next = next_pos - drone_pos_cur;

        Vector3 v = v_next - v_cur;
        Debug.Log("v: " + v);
        m_Drone.Move(v.x, v.z);
        */
    }

 
}
