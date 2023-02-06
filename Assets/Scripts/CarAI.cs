using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;


public class Node
{
    public Node parent;
    public Vector3 pos;
    public float cost;
    public float theta;
    public float velocity;
    public Node(Vector3 pos, Node parent, float cost)// float velocity, float theta)
    {
        this.parent = parent;
        this.pos = pos;
        this.cost = cost;
        //this.velocity = velocity;
        //this.theta = theta;
    }
    public Node(Vector3 pos)
    {
        this.parent = null;
        this.pos = pos;
        this.cost = 0;
    }
}

namespace UnityStandardAssets.Vehicles.Car
{
    [RequireComponent(typeof(CarController))]
    public class CarAI : MonoBehaviour
    {
        private CarController m_Car; // the car controller we want to use
        private Rigidbody m_Car_rb;

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


        private void Start()
        {
            // get the car controller
            m_Car = GetComponent<CarController>();
            terrain_manager = terrain_manager_game_object.GetComponent<TerrainManager>();

            m_Car_rb = GetComponent<Rigidbody>();

            // Plan your path here
            // Replace the code below that makes a random path
            // ...

            start_pos = terrain_manager.myInfo.start_pos;
            goal_pos = terrain_manager.myInfo.goal_pos;
            x_low = terrain_manager.myInfo.x_low;
            x_high = terrain_manager.myInfo.x_high;
            z_low = terrain_manager.myInfo.z_low;
            z_high = terrain_manager.myInfo.z_high;
            x_N = terrain_manager.myInfo.x_N;
            z_N = terrain_manager.myInfo.z_N;
            traversability = terrain_manager.myInfo.traversability;
            // Debug.Log(x_low);
            // Debug.Log(x_high);
            // Debug.Log(z_low);
            // Debug.Log(z_high);

            Node start_node = new Node(start_pos);
            graph.Add(start_node);
            Debug.Log("start_pos: " + start_pos);
            Node lastNode = null;

            traversability_larger_obstacle = CreateNewMap();
            /*
            Debug.Log("larger obstacle: " + traversability_larger_obstacle);
            for (int i = 0; i < x_N * 3; i++)
            {
                for (int j = 0; j < z_N * 3; j++)
                { 
                    Debug.Log(i + " " + j + " " + traversability_larger_obstacle[i, j]);
                }
            }
            */

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

            // Plot your path to see if it makes sense
            // Note that path can only be seen in "Scene" window, not "Game" window
            /*Node old_wp = lastNode;
            Node wp = lastNode.parent;
            while(wp != null)
            {
                Debug.DrawLine(old_wp.pos, wp.pos, Color.red, 100f);
                old_wp = wp;
            }*/
            while (lastNode != null)
            {
                rough_path.Add(lastNode);
                if (lastNode.parent != null) Debug.Log("lastnode.parent: " + lastNode.parent.pos);
                lastNode = lastNode.parent;
            }
            Debug.Log("path length: " + rough_path.Count);
            rough_path.Reverse();
            my_path = GetCenterPath(start_node);
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
                    if(CollisionFree(newNode.pos,neighbour.pos))
                    {
                        neighbour.parent = newNode;
                        neighbour.cost = dist_neighbour_newNode + newNode.cost;

                    }
                }
            }
        }
        /*
        private bool LineIntersectsRectangle(Vector2 lineStart, Vector2 lineEnd, Rect rectangle)
        {
            Vector2[] rectPoints = new Vector2[4];
            rectPoints[0] = new Vector2(rectangle.xMin, rectangle.yMin);
            rectPoints[1] = new Vector2(rectangle.xMax, rectangle.yMin);
            rectPoints[2] = new Vector2(rectangle.xMax, rectangle.yMax);
            rectPoints[3] = new Vector2(rectangle.xMin, rectangle.yMax);

            Vector2 lineDirection = (lineEnd - lineStart).normalized;
            Vector2 lineNormal = new Vector2(-lineDirection.y, lineDirection.x);

            float minRect = float.MaxValue;
            float maxRect = float.MinValue;
            float minLine = float.MaxValue;
            float maxLine = float.MinValue;

            for (int i = 0; i < rectPoints.Length; i++)
            {
                float projected = Vector2.Dot(rectPoints[i], lineNormal);
                minRect = Mathf.Min(minRect, projected);
                maxRect = Mathf.Max(maxRect, projected);
            }

            float projectedLineStart = Vector2.Dot(lineStart, lineNormal);
            float projectedLineEnd = Vector2.Dot(lineEnd, lineNormal);

            minLine = Mathf.Min(projectedLineStart, projectedLineEnd);
            maxLine = Mathf.Max(projectedLineStart, projectedLineEnd);

            if (maxLine < minRect || maxRect < minLine)
            {
                return false;
            }

            return true;
        }
        */
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
            //nextPos =  = my_path[itr_cnt];
            //agent.position = Vector3.Lerp(transform.position, nextPos, Time.deltaTime * stepSize);
            //agent.rotation = Quaternion.Euler(0, nextHeading.y, 0);

            // this is how you access information about the terrain from the map
            int i = terrain_manager.myInfo.get_i_index(transform.position.x);
            int j = terrain_manager.myInfo.get_j_index(transform.position.z);
            float grid_center_x = terrain_manager.myInfo.get_x_pos(i);
            float grid_center_z = terrain_manager.myInfo.get_z_pos(j);

            Debug.DrawLine(transform.position, new Vector3(grid_center_x, 0f, grid_center_z));

            // this is how you access information about the terrain from a simulated laser range finder
            RaycastHit hit;
            float maxRange = 50f;
            if (Physics.Raycast(transform.position + transform.up, transform.TransformDirection(Vector3.forward), out hit, maxRange))
            {
                Vector3 closestObstacleInFront = transform.TransformDirection(Vector3.forward) * hit.distance;
                Debug.DrawRay(transform.position, closestObstacleInFront, Color.yellow);
                // Debug.Log("Did Hit");
            }

            /*
            steering = Mathf.Clamp(steering, -1, 1); // -1:left, 1:right
            AccelInput = accel = Mathf.Clamp(accel, 0, 1);
            BrakeInput = footbrake = -1 * Mathf.Clamp(footbrake, -1, 0); //1: forward -1:backward
            handbrake = Mathf.Clamp(handbrake, 0, 1);
            */
            float speed = 10f;
            Vector3 next_pos = my_path[pathpoint_index].pos;
            Vector3 car_pos = transform.position;
            Vector3 acceleration = 0.5f * ((next_pos - car_pos) + m_Car_rb.velocity * (5f - m_Car_rb.velocity.magnitude));
            float angle = Vector3.Angle(acceleration, transform.forward);
            var left_or_right = Vector3.Cross(acceleration, transform.forward);
            if (left_or_right.y > 0) angle = -angle;
            float steering = angle / 180f;
            Debug.Log("steering: " + steering);

            Debug.DrawLine(transform.position, my_path[pathpoint_index].pos, Color.yellow);
            if (Physics.Raycast(transform.position + transform.up, transform.TransformDirection(Vector3.forward), out hit, 5))
            {
                if((Time.time - hit_time) > 4)
                {
                    is_hit = true;
                    hit_time = Time.time;
                }
            }
            
            if (stop)
            {
                m_Car.Move(0f, 0f, 0f, 1f);
            }
            else if (is_hit)
            {
                if ((Time.time - hit_time) < 2.5)
                {
                    m_Car.Move(0f, 0f, -0.6f, 0f);
                }
                else if ((Time.time - hit_time) < 4 && m_Car_rb.velocity.magnitude > 0.1)
                {
                    m_Car.Move(0f, 0f, 0f, 1f);
                }
                else
                {
                    is_hit = false;
                }
            }
            else{
                float min_dis = float.MaxValue;
                int min_idx = pathpoint_index;
                
                for (int idx = pathpoint_index; idx < my_path.Count; idx++)
                {
                    float car_pathpoint_dis = Vector3.Distance(my_path[idx].pos, car_pos);
                    if (car_pathpoint_dis < min_dis && idx - pathpoint_index <= 2)
                    {
                        min_dis = car_pathpoint_dis;
                        min_idx = idx;
                    }
                }
                
                pathpoint_index = min_idx;
                next_pos = my_path[pathpoint_index].pos;
                
                float dist_to_next = Vector3.Distance(car_pos, next_pos);
                if (dist_to_next < 7f && pathpoint_index + 1 < my_path.Count)
                {
                    pathpoint_index += 1;
                }

                if (Vector3.Distance(car_pos, goal_pos) <= 1f)
                {
                    Debug.Log("Stop!");
                    m_Car.Move(0f, 0f, 0f, 1f);
                    stop = true;
                    Debug.Log("Stop Time: " + Time.time);
                }
                else if (Vector3.Distance(car_pos, goal_pos) <= goalThreshold)
                {
                    Debug.Log("Reach goal!");
                    m_Car.Move(steering, 0f, -0.1f, 0.8f);
                    Debug.Log("Reach Time: " + Time.time);
                }
                else if (Vector3.Angle(next_pos - car_pos, transform.forward) > 60f)
                {
                    Debug.Log("Large angle");
                    if (left_or_right.y > 0) steering = -1;
                    else steering = 1;
                    if(m_Car_rb.velocity.magnitude <= 2) m_Car.Move(steering, 0.4f, 0f, 0f);
                    else m_Car.Move(steering, 0f, 0f, 0f);
                }
                else if (Vector3.Angle(next_pos - car_pos, transform.forward) < 30f)
                {
                    Debug.Log("Small angle");
                    if (left_or_right.y > 0) steering = -1;
                    else steering = 1;
                    if (m_Car_rb.velocity.magnitude <= speed) m_Car.Move(steering, 0.8f, 0f, 0f);
                    else m_Car.Move(steering, 0.3f, 0f, 0f);
                }
                else
                {
                    Debug.Log("Normal");
                    m_Car.Move(steering, Vector3.Dot(acceleration, transform.forward), 0f, 0f);
                }
            }
      
        }
    }
}
