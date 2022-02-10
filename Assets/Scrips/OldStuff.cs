using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class OldStuff : MonoBehaviour
{

    /*
     public void GiancarloPID_attempt()
    {
        if (up_and_smooth != null && node_index < up_and_smooth.Count)
        {
            if (inRange(old_target_pos, new Vector3(rigidbody.position.x, 0, rigidbody.position.z), 1))
                stuck_safe_exit--;
            else
                stuck_safe_exit = max_stuck_safe_exit;

            old_target_pos = new Vector3(rigidbody.position.x, 0, rigidbody.position.z);



            Vector3 target_position = up_and_smooth[node_index].worldPosition;

            float slow_down_rate = 1;

            int backup = get_closest_node(old_target_pos, up_and_smooth);
            int tolerance = 10;
            if (node_index - tolerance < backup && backup < node_index + tolerance && stuck_safe_exit != 0)
            {

                target_position = up_and_smooth[backup].worldPosition;
            }

            if (stuck_safe_exit == 0)
            {
                Debug.Log("EIEIEI");
                for (int k = 0; k < 100; k++)
                {

                    m_Car.Move(0.1f, 1f, 0, 0);
                }

                return;

                target_position = up_and_smooth[node_index + 2].worldPosition;
            }
            target_velocity = (target_position - old_target_pos) / Time.fixedDeltaTime;
            Debug.Log(node_index + ")Target: " + target_position + " Old:" + old_target_pos + " Vel: " + target_velocity + " Car_pos: " + rigidbody.position);
            //old_target_pos = target_position;

            // a PD-controller to get desired velocity
            Vector3 position_error = target_position - transform.position;
            Vector3 velocity_error = target_velocity / 2 - rigidbody.velocity;
            Vector3 desired_acceleration = k_p * position_error + k_d * velocity_error;

            float steering = Vector3.Dot(desired_acceleration, transform.right);
            float acceleration = Vector3.Dot(desired_acceleration, transform.forward) / 2;

            Debug.DrawLine(target_position, target_position + target_velocity, Color.red);
            Debug.DrawLine(transform.position, transform.position + rigidbody.velocity, Color.blue);
            Debug.DrawLine(transform.position, transform.position + desired_acceleration, Color.black);

            // this is how you control the car
            Debug.Log("Steering:" + steering + " Acceleration:" + acceleration);
            if (Math.Abs(steering) > m_Car.m_MaximumSteerAngle / 2 && stuck_safe_exit != 0)
            {
                print("HUGE: " + steering + " STuck: " + stuck_safe_exit);
                slow_down_rate = 1.0001f - steering / m_Car.m_MaximumSteerAngle;
            }

            RaycastHit hit;
            float maxRange = 5f;
            if (Physics.Raycast(transform.position + transform.up, transform.TransformDirection(Vector3.forward), out hit, maxRange))
            {
                Vector3 closestObstacleInFront = transform.TransformDirection(Vector3.forward) * hit.distance;
                Debug.DrawRay(transform.position, closestObstacleInFront, Color.yellow);
                Debug.Log("Hit " + hit.collider.gameObject.name + " at " + hit.distance);

                if (hit.distance <= 10f)
                {
                    acceleration /= 100;
                    if (hit.distance < 5f)
                    {
                        m_Car.Move(0f, -1f, 0f, 0f);
                    }
                }
                //Debug.Log(terrain_manager.myInfo.traversability[2, 2]);
            }

            if (Physics.Raycast(transform.position + transform.right, transform.TransformDirection(Vector3.right), out hit, maxRange))
            {
                Vector3 closestObstacleInFront = transform.TransformDirection(Vector3.right) * hit.distance;
                Debug.DrawRay(transform.position, closestObstacleInFront, Color.yellow);
                m_Car.Move(0f, 0f, 1f, 0f);
                Debug.Log("Hit RIGHT " + hit.collider.gameObject.name + " at " + hit.distance);

                if (hit.distance <= 10f)
                {
                    steering = -1;
                }
                //Debug.Log(terrain_manager.myInfo.traversability[2, 2]);
            }

            if (Physics.Raycast(transform.position - transform.right, transform.TransformDirection(Vector3.left), out hit, maxRange))
            {
                Vector3 closestObstacleInFront = transform.TransformDirection(Vector3.left) * hit.distance;
                Debug.DrawRay(transform.position, closestObstacleInFront, Color.yellow);
                m_Car.Move(0f, 0f, 1f, 0f);
                Debug.Log("Hit LEFT" + hit.collider.gameObject.name + " at " + hit.distance);

                if (hit.distance <= 10f)
                {
                    if (steering == -1)
                        steering = 0;
                    else steering = 1;
                }
                //Debug.Log(terrain_manager.myInfo.traversability[2, 2]);
            }

            m_Car.Move(steering, acceleration * slow_down_rate, acceleration * slow_down_rate, 0f);
            if (inRange(rigidbody.position, target_position, 2.5f))
            {
                node_index++;
                Debug.Log("Node " + (node_index - 1) + " reached");
            }

            if (stuck_safe_exit == 0)
                stuck_safe_exit = max_stuck_safe_exit;
        }
        else { Debug.Log("OH NO"); }
    }
    */


    /*


    private void MoveDubins()
        {
            //We need some information to use Dubins paths, start_pos, next position in the path, and headings.
            Vector3 start;
            Vector3 end;
            start = transform.position;
            float startHeading = transform.eulerAngles.y * Mathf.Deg2Rad;
            float endHeading;
            float turnLeft = m_Car.m_MaximumSteerAngle * -1f;
            float turnRight = m_Car.m_MaximumSteerAngle;

            foreach (Node n in graph.path)
            {
                if (n != null)
                {
                    //update the new ending position

                    end.x = n.x_pos;
                    end.y = start.y;
                    end.z = n.z_pos;
                    endHeading = n.heading;

                    List<DubinsPath> pathList = dubinsPathGenerator.makeManyDubinsPaths(
                        start,
                        startHeading,
                        end,
                        endHeading);

                    if (pathList.Count > 0)
                    {
                        foreach (DubinsPath path in pathList)
                        {
                            Debug.Log("Current path type: " + path.pathType);
                            switch (path.pathType)
                            {
                                case GenerateDrivingDirections.PathType.LRL:
                                    m_Car.Move(turnLeft, 1f, 1f, 0f);
                                    m_Car.Move(turnRight, 1f, 1f, 0f);
                                    m_Car.Move(turnLeft, 1f, 1f, 0f);
                                    break;
                                case GenerateDrivingDirections.PathType.RLR:
                                    m_Car.Move(turnRight, 1f, 1f, 0f);
                                    m_Car.Move(turnLeft, 1f, 1f, 0f);
                                    m_Car.Move(turnRight, 1f, 1f, 0f);
                                    break;
                                case GenerateDrivingDirections.PathType.LSR:
                                    m_Car.Move(turnLeft, 1f, 1f, 0f);
                                    m_Car.Move(0f, 1f, 1f, 0f);
                                    m_Car.Move(turnRight, 1f, 1f, 0f);
                                    break;
                                case GenerateDrivingDirections.PathType.LSL:
                                    m_Car.Move(turnLeft, 1f, 1f, 0f);
                                    m_Car.Move(0f, 1f, 1f, 0f);
                                    m_Car.Move(turnLeft, 1f, 1f, 0f);
                                    break;
                                case GenerateDrivingDirections.PathType.RSL:
                                    m_Car.Move(turnRight, 1f, 1f, 0f);
                                    m_Car.Move(0f, 1f, 1f, 0f);
                                    m_Car.Move(turnLeft, 1f, 1f, 0f);
                                    break;
                                case GenerateDrivingDirections.PathType.RSR:
                                    m_Car.Move(turnRight, 1f, 1f, 0f);
                                    m_Car.Move(0f, 1f, 1f, 0f);
                                    m_Car.Move(turnRight, 1f, 1f, 0f);
                                    break;
                            }

                        }

                    }
                    start = end;

                }
            }
        }
      
     
     */


}
