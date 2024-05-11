import rclpy
from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import PoseArray,Pose

from rclpy.node import Node
from nav_msgs.msg import Odometry
from tf_transformations import euler_from_quaternion, quaternion_from_euler
import numpy as np
import math
import time

from .utils import LineTrajectory


class PurePursuit(Node):
    """ Implements Pure Pursuit trajectory tracking with a fixed lookahead and speed.
    """

    def __init__(self):
        super().__init__("trajectory_follower")
        self.declare_parameter('odom_topic', "default")
        self.declare_parameter('drive_topic', "default")

        self.odom_topic = self.get_parameter('odom_topic').get_parameter_value().string_value
        self.drive_topic = self.get_parameter('drive_topic').get_parameter_value().string_value

        self.default_lookahead = 0.9  # FILL IN #
        self.lookahead=self.default_lookahead
        self.get_logger().info(f'{self.lookahead}')
        self.speed = 2.  # FILL IN #
        self.wheelbase_length = 0.3  # FILL IN #

        self.trajectory = LineTrajectory("/followed_trajectory")

        lane_line = {"points": [{"x": -19.99921417236328, "y": 1.3358267545700073, "z" : 0.0}, {"x": -18.433984756469727, "y": 7.590575218200684, "z" : 0.0}, {"x": -15.413466453552246, "y": 10.617328643798828, "z" : 0.0}, {"x": -6.186201572418213, "y": 21.114534378051758, "z" : 0.0}, {"x": -5.5363922119140625, "y": 25.662315368652344, "z" : 0.0}, {"x": -19.717021942138672, "y": 25.677358627319336, "z" : 0.0}, {"x": -20.30797004699707, "y": 26.20694923400879, "z" : 0.0}, {"x": -20.441822052001953, "y": 33.974945068359375, "z" : 0.0}, {"x": -55.0716438293457, "y": 34.07769775390625, "z" : 0.0}, {"x": -55.30067825317383, "y": 1.4463690519332886, "z" : 0.0}]}
        self.lane_line=[]
        for point in lane_line["points"]:
            x=point["x"]
            y=point["y"]
            z=point["z"]
            self.lane_line.append((x,y,z))
        

        self.traj_sub = self.create_subscription(PoseArray,"/trajectory/current", self.trajectory_callback, 1)
        
        #this is so that the thing will always visualize
        self.viz_sub = self.create_subscription(PoseArray,"loaded_trajectory/path", self.dummy_callback, 1)

        self.drive_pub = self.create_publisher(AckermannDriveStamped,
                                               self.drive_topic,
                                               1)
        
        
        #subscribe to particle filter localization #turn back on for real car
        #real
        #self.pose_sub = self.create_subscription(Odometry,"/pf/pose/odom",self.pose_callback, 1)
        #sim
        self.pose_sub = self.create_subscription(Odometry,"/odom",self.pose_callback, 1)   

        #viz target point
        self.viz_pub = self.create_publisher(PoseArray, "/target_point", 1) 

        #viz p1 point
        self.viz_pubp1 = self.create_publisher(PoseArray, "/p1", 1)

        #viz p2 point
        self.viz_pubp2 = self.create_publisher(PoseArray, "/p2", 1)

        self.stop_pub = self.create_publisher(PoseArray, "/stop_stupid", 1) 

        #offset from left line
        self.offset = 0.3175
        

        #flip counter
        self.flip_counter = 1

        #on path counter
        self.on_planned_path = 1

        self.initialized_traj=False

        self.stop_msg = PoseArray()



    def pose_callback(self, odometry_msg):
        car_x = odometry_msg.pose.pose.position.x
        car_y = odometry_msg.pose.pose.position.y
        car_z = odometry_msg.pose.pose.position.z
        
        car_xy_pos = np.array((car_x,car_y))

        
        



        #if there is no trajectory loaded wait
        #self.get_logger().info(f"{np.array(self.trajectory.points)}")
        if not self.initialized_traj:#np.array(self.trajectory.points).size == 0:
            self.get_logger().info(f"waiting for trajectory")
            drive_msg = AckermannDriveStamped()
            drive_msg.drive.speed = 0.0 
            drive_msg.drive.steering_angle = 0.0
            self.drive_pub.publish(drive_msg)
            self.t0 = time.time()

            #this stuff is just to help me with testing
            #self.get_logger().info(f'{car_xy_pos}')
            return

        raw_traj_points = np.array(self.lane_line)
        ##################################################################
        #merge parallel segments let carlos do this
        ##################################################################
        def path_collapse(path):
            """

            Args:
                path (list[ndarray[float, float]]): List of points connected by line segments defining a path
            """
            # Initialize with the first two points
            path = list(path)
            out = path[:2]

            for pt in path[2:]:
                # Compute the direciton vector of the path constructed so far, versus the
                # direction vector to the current point
                lastd = out[-1] - out[-2]
                thisd = pt - out[-1]

                # Normalize the direction vectors
                lastd /= np.abs(lastd)
                thisd /= np.abs(thisd)

                # If roughly straight, skip the last point in the path constructed so far and
                # go straight to the current point.
                if np.dot(lastd, thisd) >= 0.95:
                    out[-1] = pt
                else:
                    out.append(pt)

            return np.array(out)
        
        merged_traj_points = path_collapse(raw_traj_points)
        ##################################################################
        ##################################################################
        #find the segment that is nearest to the car
        #traj_points = np.array(self.trajectory.points)[:,:2]
        traj_points = merged_traj_points[:,:2]

        #traj_points_flags = np.array(self.trajectory.points)[:,-1]
        traj_points_flags = merged_traj_points[:,-1]


        
        
        #if the car is going backwards flip the trajectory
        if self.flip_counter == -1:
            traj_points = traj_points[::-1]
            traj_points_flags = traj_points_flags[::-1]
            



        start_pts = traj_points[:-1,:]
        end_pts = traj_points[1:,:]
        segs = np.empty((start_pts.shape[0],4))
        segs[:,:2] = start_pts
        segs[:,2:] = end_pts

        deltas = end_pts-start_pts

        traj_norms = np.zeros((segs.shape[0],2))
        traj_norms[:,0] = deltas[:,1]
        traj_norms[:,1] = -deltas[:,0]

        traj_norms_sqd = traj_norms**2
        sum_sqd_traj_norms = np.abs(np.sum(traj_norms_sqd,axis=1))
        traj_norms = traj_norms/np.sqrt(sum_sqd_traj_norms[:, np.newaxis])

        #get segment flags
        start_flags = traj_points_flags[:-1]
        end_flags = traj_points_flags[1:]

        seg_offsets = np.zeros(segs.shape[0])

        seg_offsets += np.where((start_flags == 0.) & (end_flags == 0.), self.offset, 0.)
        seg_offsets += np.where((start_flags == 1.) & (end_flags == 0.), self.offset, 0.)
        seg_offsets += np.where((start_flags == 0.) & (end_flags == 1.), self.offset,0)

        segs[:,:2]  += seg_offsets[:,np.newaxis]*traj_norms
        segs[:,2:]  += seg_offsets[:,np.newaxis]*traj_norms

        #solve for the paramter to make the lines int
        ac = segs[1:,:2]-segs[:-1,:2]
        ac = ac.flatten()
        ab = segs[:-1,2:]-segs[:-1,:2]
        cd = segs[1:,2:]-segs[1:,:2]

        A = np.zeros((2*(segs.shape[0]-1),2*(segs.shape[0]-1)))
        for i in range(0,A.shape[0],2):
            A[i:i+2,i] = ab[int(i/2),:]
            A[i:i+2,i+1] = cd[int(i/2),:]

        t = np.linalg.solve(A,ac.T)

        new_points = np.zeros((segs.shape[0]+1,2))
        start_points=segs[:,:2]
        end_points = segs[:,2:]

        new_points[0,:]=start_points[0,:]
        new_points[-1,:]=end_points[-1,:]

        t = t.reshape(-1, 1)
        new_points[1:-1,:]=start_points[:-1]+ab[:,:]*t[::2]
        traj_points = new_points

        
        #self.get_logger().info(f'trajpts {traj_points}')
        N=traj_points.shape[0]
        nrst_distances = self.find_dist(traj_points[0:N-1,:],traj_points[1:N,:],car_xy_pos)
        min_index = np.argmin(nrst_distances)
        nearest_segment = traj_points[min_index:(min_index+2),:]

        #####################################
        #do logic with goal point
        ############################
        goal_point = self.trajectory.points
        N=goal_point.shape[0]
        nrst_distances_to_goal = self.find_dist(traj_points[0:N-1,:],traj_points[1:N,:],goal_point)
        min_index = np.argmin(nrst_distances_to_goal)
        nearest_segment_to_goal = traj_points[min_index:(min_index+2),:]

        #check if the goal is on the right
        nearest_vector_delta = nearest_segment_to_goal[1,:]-nearest_segment_to_goal[0,:]
        nearest_vector_norm = np.array([nearest_vector_delta[1],-nearest_vector_delta[0]])
        point_to_goal = goal_point-nearest_segment_to_goal[0,:]
        dot = np.dot(point_to_goal,nearest_vector_norm)
        if dot > 0:
            goal_on_right = True
        else:
            goal_on_right = False

        # check if the goal and the car have the same nearest segment
        if np.array_equal(nearest_segment,nearest_segment_to_goal):
            on_goal_segment = True
        else:
            on_goal_segment = False

        # check if the projetions are close
        goal_projection = np.dot(point_to_goal,nearest_vector_delta)/(np.linalg.norm(point_to_goal)*np.linalg.norm(nearest_vector_delta))
        point_to_car = car_xy_pos-nearest_segment[0,:]
        car_projection = np.dot(point_to_car,nearest_vector_delta)/(np.linalg.norm(point_to_car)*np.linalg.norm(nearest_vector_delta))
        if abs(goal_projection-car_projection)< 0.1:
            projection_close = True
        else:
            projection_close = False

        if goal_on_right and on_goal_segment and projection_close:
            drive_msg = AckermannDriveStamped()
            drive_msg.drive.speed = 0.0 
            drive_msg.drive.steering_angle = 0.0
            self.drive_pub.publish(drive_msg)
            self.stop_pub.publish(self.stop_msg)
            return


            
        

        #find the intersection point(s) of the segment with the circle
        #self.get_logger().info(f'nearest segment{nearest_segment}')
        p1 = nearest_segment[0,:]
        p2 = nearest_segment[1,:]

        #check to see if you are running the course backwards
        car_ort_x = odometry_msg.pose.pose.orientation.x
        car_ort_y = odometry_msg.pose.pose.orientation.y
        car_ort_z = odometry_msg.pose.pose.orientation.z
        car_ort_w = odometry_msg.pose.pose.orientation.w
        theta = euler_from_quaternion((car_ort_x, car_ort_y, car_ort_z, car_ort_w))[-1]
        car_x_direction = np.array([math.cos(theta),math.sin(theta)])
        car_y_direction = np.array([-math.sin(theta),math.cos(theta)])


        direction = p2-p1
        direction_parameter = np.dot(direction,np.array(car_x_direction)) # if positive do nothing
        if direction_parameter < 0:
            #if the car is going backwards we need to tell it to flip the path 
            #and repeat the process
            self.flip_counter*=-1
            return
            

        #find the centerline distance from the current segment for data
        centerline_distance = self.find_centerline_dist(p1,p2,car_xy_pos)-self.offset
        self.lookahead = max(np.min(nrst_distances),self.default_lookahead)
        # # Open a file in write mode
        # current_time = (time.time()-self.t0)
        # with open("centerline_data.txt", "a") as file:
        #     # Write each item from the data list to the file
        #     file.write(f"{centerline_distance},{current_time}\n")


    
        #if the car is so close to the end look to the next
        dist_from_end = np.linalg.norm(car_xy_pos-p2)
        i_counter = 0
        while (dist_from_end <= self.lookahead) and (min_index+i_counter+3) <= traj_points.shape[0]:
            nearest_segment = traj_points[(min_index+i_counter+1):(min_index+i_counter+3),:]
            p1 = nearest_segment[0,:]
            p2 = nearest_segment[1,:]
            dist_from_end = np.linalg.norm(car_xy_pos-p2)
            i_counter+=1

        #if the car reaches the end stop
        #self.get_logger().info(f'{dist_from_end}')
        if (dist_from_end <= 0.05) and np.all(p2 == traj_points[-1]):
            self.speed = self.speed
            steering_angle = 0.34
            drive_msg = AckermannDriveStamped()
            drive_msg.drive.speed = self.speed 
            drive_msg.drive.steering_angle = steering_angle
            self.drive_pub.publish(drive_msg)
            time.sleep(2.66/self.speed + .2)
            return  

        V = p2-p1
        r = self.lookahead
        Q = car_xy_pos

        a = np.dot(V,V)
        b = 2 * np.dot(V,(p1 - Q))
        c = np.dot(p1,p1) + np.dot(Q,Q) - 2 * np.dot(p1,Q) - r**2

        disc = b**2 - 4 * a * c
        if disc < 0: #no solution
            return None
        
        # if a solution exists find it
        sqrt_disc = math.sqrt(disc)
        t1 = (-b + sqrt_disc) / (2 * a)
        t2 = (-b - sqrt_disc) / (2 * a) #keep this in case we need

        #in the event that the line does intersect find the points
        int1 = p1 + t1*V
        int2 = p1 + t2*V
        
        #now pick the point that is in front of the car
        if not (0 <= t1 <= 1 or 0 <= t2 <= 1):
            #if the circle would only intersect if the line was extened make the end point be the goal
            target_point = p2
        else:
            #i think you just want to pick the one that is closer to the second point, ie the greatest t?
            #t1 should be greater, but it might be out of range! if it is out of range then use t2
            target_point = int1
        
        
        
        

        


        
        
        #now that we  have the target point we can do the pure pursuite algorithm
        #determine the heading of the car in the world frame

        #theta = 0.
        
        
        #calculate the position of the point relative to the car
        target_point_point_rel_to_car = car_xy_pos-target_point
        
        #in order to get the correct eta we need to broadcast these to the cars frame
        rel_x = np.dot(car_x_direction,target_point_point_rel_to_car)
        rel_y = np.dot(car_y_direction,target_point_point_rel_to_car)
        eta = math.atan(rel_y/rel_x)

        #calculate the steering angle  (right is negative left is postive)
        steering_angle = math.atan((2*self.wheelbase_length*math.sin(eta))/self.lookahead)
        steering_angle = np.clip(np.array([steering_angle]),-0.34,0.34)[0]


        #publish the drive command
        drive_msg = AckermannDriveStamped()
        drive_msg.drive.speed = self.speed 
        drive_msg.drive.steering_angle = steering_angle
        self.drive_pub.publish(drive_msg)




        # ###########################################################
        # #FOR RUNNING IN SIM: Transform all viz points back to world frame
        # ###########################################################
        
        # new_p1 = np.empty(2)
        # new_p2 = np.empty(2)
        # new_target = np.empty(2)

        # new_p1[0] = np.cos(theta)*p1[0]-np.sin(theta)*p1[1]
        # new_p1[1] = np.sin(theta)*p1[0]+np.cos(theta)*p1[1]
        # p1=new_p1

        # new_p2[0] = np.cos(theta)*p2[0]-np.sin(theta)*p2[1]
        # new_p2[1] = np.sin(theta)*p2[0]+np.cos(theta)*p2[1]
        # p2=new_p2

        # new_target[0] = np.cos(theta)*target_point[0]-np.sin(theta)*target_point[1]
        # new_target[1] = np.sin(theta)*target_point[0]+np.cos(theta)*target_point[1]
        # target_point=new_target

        # p1[0] = p1[0]+car_x_world
        # p1[1] = p1[1]+car_y_world

        # p2[0] = p2[0]+car_x_world
        # p2[1] = p2[1]+car_y_world

        # target_point[0] = target_point[0]+car_x_world
        # target_point[1] = target_point[1]+car_y_world
        
        # ######################################################################################################
        # #######################################################################################################

        #visualize the target_point
        from threading import Lock
        self.lock = Lock()
        with self.lock:
            msg = PoseArray()
            msg.header.frame_id = "map"
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.poses.append(PurePursuit.pose_to_msg(target_point[0], target_point[1], 0.0))
        self.viz_pub.publish(msg)

         #visualize the p1
        from threading import Lock
        self.lock = Lock()
        with self.lock:
            msg = PoseArray()
            msg.header.frame_id = "map"
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.poses.append(PurePursuit.pose_to_msg(p1[0], p1[1], 0.0))
        self.viz_pubp1.publish(msg)

         #visualize the target_point
        from threading import Lock
        self.lock = Lock()
        with self.lock:
            msg = PoseArray()
            msg.header.frame_id = "map"
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.poses.append(PurePursuit.pose_to_msg(p2[0], p2[1], 0.0))
        self.viz_pubp2.publish(msg)



    def trajectory_callback(self, msg):
        self.get_logger().info(f"Receiving new trajectory {len(msg.poses)} points")

        self.trajectory.clear()
        self.trajectory.fromPoseArray(msg)
        self.trajectory.publish_viz(duration=0.0)
        self.initialized_traj = True
    
    def dummy_callback(self,msg):
        return

    def find_dist(self, linepoint1, linepoint2, point):
        linepoint1 = np.array(linepoint1)
        linepoint2 = np.array(linepoint2)
        point = np.array(point)
        delta = linepoint2-linepoint1
        norm_vec = np.empty(delta.shape)
        norm_vec[:,0] = -delta[:,1]
        norm_vec[:,1] = delta[:,0]
        qp = point-linepoint1
        norm_mag = np.sum((norm_vec*norm_vec),axis = 1)**.5
        # Parameter t for the projection of the point onto the line segment
        t = np.sum(qp*delta,axis=1)/np.sum(delta*delta,axis=1)
        #this is essentially doing the dot product, gives shortest distance to infinite line
        d = np.abs(np.sum(norm_vec*qp,axis=1))/norm_mag 
        #the edge case
        d = np.where(t>=1, np.linalg.norm(linepoint2-point, axis=1),d)
        d = np.where(t<=0, np.linalg.norm(linepoint1-point, axis=1),d)
        #self.get_logger().info(f'd: {d}')
        return d
    
    def find_centerline_dist(self, linepoint1, linepoint2, point):
        linepoint1 = np.array(linepoint1)
        linepoint2 = np.array(linepoint2)
        point = np.array(point)
        delta = linepoint2-linepoint1
        norm_vec = np.empty(delta.shape)
        norm_vec[0] = -delta[1]
        norm_vec[1] = delta[0]
        qp = point-linepoint1
        norm_mag = np.sum((norm_vec*norm_vec))**.5
        #this is essentially doing the dot product, gives shortest distance to infinite line
        d = (np.sum(norm_vec*qp))/norm_mag 
        return d
        
    @staticmethod
    def pose_to_msg(x, y, theta):
        msg = Pose()
        msg.position.x = float(x)
        msg.position.y = float(y)
        msg.position.z = 0.0
        
        quaternion = quaternion_from_euler(0.0, 0.0, theta)
        msg.orientation.x = quaternion[0]
        msg.orientation.y = quaternion[1]
        msg.orientation.z = quaternion[2]
        msg.orientation.w = quaternion[3]

        return msg


def main(args=None):
    rclpy.init(args=args)
    follower = PurePursuit()
    rclpy.spin(follower)
    rclpy.shutdown()
