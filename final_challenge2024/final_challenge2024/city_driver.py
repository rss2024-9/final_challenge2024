import rclpy
from rclpy.node import Node


import numpy as np
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped, PoseArray,PointStamped
import time
from .utils import LineTrajectory


class CityDriver(Node):
    def __init__(self):
        super().__init__("city_driver")
        self.publisher = None #TODO
        self.localize_sub = self.create_subscription(Odometry,"/pf/pose/odom",self.localize_cb,10)
        self.curr_pose = None
        self.goal_sub = self.create_subscription(
            PoseArray,
            "/shell_points",
            self.goal_cb,
            10
        ) #change this to accept whatever input they give use
        self.goals=[]
        self.end_point=None
        self.state = None

        self.traj_pub = self.create_publisher(
            PoseArray,
            "/trajectory/current",
            10
        )

        self.traj_sub = self.create_subscription(PoseArray,"/trajectory/store", self.trajectory_callback, 1)

        self.wait_time= 0
        self.max_wait = 5 #wait for shell to be placed

        self.initial_pub = self.create_publisher(PoseWithCovarianceStamped,
            "/initialpose",
            10
        )
        self.goal_pub = self.create_publisher(
            PoseStamped,
            "/goal_pose",
            10
        )

        self.trajectory = LineTrajectory("/followed_trajectory")
        self.traj_pub = self.create_publisher(
            PoseArray,
            "/trajectory/current",
            10
        )
        self.stop_sub = self.create_subscription(PoseArray,"/stop_stupid",self.stop_cb,1)

        self.viz_timer = self.create_timer(1/5, self.motion_cb)
        self.path = None
        self.pub_start = True
        self.stopping_dist = 0.3 #m
        
        

        self.get_logger().info("Driver Initialized")

    def is_close(self,start_point,end_point):
        x1,y1= start_point.pose.pose.position.x,start_point.pose.pose.position.y
        car_xy_pos = np.array((x1,y1))
        x2,y2 = end_point.pose.position.x,end_point.pose.position.y
        p2 = np.array((x2,y2))
        return np.linalg.norm(car_xy_pos-p2) <= self.stopping_dist
    
    def stop_cb(self,msg):
        self.state = "wait"
        curr_time = time.time()
        self.wait_time = curr_time


    def localize_cb(self, odom_msg):
        self.curr_pose = odom_msg

        # #if at goal, wait for shell to be placed
        # if (self.curr_pose and self.end_point) and self.is_close(self.curr_pose,self.end_point):
            
        #     if self.wait_time == 0:
        #         self.state = "wait"
        #         curr_time = time.time()
        #         self.wait_time = curr_time


    def trajectory_callback(self,msg):
        #self.get_logger().info("in traj cb")
        self.path = msg
        #publish path immediately if starting
        if self.pub_start:
            self.pub_start=False
            self.publish_path()
    


    def goal_cb(self,msg):
        """store goal points to send to path planner"""
        #add start point to end of goal points
    
        # start1 = PoseWithCovarianceStamped()
        # end1 = PoseStamped()
        # end4 = PoseStamped()

        # end4.pose.position.x = 24.0  
        # end4.pose.position.y = -1.0
        # end4.pose.orientation.z = -1.0
        # end4.pose.orientation.w = 0.0

        

        # start1.pose.pose.position.x = 24.0  
        # start1.pose.pose.position.y = -1.0
        # start1.pose.pose.orientation.z = -1.0
        # start1.pose.pose.orientation.w = 0.0
        # self.initial_pub.publish(start1)
        # time.sleep(2)
        # self.curr_pose = start1

        # end1.pose.position.x = 12.07  
        # end1.pose.position.y = -0.75  
        # end1.pose.orientation.z = 1.0
        # end1.pose.orientation.w = 0.0

        # end3 = PoseStamped()

        # end3.pose.position.x = -19.755
        # end3.pose.position.y = 3.188
        # end3.pose.orientation.z = 0.666
        # end3.pose.orientation.w = 0.746

        # end2 = PoseStamped()

        # end2.pose.position.x = 12.07  
        # end2.pose.position.y = -0.75  
        # end2.pose.orientation.z = -1.0
        # end2.pose.orientation.w = 0.0


        

        # self.goals.extend([end1,end3,end2,end4])

        # self.state="start"
        # self.get_logger().info("added goals and changed position")

        ##All of the above is for tests

        #!TODO take in PoseArray of goal points and put them in instance variable, also add start point
        #self.get_logger().info("made it in here")
        temp=[]
        for point in msg.poses:
            new_pose = PoseStamped()
            new_pose.pose.position.x = point.position.x
            new_pose.pose.position.y = point.position.y
            new_pose.pose.position.z = point.position.z

            temp.append(new_pose)
        start = self.curr_pose
        return_point = PoseStamped()
        start = self.curr_pose
        return_point.pose.position.x = start.pose.pose.position.x
        return_point.pose.position.y = start.pose.pose.position.y
        return_point.pose.orientation.z = start.pose.pose.orientation.z
        return_point.pose.orientation.w = start.pose.pose.orientation.w
        temp.append(return_point)
        self.goals=temp
        #self.get_logger().info("got goal points, starting state machine")
        self.state="start"

    def path_plan(self,goal):
        """
        Creates a path from current position to goal position,probably want to move all path planning inside this node for ease
        """
        #Modify path planning so it does the following:

        #get path from start position to nearest point on trajectory

        #get path from nearest point on trajectory to goal position
        
        #string together start ->traj -> end traj -> goal paths

        #return entire path

        #If path plan does the above stuff, all we need is this next line of code

        self.goal_pub.publish(goal)

        

        
    def publish_path(self):
        """
        Publish trajectory to follower
        """

        #publish path so car can follow it
        self.traj_pub.publish(self.path)
        #cleare path variable just in case
        self.path=None

    def send_goal(self,pose):
        msg = PoseArray()
        msg.poses.append(pose)
        self.traj_pub.publish(msg)

    def motion_cb(self):
        """
        State machine main function. while there are goals to get to move through start state, to drive, 
        to waiting(letting shell be place), to driving until you reach end of goals
        """

        if self.goals:
            
            if self.state == "start" :
                #self.get_logger().info("went through start")
                self.state = "drive"
                self.end_point = self.goals.pop(0)
                self.send_goal(self.end_point)
                #self.publish_path()
                
            elif self.state == "drive":
                #self.get_logger().info("driving")
                pass 
            # elif self.state == "wait_start":
            #     #self.get_logger().info("in wait start")
            #     self.end_point = self.goals.pop(0)
            #     self.path_plan(self.end_point)
            #     self.state="wait"
            elif self.state == "wait":
                #self.get_logger().info("in wait")
                curr_time = time.time()
                if (curr_time - self.wait_time ) >=self.max_wait: #and self.path:
                    self.end_point = self.goals.pop(0)
                    self.send_goal(self.end_point)
                    self.state = "drive"
                    self.wait_time = 0
            else:
                pass
        
        # elif not self.goals and self.path is not None:
        #     #wait to publish return to start path until after last shell is placed
        #     curr_time = time.time()
        #     if (curr_time - self.wait_time ) >=self.max_wait and self.path:
        #         self.publish_path()
        #         #self.get_logger().info("returning to start")
        


        







       

def main(args=None):
    rclpy.init(args=args)
    try:
        rclpy.spin(CityDriver())
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()

if __name__=="__main__":
    main()