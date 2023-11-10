#! /usr/bin/env python3
import rospy
import numpy as np
from visualization_msgs.msg import Marker
from nav_msgs.msg import OccupancyGrid
from map_msgs.msg import OccupancyGridUpdate
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from practical_motion_msgs.srv import LoadList, GetNextWaypoint
from tf import TransformListener

class VFH:

    def __init__(self):

        #Get parameters

        #VFH weights
        self.wg = rospy.get_param('/vfh/wg',10)
        self.wo = rospy.get_param('/vfh/wo',7)
        self.wd = rospy.get_param('/vfh/wd',1)
        
        self.goal_des = ""

        #Lookahead distance
        self.LA = rospy.get_param('/vfh/LA',15)
        #VFH critical cost
        self.crit_cost = rospy.get_param('/vfh/crit_cost',60)
        #Costmap size
        self.l = rospy.get_param('/lane_costmap/costmap/width', 4)
        #Get local costmap resolution
        self.ds = rospy.get_param('/lane_costmap/costmap/resolution', 0.1)

        #Number of grid cells along one edge
        self.ncm = int(round(self.l/self.ds))

        # print(self.ncm)

        #Angle discretization
        self.ndpsi = rospy.get_param('/vfh/ndpsi',72)

        #Get goal
        self.xG = np.array([42,-36])
        self.psiG = np.mod(2*np.pi - np.pi/2,2*np.pi)

        #Get initial position. Get this from some topic.
        # self.xR0 = np.array([-2.3,0.5])
        self.xR0 = self.xG

        #Initialize previous waypoint position and pose
        self.xyvfh0 = self.xR0

        # #Define subscriber to odom
        # self.odom_sub = rospy.Subscriber("/odom",Odometry,self.robot_pose)
        # #Define flag
        # self.fodom = False

        #Define subscriber to local_costmap data
        # self.local_costmap_sub = rospy.Subscriber("/costmap_2d/costmap/costmap_updates",
        # OccupancyGridUpdate,self.local_costmap)

        self.local_costmap_sub = rospy.Subscriber("/composite_costmap/costmap/costmap", OccupancyGrid, self.local_costmap)

        #Define flag for receiving costmap
        self.fcost = False
        
        #Define flag for receiving global waypoint
        self.fgwp = False

        #Define publisher for VFH waypont
        # self.vfh_pub = rospy.Publisher("/vfh/waypoint",PoseStamped,queue_size=2)
        self.vfh_pub = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size=2)

        #Define publisher for VFH waypoint marker
        self.marker_pub = rospy.Publisher("/vfh_marker",Marker,queue_size=2)

        #Define publisher for goal marker
        self.goal_pub = rospy.Publisher("/goal_marker",Marker,queue_size=2)
        
        #Initialize global waypoint
        self.temp_pose = PoseStamped()
        
        self.listener = TransformListener()
        rospy.wait_for_service('waypoint_server/load_list')
        rospy.wait_for_service('waypoint_server/get_next_waypoint')
        # print('here')
        self.loadWptListClient = rospy.ServiceProxy('waypoint_server/load_list', LoadList)
        self.getNextWptClient = rospy.ServiceProxy('waypoint_server/get_next_waypoint', GetNextWaypoint)
        # self.file = 'comp_course_S_intermediate.txt'
        self.file = 'comp_course_N_intermediate.txt'
        # self.file = 'practice_course.txt'
        print(self.file)
        numwpts = self.loadWptListClient(self.file)
        print(numwpts)
        self.getGoal()
        
    def getGoal(self):
        
        while not rospy.is_shutdown():
            try:
                now = rospy.Time.now()
                self.listener.waitForTransform("/utm", "/base_link", now, rospy.Duration(1.0))
                break
            except:
                print('no tf found')
                
        nextWpt = self.getNextWptClient()
        
        self.goal_des = nextWpt.wpt.description
        # print(nextWpt)
        # self.xG = np.array([nextWpt.wpt.x, nextWpt.wpt.y])
        self.temp_pose.header.frame_id = nextWpt.wpt.frame_id
        self.temp_pose.pose.position.x = nextWpt.wpt.x
        self.temp_pose.pose.position.y = nextWpt.wpt.y
        self.temp_pose.pose.position.z = 0
        
        q = quaternion_from_euler(0.0, 0.0, nextWpt.wpt.theta)
        self.temp_pose.pose.orientation.x = q[0]
        self.temp_pose.pose.orientation.y = q[1]
        self.temp_pose.pose.orientation.z = q[2]
        self.temp_pose.pose.orientation.w = q[3]
        print(self.temp_pose)
        
        self.transformGoal(self.temp_pose)
        
    def transformGoal(self, pose):
        # print('transforming goal')
        new_pose = self.listener.transformPose('base_link', pose)
        print(new_pose)
        
        self.xG = np.array([new_pose.pose.position.x, new_pose.pose.position.y])
        quat = (
            new_pose.pose.orientation.x,
            new_pose.pose.orientation.y,
            new_pose.pose.orientation.z,
            new_pose.pose.orientation.w)
        RPY = euler_from_quaternion(quat)
        self.psiG = RPY[2]
        
        #Set global goal flag to true
        self.fgwp = True
        

    # #Receive robot pose
    # def robot_pose(self,pose_msg):
    #     self.X = pose_msg.pose.pose.position.x
    #     self.Y = pose_msg.pose.pose.position.y
    #     psiz = pose_msg.pose.pose.orientation.z
    #     psiw = pose_msg.pose.pose.orientation.w
    #     self.psi = np.mod(2*np.pi + 2*np.arctan2((psiz),(0.001+psiw)),2*np.pi)

    #     #Set flag to true
    #     self.fodom = True


    #Recieve local costmap data
    #local_costmap is postioned in /odom frame
    def local_costmap(self,costmap_msg):
        self.local_cost = costmap_msg.data

        #Set flag to true
        self.fcost = True

        if self.fgwp:
            self.pub_VFHwp()

    #Determine VFH waypoint
    def pub_VFHwp(self):
        
        # print('Execute VFH')

        #Collect costmap
        ODcp = np.array(self.local_cost)

        #Collect critical cost
        crit_cost = self.crit_cost

        #Collect robot position and pose
        # xyR = np.array([self.X,self.Y])
        
        #Collect robot position in base_link
        xyR = np.array([0, 0])
        psiR = 0

        #Collect GPS goal position and pose in base_link
        #Use utm-->base_link
        xG = self.xG
        psiG = self.psiG

        #Collect grid cell size
        ds = self.ds
        #Number of grid cells along one edge
        ncm = self.ncm
        #Collect lookahead distance
        LA = self.LA

        #VFH weights
        wg = self.wg
        wd = self.wd
        wo = self.wo

        #Discretize heading angles
        ndpsi = self.ndpsi
        vfhpsi = np.linspace(0,2*np.pi,ndpsi)

        #Initialize candidate waypoint location and pose
        xyvfh = np.array([0,0])
        psivfh = 0

        #Initialize debuggers
        THETAG = 0
        THETAWP = 0
        THETAWP0 = 0
        CS = 0
        RS = 0
        CWP = 0
        Ind = 0

        #Initialize VFH cost
        Jvfh = 10000

        #Discretize forward motion
        #Distance to goal in terms of grid cells
        d2G = int(np.linalg.norm(xyR-xG)/ds)

        #Check if goal is further than lookahead distance
        if d2G>LA:
            
            # print('Goal is further than lookahead distance')

            npts = np.arange(3,LA,dtype=int)

            #This theta is in base_link
            for theta in vfhpsi:
                
                # print('Looking at candidate headings')
                
                #Get indices in 1-D costmap array
                #Get relative xy positions of each cell in world frame
                #in grid cell count
                
                #Column shift
                cs = npts*np.cos(theta)
                #Row shift
                rs = npts*np.sin(theta)

                #Indices
                csg = np.ceil(ncm//2 + cs)
                rsg = np.floor(ncm//2 + rs)

                I = np.int_(rsg*ncm + csg - 1)

                #Retrieve costs at these indices
                cJ = ODcp[I]


                #Check if all costs are below critical cost lJ
                if np.all(cJ<crit_cost):
                    
                    print('All costs are below threshold')

                    #Jump to furthest grid cell along direction theta
                    #Candidate waypoint xy coordinates in base_link
                    cwx = xyR[0] + ds*npts[-1]*np.cos(theta)
                    cwy = xyR[1] + ds*npts[-1]*np.sin(theta)
                    
                    #Robot heading to goal in base_link
                    thetag = np.mod(2*np.pi + np.arctan2((xG[1]-xyR[1]),(xG[0]-xyR[0]+0.001)), 2*np.pi)
                    #Robot heading to candidate waypoint
                    thetawp = np.mod(2*np.pi + np.arctan2((cwy-xyR[1]),(cwx-xyR[0]+0.001)), 2*np.pi) 
                    #thetawp = theta
                    #Robot heading to previous waypoint
                    thetawp0 = np.mod(2*np.pi + np.arctan2((self.xyvfh0[1]-xyR[1]),(self.xyvfh0[0]-xyR[0]+0.001)), 2*np.pi)
                    
                    #Calculate VFH cost at the candidate waypoint
                    # a = targetA - sourceA
                    # a = (a + 180) % 360 - 180

                    sd = np.array([thetag,thetawp0,psiR])
                    sd = sd - thetawp
                    sd = np.abs(np.mod((sd + np.pi),2*np.pi) - np.pi)

                    J1 = wg*sd[0]
                    J2 = wd*sd[1]
                    J3 = wo*sd[2]

                    J = J1 + J2 + J3

                    # print('Cand. VFH waypoint: ',np.array([cwx,cwy]))
                    # print('Cand. VFH waypoint pose: ',thetawp)
                    # print('Robot to Goal pose: ',thetag)
                    # print('Cand. VFH cost wg: ',J1)
                    # print('Cand. VFH cost wd: ',J2)
                    # print('Cand. VFH cost wo: ',J3)
                    # print('Cand. VFH cost: ',J)
                    # print('Cand. costmap cost: ', cJ[-1])

                    #Compare candidate waypoint cost and current minimum
                    if J<=Jvfh:
                        
                        print('Update VFH waypoint')
                        
                        #Update candidate waypoint and min VFH cost
                        xyvfh = np.array([cwx,cwy])
                        psivfh = thetawp
                        Jvfh = J

                        #Debuggers
                        # THETAWP = thetawp
                        # THETAG = thetag
                        # THETAWP0 = thetawp0
                        CWP = cJ
                        # Ind = I
                        # CS = cs
                        # RS = rs
            self.transformGoal(self.temp_pose)

        else:
            #Within LA
            xyvfh = np.array(xG)
            psivfh = psiG
            # print('Cand. VFH waypoint: ',xyvfh)
            # print('Cand. VFH waypoint pose: ',psivfh)
            # THETAWP = thetawp
            # THETAG = thetag
            # THETAWP0 = thetawp0
            
            #Call server for next goal
            self.getGoal()

        #Update previous waypoint
        self.xyvfh0 = xyvfh

        #Print waypoint coordinate and cost
        print('VFH waypoint: ',xyvfh)
        print('VFH waypoint pose: ',psivfh)
        print('Robot costmap cost: ',ODcp[(ncm-ncm//2-1)*ncm + ncm//2])
        # print('VFH wp costmap cost: ',CWP)
        # print('VFH waypoint indices',Ind)
        # print('THETAWP: ',THETAWP)
        # print('THETAG: ',THETAG)
        # print('THETAWP0: ',THETAWP0)
        # print('psi: ',psiR)

        #Publish VFH paypoint
        wp = PoseStamped()
        wp.header.frame_id = "base_link"
        wp.pose.position.x = xyvfh[0]
        wp.pose.position.y = xyvfh[1]
        wp.pose.position.z = 0

        #Convert to quaternion angle
        q = quaternion_from_euler(0.0, 0.0, psivfh)

        wp.pose.orientation.x = q[0]
        wp.pose.orientation.y = q[1]
        wp.pose.orientation.z = q[2]
        wp.pose.orientation.w = q[3]

        if self.goal_des == "wpt1":
            wp.header.frame_id = "base_link"
            wp.pose.position.x = self.xG[0]
            wp.pose.position.y = self.xG[1]
            wp.pose.position.z = 0

            #Convert to quaternion angle
            q = quaternion_from_euler(0.0, 0.0, self.psiG)

            wp.pose.orientation.x = q[0]
            wp.pose.orientation.y = q[1]
            wp.pose.orientation.z = q[2]
            wp.pose.orientation.w = q[3]

            self.vfh_pub.publish(wp)
        else:
            self.vfh_pub.publish(wp)

        #Send VFH waypoint to a blue marker publisher
        self.pub_marker(xyvfh[0],xyvfh[1],psivfh)
        #Send goal to a green goal publisher
        self.goal_marker(xG[0],xG[1],psiG)

        #Reset flags
        self.fcost = False
        self.fgwp = False
        
        self.transformGoal(self.temp_pose)

    #A function that publishes a marker at the VFH waypoint
    def pub_marker(self,wpx,wpy,wppsi):

        #Define mark
        marker = Marker()

        marker.header.frame_id = "base_link"
        marker.header.stamp = rospy.Time.now()

        marker.type = 0
        marker.id = 0

        marker.scale.x = 0.3
        marker.scale.y = 0.05
        marker.scale.z = 0.05

        marker.color.r = 1
        marker.color.g = 1
        marker.color.b = 0
        marker.color.a = 1

        psiR = wppsi

        marker.pose.position.x = wpx
        marker.pose.position.y = wpy
        marker.pose.position.z = 0.1
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = np.sin(psiR/2)
        marker.pose.orientation.w = np.cos(psiR/2)

        #Publish marker
        self.marker_pub.publish(marker)


    #A function that publishes a marker at the goal
    def goal_marker(self,wpx,wpy,wppsi):

        #Define marker
        marker = Marker()

        marker.header.frame_id = "base_link"
        marker.header.stamp = rospy.Time.now()

        marker.type = 0
        marker.id = 0

        marker.scale.x = 0.3
        marker.scale.y = 0.05
        marker.scale.z = 0.05

        marker.color.r = 0
        marker.color.g = 1
        marker.color.b = 0
        marker.color.a = 1

        psiR = wppsi

        marker.pose.position.x = wpx
        marker.pose.position.y = wpy
        marker.pose.position.z = 0.2
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = np.sin(psiR/2)
        marker.pose.orientation.w = np.cos(psiR/2)

        #Publish marker
        self.goal_pub.publish(marker)



if __name__ == '__main__':
    #Initialize node
    rospy.init_node('vfh_baselink',anonymous=True)

    try:

        VFH()
        rospy.spin()

    except rospy.ROSInterruptException:
        pass
