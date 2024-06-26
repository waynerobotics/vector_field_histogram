#! /usr/bin/env python3
import rospy
import numpy as np
from visualization_msgs.msg import Marker
from nav_msgs.msg import OccupancyGrid
from map_msgs.msg import OccupancyGridUpdate
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan,Imu
from geometry_msgs.msg import PoseStamped, PointStamped
from tf.transformations import quaternion_from_euler
import tf2_ros
from tf2_geometry_msgs import do_transform_point

class VFH:

    def __init__(self):

        #Get parameters

        #VFH weights
        self.wg = rospy.get_param('/vfh/wg',10)
        self.wo = rospy.get_param('/vfh/wo',7)
        self.wd = rospy.get_param('/vfh/wd',1)

        #Lookahead distance
        self.LA = rospy.get_param('/vfh/LA',15)
        #VFH critical cost
        self.crit_cost = rospy.get_param('/vfh/crit_cost',60)
        #Costmap size
        self.l = rospy.get_param('/costmap_2d/costmap/width',8)
        #Get local costmap resolution
        self.ds = rospy.get_param('/costmap_2d/costmap/resolution',0.1)

        #Number of grid cells along one edge
        self.ncm = int(round(self.l/self.ds))

        # print(self.ncm)

        #Angle discretization
        self.ndpsi = rospy.get_param('/vfh/ndpsi',72)

        #Get goal
        self.xGG_odom = np.array([4, 0])
        self.xGG_odom_point = PointStamped()
        self.xGG_odom_point.point.x = self.xGG_odom[0]
        self.xGG_odom_point.point.y = self.xGG_odom[1]
        self.xGG_odom_point.point.z = 0
        self.psiGG = np.mod(2*np.pi - np.pi/2,2*np.pi)
        
        # xGG_odom in base_link, for later use
        self.xGG = np.array([0, 0])

        #Get initial position. Get this from some topic.
        # self.xR0 = np.array([-2.3,0.5])
        self.xR0 = self.xGG_odom

        #Initialize previous waypoint position and pose
        self.xyvfh0 = self.xR0
        
        # get a single message from /imu_data
        # self.imu_msg = rospy.wait_for_message('/imu_data',Imu)
        # self.imu_flag = True

        #Define subscriber to odom
        # self.odom_sub = rospy.Subscriber("/odometry/filtered",Odometry,self.robot_pose)
        self.odom_sub = rospy.Subscriber("/hoverboard_velocity_controller/odom", Odometry, self.robot_pose)
        #Define flag
        self.fodom = False

        #Define subscriber to local_costmap data
        # self.local_costmap_sub = rospy.Subscriber("/costmap_2d/costmap/costmap_updates",
        # OccupancyGridUpdate,self.local_costmap)

        self.local_costmap_sub = rospy.Subscriber("/costmap_2d/costmap/costmap",
        OccupancyGrid,self.local_costmap)

        #Define flag 
        self.fcost = False

        #Define subscriber to img_to_laser/lane_scan
        self.lane_scan_sub = rospy.Subscriber("/img_to_laser/lane_scan",LaserScan,self.lane_scan)
        #Define flag
        self.flane_scan = False

        #Define publisher for VFH waypont
        self.vfh_pub = rospy.Publisher("/vfh/waypoint",PoseStamped,queue_size=2)

        #Define publisher for VFH waypoint marker
        self.marker_pub = rospy.Publisher("/visualization_marker",Marker,queue_size=2)

        #Define publisher for goal marker
        self.goal_pub = rospy.Publisher("/goal_marker",Marker,queue_size=2)

        #Define publisher for local goal marker
        self.local_pub = rospy.Publisher("/local_marker",Marker,queue_size=2)

        #Define flag for local goal
        self.flocal_goal = False
        
        # tf2_ros variables
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)
        

    #Receive robot pose
    def robot_pose(self,pose_msg):
        self.X = pose_msg.pose.pose.position.x
        self.Y = pose_msg.pose.pose.position.y
        psiz = pose_msg.pose.pose.orientation.z
        psiw = pose_msg.pose.pose.orientation.w
        self.psi = np.mod(2*np.pi + 2*np.arctan2((psiz),(0.001+psiw)),2*np.pi)

        #Set flag to true
        self.fodom = True
        
        # if self.imu_flag:
        #     psiz = 0
        #     psiw = 1
        #     self.imu_flag = False


    #Receive lane scan data 
    def lane_scan(self,lane):
        self.lane_scanned = np.array(lane.ranges)

        #Set flag to true
        self.flane_scan = True

        #Determine local goal if robot pose available
        if self.fodom:
            self.local_goal(self.lane_scanned)


    #Recieve local costmap data
    #local_costmap is postioned in /odom frame
    def local_costmap(self,costmap_msg):
        self.local_cost = costmap_msg.data

        #Set flag to true
        self.fcost = True

        if self.fodom and self.flane_scan and self.flocal_goal:
            self.pub_VFHwp()


    #Determine local goal
    def local_goal(self,lane_scanned):

        #Filter points
        min_range = 1
        max_range = 4
        #Determine indices of admissible points
        idx = np.array(np.where(np.logical_and(lane_scanned>=min_range, lane_scanned<=max_range)))
        idx = idx[0]
        print('Indices of admissible lane points', idx)
        #Determine distance to the admissible points
        dist = lane_scanned[idx]
        # print(dist)

        #Determine heading, relative to the robot, to the admissible points
        psil = -np.pi/179*idx + np.pi/2

        #Heading in global frame
        # psil = psil + self.psi

        #x-y location relative to the robot
        x = np.multiply(dist,np.cos(psil))
        y = np.multiply(dist,np.sin(psil))

        # print(x)
        # print(y)
        
        # #Transform to odom
        # xod = self.X + x
        # yod = self.Y + y

        # print(xod)
        # print(yod)

        # lookup transform between odom and base_link
        while not rospy.is_shutdown():
            try:
                # lookupTransform(target_frame, source_frame, time) -> position, quaternion
                trans = self.tfBuffer.lookup_transform('base_link', 'odom', rospy.Time())
                self.xGG = do_transform_point(self.xGG_odom_point , trans)
                self.xGG = np.array([self.xGG.point.x, self.xGG.point.y])
                print('wpt: ', self.xGG)
                break
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                print('no tf found')
                continue
        
        #Determine lane point closest to first global waypoint
        D2GWP = np.sqrt(np.square(x-self.xGG[0]) + np.square(y-self.xGG[1]))

        # print(D2GWP)

        idxmin = np.argmin(D2GWP)
        # print('Index of closest lane point',idxmin)
        self.xlG = np.array([x[idxmin],y[idxmin]])
        self.psilG = psil[idxmin]

        #Send to local goal marker
        self.local_marker(self.xlG[0],self.xlG[1],self.psilG)

        #Set flag to true
        self.flocal_goal = True



    #Determine VFH waypoint
    def pub_VFHwp(self):

        #Collect costmap
        ODcp = np.array(self.local_cost)

        #Collect critical cost
        crit_cost = self.crit_cost

        #Collect robot position and pose
        xyR = np.array([self.X,self.Y])
        psiR = self.psi

        # #Collect goal position and pose
        # xG = self.xGG
        # psiG = self.psiGG

        #Set local goal position and pose as VFH goal
        xG = self.xlG
        psiG = self.psilG

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

            npts = np.arange(3,LA,dtype=int)

        #     #Filter heading within lane_scan
        #     min_range = 1.5
        #     max_range = 5
        #     lane_scanned_filtered = np.where(np.logical_and(lane_scanned>=min_range, lane_scanned<=max_range))

        #     print(np.array(lane_scanned_filtered).size)
            
        #     if np.array(lane_scanned_filtered).size>1:

                

        #         #We see sufficient lane to go ahead with VFH for lane-keeping

        #         #Max forward heading relative to the robot 
        #         max_heading_angle = -np.pi/179*np.min(lane_scanned_filtered) + np.pi/2 
        #         #Min forward heading relative to the robot 
        #         min_heading_angle = -np.pi/179*np.max(lane_scanned_filtered) + np.pi/2 

        #         print(max_heading_angle)
        #         print(min_heading_angle)

        #         #vfhpsi relative to the robot
        #         vfhpsirel = vfhpsi-psiR
        #         vfhpsirel = np.mod((vfhpsirel + np.pi),2*np.pi) - np.pi

        #         #Collect allowable heading angles for a forward maneuver
        #         vfhpsi1 = vfhpsi[(vfhpsirel>min_heading_angle)*(vfhpsirel<max_heading_angle)]

        #         #Collect allowable heading angles for a reverse maneuver
        #         rev_angle = np.pi/6
        #         revmin = np.pi-rev_angle
        #         revmax = np.pi+rev_angle
        #         vfhpsi2 = vfhpsi[(vfhpsirel>revmin)*(vfhpsirel<revmax)]

        #         #Collect admissible heading angles
        #         vfhpsi = np.concatenate((vfhpsi1,vfhpsi2))

        #         print(vfhpsi)

            for theta in vfhpsi:
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

                    #Jump to furthest grid cell along direction theta
                    #Candidate waypoint xy coordinates in world frame
                    cwx = xyR[0] + ds*npts[-1]*np.cos(theta)
                    cwy = xyR[1] + ds*npts[-1]*np.sin(theta)
                    
                    #Robot heading to goal
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
            
            # else:
            #     print('Not enough lanes to use VFH safely')
            #     # break

        else:
            xyvfh = np.array(xG)
            psivfh = psiG
            # print('Cand. VFH waypoint: ',xyvfh)
            # print('Cand. VFH waypoint pose: ',psivfh)
            # THETAWP = thetawp
            # THETAG = thetag
            # THETAWP0 = thetawp0

        #Update previous waypoint
        self.xyvfh0 = xyvfh

        #Print waypoint coordinate and cost
        # print('VFH waypoint: ',xyvfh)
        # print('VFH waypoint pose: ',psivfh)
        # print('Robot costmap cost: ',ODcp[(ncm-ncm//2-1)*ncm + ncm//2])
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

        self.vfh_pub.publish(wp)

        #Send VFH waypoint to a blue marker publisher
        self.pub_marker(xyvfh[0],xyvfh[1],psivfh)
        #Send global goal to a green goal publisher
        self.goal_marker(self.xGG[0],self.xGG[1],self.psiGG)

        #Reset flags
        self.fcost = False
        self.fodom = False

    #A function that publishes a marker at the VFH waypoint
    def pub_marker(self,wpx,wpy,wppsi):

        #Define marker
        self.marker = Marker()

        self.marker.header.frame_id = "base_link"
        self.marker.header.stamp = rospy.Time.now()

        self.marker.type = 0
        self.marker.id = 0

        self.marker.scale.x = 0.3
        self.marker.scale.y = 0.05
        self.marker.scale.z = 0.05

        self.marker.color.r = 1
        self.marker.color.g = 1
        self.marker.color.b = 0
        self.marker.color.a = 1

        psiR = wppsi

        self.marker.pose.position.x = wpx
        self.marker.pose.position.y = wpy
        self.marker.pose.position.z = 0.1
        self.marker.pose.orientation.x = 0.0
        self.marker.pose.orientation.y = 0.0
        self.marker.pose.orientation.z = np.sin(psiR/2)
        self.marker.pose.orientation.w = np.cos(psiR/2)

        #Publish marker
        self.marker_pub.publish(self.marker)
        print(self.marker)


    #A function that publishes a marker at the global goal
    def goal_marker(self,wpx,wpy,wppsi):

        #Define marker
        self.marker = Marker()

        self.marker.header.frame_id = "base_link"
        self.marker.header.stamp = rospy.Time.now()

        self.marker.type = 0
        self.marker.id = 0

        self.marker.scale.x = 0.3
        self.marker.scale.y = 0.05
        self.marker.scale.z = 0.05

        self.marker.color.r = 0
        self.marker.color.g = 1
        self.marker.color.b = 0
        self.marker.color.a = 1

        psiR = wppsi

        self.marker.pose.position.x = wpx
        self.marker.pose.position.y = wpy
        self.marker.pose.position.z = 0.2
        self.marker.pose.orientation.x = 0.0
        self.marker.pose.orientation.y = 0.0
        self.marker.pose.orientation.z = np.sin(psiR/2)
        self.marker.pose.orientation.w = np.cos(psiR/2)

        #Publish marker
        self.goal_pub.publish(self.marker)


    #A function that publishes a marker at the local goal
    def local_marker(self,wpx,wpy,wppsi):

        #Define marker
        self.marker = Marker()

        self.marker.header.frame_id = "base_link"
        self.marker.header.stamp = rospy.Time.now()

        self.marker.type = 0
        self.marker.id = 0

        self.marker.scale.x = 0.3
        self.marker.scale.y = 0.05
        self.marker.scale.z = 0.05

        self.marker.color.r = 1
        self.marker.color.g = 0
        self.marker.color.b = 1
        self.marker.color.a = 1

        psiR = wppsi

        self.marker.pose.position.x = wpx
        self.marker.pose.position.y = wpy
        self.marker.pose.position.z = 0.2
        self.marker.pose.orientation.x = 0.0
        self.marker.pose.orientation.y = 0.0
        self.marker.pose.orientation.z = np.sin(psiR/2)
        self.marker.pose.orientation.w = np.cos(psiR/2)

        #Publish marker
        self.local_pub.publish(self.marker)



if __name__ == '__main__':
    #Initialize node
    rospy.init_node('vfhROS',anonymous=True)

    try:

        VFH()
        rospy.spin()

    except rospy.ROSInterruptException:
        pass
