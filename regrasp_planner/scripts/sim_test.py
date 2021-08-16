#!/usr/bin/env python
import rospy
from fetch_robot import Fetch_Robot
from  tf_util import TF_Helper, transformProduct, getMatrixFromQuaternionAndTrans
from rail_segmentation.srv import SearchTable

if __name__=='__main__':
    rospy.init_node('test_node')
    robot = Fetch_Robot(sim=True)
    tf_helper = TF_Helper()

    # add the table into moveit
    tran_base_Table = tf_helper.getTransform('/base_link', '/Table') #return tuple (trans,rot) of parent_lin to child_link
    robot.addCollisionObject("table_collsion", tran_base_Table,'objects/Table.stl', size_scale=1.0)


    # add the cup into the moveit
    tran_base_Cup = tf_helper.getTransform('/base_link', '/Cup') #return tuple (trans,rot) of parent_lin to child_link
    robot.addCollisionObject("cup_collsion", tran_base_Cup,'objects/cup.stl')

    # get table information
    rospy.wait_for_service('table_searcher/search_table')
    tableSearcher = rospy.ServiceProxy('table_searcher/search_table', SearchTable)

    try:
      tableresult = tableSearcher()
    except rospy.ServiceException as exc:
        print("Service did not process request: " + str(exc))
        
    # get the pose mat from base link to table
    tableposMat = getMatrixFromQuaternionAndTrans(tableresult.orientation, tableresult.centroid)

    # show the position tf in rviz
    tf_helper.pubTransform("place_pos", ((tableresult.centroid.x, tableresult.centroid.y, tableresult.centroid.z), \
                    (tableresult.orientation.x, tableresult.orientation.y, tableresult.orientation.z, tableresult.orientation.w)))