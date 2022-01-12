import sys
sys.path.append("$(rospack find home_service_robot)/scripts")
import rospy
import core
    

def find_book():
    astra = core.Astra()
    manipulator = core.Manipulator()
    navigation = core.Navigation()

