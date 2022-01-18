import sys
import rospkg
print(f"path: {rospkg.RosPack()}")
sys.path.append(f"{rospkg.RosPack().get_path('home_service_robot')}/scripts")
import rospy
import core

# def find_book():
#     astra = core.Astra()
#     manipulator = core.Manipulator()
#     navigation = core.Navigation()

