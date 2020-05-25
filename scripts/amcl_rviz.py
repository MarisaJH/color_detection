import os
import sys
from multiprocessing import Pool

processes = ["color_detection amcl.launch", "turtlebot_rviz_launchers view_navigation.launch --screen"]
def run_processes(process):
    os.system('roslaunch {}'.format(process))

if __name__ == "__main__":

    if len(sys.argv) > 1:
        arg = sys.argv[1]
        if arg == "physical":
            processes[0] = "color_detection amcl_zones.launch"
        elif arg == "sketched":
            processes[0] = "color_detection amcl_sketched_zones.launch"
        elif arg == "virtual":
            processes[0] = "color_detection amcl_virtual_zones.launch"

    pool = Pool(processes=2)
    pool.map(run_processes, processes)
