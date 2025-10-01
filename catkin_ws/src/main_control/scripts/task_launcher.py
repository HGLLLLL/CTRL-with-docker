#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import subprocess
import signal
import os
from std_msgs.msg import UInt8

class TaskLauncher:
    """
    監聽來自實體按鈕的 ROS 訊息 (數字ID)，並管理任務的啟動與終止。
    新功能：當新按鈕被按下時，會先終止正在運行的任務，然後再啟動新任務。
    """
    def __init__(self):
        rospy.init_node('task_launcher_node')
        rospy.loginfo("Task Launcher Node running. Ready to start/switch tasks.")

        # --- if want to add new num to the map, just add it directly---
        self.task_map = {
            1: ["main_control", "A_B_C_D.launch"],
            2: ["main_control", "A_B_C_D_1.launch"],
            3: ["main_control", "B_C_D.launch"],
            4: ["main_control", "B_C_D_1.launch"],
            5: ["main_control", "C_D.launch"],
            6: ["main_control", "C_D_1.launch"],
            7: ["main_control", "D.launch"],
            8: ["main_control", "D_1.launch"],
        }
        
        self.active_process = None
        self.active_task_id = None

        # subscribe to button press topic(if nano uses rosserial)
        rospy.Subscriber("/button_press", UInt8, self.button_callback)
        
        # register shutdown hook
        rospy.on_shutdown(self.shutdown_hook)

    def terminate_active_process(self):
        """
        終止當前由 roslaunch 啟動的任務。
        """
        if not self.active_process or self.active_process.poll() is not None:
            rospy.loginfo("No active process to terminate.")
            return

        rospy.logwarn("Terminating active task (ID: %s, PID: %s)...", self.active_task_id, self.active_process.pid)
        
        try:
            # 使用 os.killpg 和 preexec_fn=os.setsid 來確保整個進程組被終止
            # 這是最可靠的方式，能關閉 roslaunch 及其所有子進程(said by ai)
            os.killpg(os.getpgid(self.active_process.pid), signal.SIGINT)
            
            # if the process does not terminate within timeout, raise exception
            self.active_process.wait(timeout=5)
            rospy.loginfo("Process terminated gracefully.")
            
        except subprocess.TimeoutExpired:
            # kill it!!!!!
            rospy.logerr("Process did not terminate gracefully. Forcing kill...")
            os.killpg(os.getpgid(self.active_process.pid), signal.SIGKILL)
            rospy.logwarn("Process killed.")
            
        except Exception as e:
            rospy.logerr("Error during process termination: %s", e)

        self.active_process = None
        self.active_task_id = None


    def button_callback(self, msg):
        new_task_id = msg.data
        rospy.loginfo("Received button press ID: %d", new_task_id)

        # if press 0 代表空白模式，只 terminate，不執行新任務
        if new_task_id == 0:
            rospy.loginfo("Blank mode triggered, only terminating current launchfile.")
            if self.active_process and self.active_process.poll() is None:
                self.terminate_active_process()
            else:
                rospy.loginfo("No active launchfile to terminate.")
            return  # 直接 return，不執行後續的任務啟動

        # 舊邏輯：判斷任務是否合法
        if new_task_id not in self.task_map:
            rospy.logwarn("No task associated with ID: %d. Ignoring.", new_task_id)
            return

        # terminate first and run new
        if self.active_process and self.active_process.poll() is None:
            self.terminate_active_process()
            # add a short delay to ensure the process has fully terminated
            rospy.sleep(1.0)

        # launch new task
        package, launch_file = self.task_map[new_task_id]
        rospy.loginfo("Launching new task for ID %d: %s from package: %s", new_task_id, launch_file, package)
        try:
            command = ["roslaunch", package, launch_file]
            # preexec_fn=os.setsid 使得 roslaunch 在一個新的進程組中運行
            # 這讓我們可以透過 killpg 來終止整個組，包含所有子節點(said by ai)
            self.active_process = subprocess.Popen(command, preexec_fn=os.setsid)
            self.active_task_id = new_task_id
            rospy.loginfo("New process started with PID: %s", self.active_process.pid)
        except Exception as e:
            rospy.logerr("Failed to launch task for ID %d: %s", new_task_id, e)
            self.active_process = None
            self.active_task_id = None

    
    def shutdown_hook(self):
        """
        當此節點被關閉時 (e.g. Ctrl+C)，確保其子進程也被關閉。
        """
        rospy.logwarn("Task Launcher is shutting down. Terminating any active task.")
        self.terminate_active_process()

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        launcher = TaskLauncher()
        launcher.run()
    except rospy.ROSInterruptException:
        pass

