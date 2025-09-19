#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from object_detect.srv import DetectCoffeeSupply
from std_srvs.srv import Trigger # The service type for starting/stopping the camera
import time

class TestMainController:
    """
    A simplified main controller for the specific purpose of testing 
    the coffee supply detection service and its camera control mechanism.
    """
    def __init__(self):
        rospy.init_node('test_main_control_node')
        rospy.loginfo("Test Main Controller Node Started.")

        # --- Service Clients ---
        # This is the core of our test setup. We need to connect to the
        # three services provided by the coffee_supply_server.
        rospy.loginfo("Waiting for coffee detection services...")
        try:
            # Wait for all services to be available before starting the test
            rospy.wait_for_service('start_coffee_camera', timeout=10.0)
            self.start_coffee_cam_client = rospy.ServiceProxy('start_coffee_camera', Trigger)
            
            rospy.wait_for_service('stop_coffee_camera', timeout=10.0)
            self.stop_coffee_cam_client = rospy.ServiceProxy('stop_coffee_camera', Trigger)
            
            rospy.wait_for_service('CoffeeSupply', timeout=10.0)
            self.detect_coffee_client = rospy.ServiceProxy('CoffeeSupply', DetectCoffeeSupply)
            
            rospy.loginfo("All coffee detection services are ready.")

        except rospy.ROSException as e:
            rospy.logfatal(f"A required service did not become available. Shutting down. Error: {e}")
            return # Abort initialization

        # --- State Variables for storing detection results ---
        self.coffee_color = None
        self.table = 0
        self.cup_side = None
        self.motor_num = None
        
        # --- Run the test flow ---
        self.run_test_flow()
        

    def detect_coffee_supply(self):
        """
        This function is a direct copy from the main production script.
        It manages the camera lifecycle to detect coffee supply, ensuring
        the test is representative of the real use case.
        """
        rospy.loginfo("--- Starting Coffee Supply Detection Task ---")
        try:
            # Step 1: Start the camera via its service
            rospy.loginfo("Requesting to start camera...")
            start_resp = self.start_coffee_cam_client()
            if not start_resp.success:
                rospy.logerr(f"Failed to start coffee camera: {start_resp.message}")
                return False
            
            rospy.loginfo("Camera started successfully. Calling detection service...")
            # Allow a brief moment for camera auto-exposure and focus to stabilize
            rospy.sleep(0.5)

            # Step 2: Call the main detection service
            resp = self.detect_coffee_client()
            
            if not resp.success or resp.table == 0:
                rospy.logwarn(f"CoffeeSupply service reported no success or no table found. Message: {resp}")
                return False

            # Step 3: Process the successful response and store the results
            self.coffee_color = resp.target_name.lower()
            self.table = int(resp.table)
            self.cup_side = resp.cup_side.lower()
            
            # Determine which motor would be used based on coffee color
            if self.coffee_color == 'black':
                self.motor_num = 1
            elif self.coffee_color == 'white':
                self.motor_num = 2
            else:
                rospy.logwarn(f"Unknown coffee color '{self.coffee_color}', can't determine motor number.")
                self.motor_num = None

            rospy.loginfo(f"DETECTION SUCCESS! Target: '{self.coffee_color}' coffee for table: {self.table}. Cup is on the '{self.cup_side}' side.")
            return True

        except rospy.ServiceException as e:
            rospy.logerr(f"A service call failed during the detection process: {e}")
            return False
        finally:
            # Step 4: CRITICAL! Always ensure the camera is stopped,
            # regardless of whether the detection succeeded or failed.
            rospy.loginfo("Requesting to stop camera (in 'finally' block)...")
            try:
                stop_resp = self.stop_coffee_cam_client()
                if not stop_resp.success:
                    rospy.logwarn(f"Could not stop coffee camera cleanly: {stop_resp.message}")
                else:
                    rospy.loginfo("Camera stopped successfully.")
            except rospy.ServiceException as e:
                rospy.logerr(f"Failed to call the stop_coffee_camera service: {e}")

            rospy.loginfo("--- Finished Coffee Supply Detection Task ---")


    def run_test_flow(self):
        """
        The main testing sequence. It calls the detection function multiple
        times to check for reliability and proper resource management.
        """
        rospy.loginfo("=============================================")
        rospy.loginfo("=      STARTING COFFEE DETECTION TEST       =")
        rospy.loginfo("=============================================")
        
        # --- First Test Run ---
        rospy.loginfo("\n>>> Performing First Detection Test...")
        success1 = self.detect_coffee_supply()
        if success1:
            rospy.loginfo(">>> First Test Result: SUCCESS")
        else:
            rospy.logerr(">>> First Test Result: FAILED")
            
        
        # time.sleep(2)
        
        # # --- Second Test Run ---
        # # This is important to ensure that the camera was properly stopped
        # # and can be successfully restarted.
        # rospy.loginfo("\n>>> Performing Second Detection Test...")
        # success2 = self.detect_coffee_supply()
        # if success2:
        #     rospy.loginfo(">>> Second Test Result: SUCCESS")
        # else:
        #     rospy.logerr(">>> Second Test Result: FAILED")

        # time.sleep(2)
        
        # # --- Second Test Run ---
        # # This is important to ensure that the camera was properly stopped
        # # and can be successfully restarted.
        # rospy.loginfo("\n>>> Performing Second Detection Test...")
        # success2 = self.detect_coffee_supply()
        # if success2:
        #     rospy.loginfo(">>> Third Test Result: SUCCESS")
        # else:
        #     rospy.logerr(">>> Third Test Result: FAILED")
        

        rospy.loginfo("\n=============================================")
        rospy.loginfo("=         COFFEE DETECTION TEST ENDED         =")
        rospy.loginfo("=============================================")
        
        # Shutdown the node after the test is complete
        rospy.signal_shutdown("Test finished.")

if __name__ == '__main__':
    try:
        TestMainController()
        # The rospy.spin() is not strictly necessary here because the test flow
        # runs to completion and then shuts down the node. But it's good practice
        # to have it in case the logic changes.
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Test node interrupted and shut down.")
    except Exception as e:
        rospy.logfatal(f"An unhandled exception occurred in the test controller: {e}")

