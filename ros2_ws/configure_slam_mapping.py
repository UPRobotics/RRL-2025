#!/usr/bin/env python3

"""
SLAM Toolbox Manual Configuration Script (Python Version)
This script provides advanced manual configuration for SLAM Toolbox mapping node
with parameter customization and detailed monitoring.
"""

import rclpy
from rclpy.node import Node
from lifecycle_msgs.srv import GetState, ChangeState, GetAvailableTransitions
from lifecycle_msgs.msg import Transition
from std_msgs.msg import String
import time
import sys
import signal


class SLAMConfigurator(Node):
    def __init__(self):
        super().__init__('slam_configurator')
        
        # Service clients for SLAM Toolbox lifecycle management
        self.get_state_client = self.create_client(GetState, '/slam_toolbox/get_state')
        self.change_state_client = self.create_client(ChangeState, '/slam_toolbox/change_state')
        self.get_transitions_client = self.create_client(GetAvailableTransitions, '/slam_toolbox/get_available_transitions')
        
        # Wait for services to be available
        self.wait_for_services()
        
    def wait_for_services(self):
        """Wait for all required services to become available."""
        services = [
            (self.get_state_client, '/slam_toolbox/get_state'),
            (self.change_state_client, '/slam_toolbox/change_state'),
            (self.get_transitions_client, '/slam_toolbox/get_available_transitions')
        ]
        
        for client, service_name in services:
            self.get_logger().info(f'Waiting for service: {service_name}')
            if not client.wait_for_service(timeout_sec=10.0):
                self.get_logger().error(f'Service {service_name} not available')
                return False
            self.get_logger().info(f'Service {service_name} is available')
        
        return True
    
    def get_current_state(self):
        """Get the current state of SLAM Toolbox."""
        request = GetState.Request()
        future = self.get_state_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        
        if future.result() is not None:
            return future.result().current_state
        else:
            self.get_logger().error('Failed to get current state')
            return None
    
    def get_available_transitions(self):
        """Get available transitions from current state."""
        request = GetAvailableTransitions.Request()
        future = self.get_transitions_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        
        if future.result() is not None:
            return future.result().available_transitions
        else:
            self.get_logger().error('Failed to get available transitions')
            return []
    
    def change_state(self, transition_id, transition_label):
        """Change the state of SLAM Toolbox."""
        request = ChangeState.Request()
        request.transition = Transition()
        request.transition.id = transition_id
        
        self.get_logger().info(f'Requesting transition: {transition_label} (ID: {transition_id})')
        
        future = self.change_state_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
        
        if future.result() is not None:
            success = future.result().success
            if success:
                self.get_logger().info(f'Successfully transitioned to: {transition_label}')
            else:
                self.get_logger().error(f'Failed to transition to: {transition_label}')
            return success
        else:
            self.get_logger().error('Failed to call change_state service')
            return False
    
    def wait_for_state(self, target_state_id, timeout=10.0):
        """Wait for SLAM Toolbox to reach a specific state."""
        start_time = time.time()
        
        while (time.time() - start_time) < timeout:
            current_state = self.get_current_state()
            if current_state and current_state.id == target_state_id:
                return True
            time.sleep(0.5)
        
        return False
    
    def configure_and_activate(self):
        """Main configuration process."""
        self.get_logger().info('Starting SLAM Toolbox configuration...')
        
        # Get current state
        current_state = self.get_current_state()
        if not current_state:
            self.get_logger().error('Could not determine current state')
            return False
        
        self.get_logger().info(f'Current state: {current_state.label} (ID: {current_state.id})')
        
        # State machine for configuration
        if current_state.id == 1:  # unconfigured
            self.get_logger().info('SLAM Toolbox is unconfigured. Configuring...')
            
            # Configure
            if not self.change_state(1, 'configure'):
                return False
            
            if not self.wait_for_state(2, timeout=15.0):  # inactive
                self.get_logger().error('Timeout waiting for configured state')
                return False
            
            # Activate
            self.get_logger().info('Activating SLAM Toolbox...')
            if not self.change_state(3, 'activate'):
                return False
            
            if not self.wait_for_state(3, timeout=15.0):  # active
                self.get_logger().error('Timeout waiting for active state')
                return False
        
        elif current_state.id == 2:  # inactive (configured)
            self.get_logger().info('SLAM Toolbox is configured but inactive. Activating...')
            
            if not self.change_state(3, 'activate'):
                return False
            
            if not self.wait_for_state(3, timeout=15.0):  # active
                self.get_logger().error('Timeout waiting for active state')
                return False
        
        elif current_state.id == 3:  # active
            self.get_logger().info('SLAM Toolbox is already active and mapping!')
            return True
        
        else:
            self.get_logger().warning(f'SLAM Toolbox in unexpected state: {current_state.label}')
            
            # Try cleanup and reconfigure
            self.get_logger().info('Attempting cleanup and reconfiguration...')
            
            available_transitions = self.get_available_transitions()
            
            # Look for cleanup transition
            for transition in available_transitions:
                if transition.transition.label == 'cleanup':
                    if self.change_state(transition.transition.id, 'cleanup'):
                        time.sleep(2)
                        return self.configure_and_activate()  # Recursive call
                    break
        
        # Final verification
        final_state = self.get_current_state()
        if final_state and final_state.id == 3:
            self.get_logger().info('✅ SLAM Toolbox successfully configured and activated!')
            return True
        else:
            self.get_logger().error(f'Failed to activate SLAM Toolbox. Final state: {final_state.label if final_state else "Unknown"}')
            return False
    
    def verify_mapping_topics(self):
        """Verify that mapping topics are available."""
        self.get_logger().info('Verifying mapping topics...')
        
        # Get list of topics
        topic_names = self.get_topic_names_and_types()
        available_topics = [name for name, _ in topic_names]
        
        required_topics = ['/map', '/map_metadata', '/pose']
        optional_topics = ['/map_updates', '/slam_toolbox/graph_visualization']
        
        for topic in required_topics:
            if topic in available_topics:
                self.get_logger().info(f'✓ Required topic available: {topic}')
            else:
                self.get_logger().warning(f'✗ Required topic missing: {topic}')
        
        for topic in optional_topics:
            if topic in available_topics:
                self.get_logger().info(f'✓ Optional topic available: {topic}')
            else:
                self.get_logger().info(f'○ Optional topic not found: {topic}')


def signal_handler(sig, frame):
    """Handle script interruption."""
    print('\n\n[INFO] Script interrupted by user')
    rclpy.shutdown()
    sys.exit(130)


def main():
    # Set up signal handler
    signal.signal(signal.SIGINT, signal_handler)
    
    print("=" * 50)
    print("SLAM Toolbox Manual Configuration Script (Python)")
    print("=" * 50)
    
    # Initialize ROS2
    rclpy.init()
    
    try:
        # Create configurator node
        configurator = SLAMConfigurator()
        
        # Configure and activate SLAM Toolbox
        success = configurator.configure_and_activate()
        
        if success:
            # Verify mapping functionality
            configurator.verify_mapping_topics()
            
            print("\n" + "=" * 50)
            print("SLAM CONFIGURATION COMPLETED SUCCESSFULLY!")
            print("=" * 50)
            print("\nAvailable commands for monitoring:")
            print("  ros2 topic echo /map --once")
            print("  ros2 topic echo /pose --once")
            print("  ros2 topic hz /map")
            print("  rviz2 -d src/testing/rviz/slam_config.rviz")
            print("\nPress Ctrl+C to exit")
            print("=" * 50)
            
            # Keep the node alive for monitoring
            try:
                rclpy.spin(configurator)
            except KeyboardInterrupt:
                print("\n[INFO] Shutting down configurator...")
        
        else:
            print("\n[ERROR] Failed to configure SLAM Toolbox")
            return 1
    
    except Exception as e:
        print(f"\n[ERROR] Exception occurred: {e}")
        return 1
    
    finally:
        rclpy.shutdown()
    
    return 0


if __name__ == '__main__':
    sys.exit(main())
