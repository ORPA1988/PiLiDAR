#!/usr/bin/env python3
"""
Live 2D Polar Plot Visualization for LiDAR Scanner

Usage:
    python tools/live_view_2d.py [--max-distance METERS]
    
Options:
    --max-distance METERS    Maximum display range in meters (default: 6.0)

This script visualizes LiDAR data in real-time as a 2D polar plot.
Connect to the LiDAR scanner and watch the scan unfold.
"""

import sys
import os
import argparse
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

import numpy as np
import matplotlib
# Use a backend suitable for Raspberry Pi desktops; fallback safely
try:
    matplotlib.use('TkAgg')
except Exception:
    pass
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from collections import deque
import threading
import time

from lib.config import Config
from lib.lidar_driver import Lidar


class LiveLidarView:
    """Real-time 2D polar visualization of LiDAR scan data."""
    
    def __init__(self, max_distance=6000, max_points=5000):
        """
        Initialize live LiDAR viewer.
        
        Args:
            max_distance: Maximum distance in mm for plot range
            max_points: Maximum number of points to display (for performance)
        """
        self.max_distance = max_distance
        self.max_points = max_points
        
        # Data storage with circular buffer
        self.angles = deque(maxlen=max_points)
        self.distances = deque(maxlen=max_points)
        self.intensities = deque(maxlen=max_points)
        
        # Setup plot with fixed size
        self.fig = plt.figure(figsize=(10, 10))
        self.ax = self.fig.add_subplot(111, projection='polar')
        self.scatter = None
        self.text_info = None
        self.cid_key = None
        
        self.setup_plot()
        self.bind_events()
        
        self.point_count = 0
        self.current_z_angle = 0
        
        # Thread-safe flag for updates
        self.running = False
        self.needs_update = False
        self.update_lock = threading.Lock()
        
        # Force initial draw
        plt.draw()
        plt.pause(0.001)
        
    def setup_plot(self):
        """Configure plot appearance with responsive layout."""
        self.ax.set_ylim(0, self.max_distance)
        try:
            self.ax.set_theta_zero_location('N')  # 0° at top
            self.ax.set_theta_direction(-1)  # Clockwise
        except AttributeError:
            pass  # Not all matplotlib versions support these
        
        # Responsive title size based on figure size
        fig_width = self.fig.get_figwidth()
        title_size = max(10, min(16, int(fig_width * 1.5)))
        self.ax.set_title('Live LiDAR Scan (2D Polar View)', 
                         pad=20, fontsize=title_size, fontweight='bold')
        self.ax.grid(True, alpha=0.3)
        
        # Info text with responsive font size
        info_fontsize = max(8, min(12, int(fig_width * 1.2)))
        self.text_info = self.fig.text(0.02, 0.98, '', 
                                       transform=self.fig.transFigure,
                                       verticalalignment='top',
                                       fontsize=info_fontsize,
                                       family='monospace')

    def bind_events(self):
        """Bind keyboard events for interactive controls."""
        def on_key(event):
            # Adjust range with +/- keys; 'r' resets plot; 'q' quits
            if event.key == '+':
                self.set_range(self.max_distance * 1.25)
            elif event.key == '-':
                self.set_range(max(100.0, self.max_distance / 1.25))
            elif event.key == 'r':
                self.clear_points()
            elif event.key == 'q':
                plt.close(self.fig)
        self.cid_key = self.fig.canvas.mpl_connect('key_press_event', on_key)

    def set_range(self, max_distance_mm):
        """Update display range (in mm) and refresh plot."""
        self.max_distance = float(max_distance_mm)
        self.ax.set_ylim(0, self.max_distance)
        self.update_plot()

    def clear_points(self):
        """Clear all stored points from the buffer."""
        self.angles.clear()
        self.distances.clear()
        self.intensities.clear()
        self.point_count = 0
        self.scatter = None
        self.update_plot()
        
    def add_point(self, angle_deg, distance_mm, intensity=None):
        """
        Add a new LiDAR measurement point.
        
        Args:
            angle_deg: Angle in degrees (0-360)
            distance_mm: Distance in millimeters
            intensity: Optional intensity value
        """
        if distance_mm > 0:  # Filter invalid measurements
            with self.update_lock:
                self.angles.append(np.deg2rad(angle_deg))
                self.distances.append(distance_mm)
                self.intensities.append(intensity if intensity is not None else 128)
                self.point_count += 1
                self.needs_update = True
    
    def update_z_angle(self, z_angle):
        """Update current z-axis rotation angle."""
        self.current_z_angle = z_angle
    
    def update_plot(self, frame=None):
        """Redraw the plot with current data."""
        with self.update_lock:
            if not self.needs_update and frame is None:
                return
            self.needs_update = False
            
            # Clear axes to ensure polar projection re-renders correctly
            self.ax.clear()
            self.setup_plot()

            if len(self.angles) > 0:
                angles_array = np.array(list(self.angles))
                distances_array = np.array(list(self.distances))
                intensities_array = np.array(list(self.intensities))

                # Downsample to reduce CPU if buffer is large
                if len(angles_array) > self.max_points:
                    step = max(1, len(angles_array) // self.max_points)
                    angles_array = angles_array[::step]
                    distances_array = distances_array[::step]
                    intensities_array = intensities_array[::step]

                # Color by intensity (normalized)
                try:
                    cmap = plt.colormaps['viridis']
                except (AttributeError, KeyError):
                    from matplotlib.cm import get_cmap
                    cmap = get_cmap('viridis')
                colors = cmap(intensities_array.astype(float) / 255.0)

                fig_width = self.fig.get_figwidth()
                marker_size = max(1, min(5, fig_width * 0.3))

                # Recreate scatter each frame for reliable polar plotting
                self.scatter = self.ax.scatter(angles_array, distances_array, c=colors, s=marker_size, alpha=0.6)
            
            # Update info text
            info_text = (f"Points: {self.point_count}\n"
                        f"Z-Angle: {self.current_z_angle:.2f}°\n"
                        f"Buffer: {len(self.angles)}/{self.max_points}")
            if self.text_info is not None:
                self.text_info.set_text(info_text)
        
        # Ensure immediate canvas update (outside lock to avoid deadlock)
        try:
            self.fig.canvas.draw()
            self.fig.canvas.flush_events()
        except Exception:
            pass
    
    def show(self):
        """Display the plot window and start update timer."""
        plt.ion()  # Interactive mode
        plt.show(block=False)
        self.running = True
        
        # Start periodic update timer (50ms = 20 FPS)
        def update_timer():
            while self.running:
                if self.needs_update:
                    try:
                        self.update_plot()
                    except Exception as e:
                        print(f"Update error: {e}")
                time.sleep(0.05)  # 50ms between updates
        
        self.update_thread = threading.Thread(target=update_timer, daemon=True)
        self.update_thread.start()
        
        # Initial empty plot draw
        self.update_plot()
        plt.pause(0.01)
    
    def close(self):
        """Close the plot window and stop update thread."""
        self.running = False
        if hasattr(self, 'update_thread'):
            self.update_thread.join(timeout=1.0)
        plt.close(self.fig)


def lidar_callback_generator(live_view):
    """
    Generate callback function for LiDAR driver.
    
    Args:
        live_view: LiveLidarView instance
        
    Returns:
        Callback function
    """
    update_counter = [0]  # Use list to allow modification in closure
    
    def callback():
        """Callback executed after each LiDAR package."""
        update_counter[0] += 1
        
        # Update plot every N packages (for performance)
        if update_counter[0] % 5 == 0:
            live_view.update_plot()
    
    return callback


def main():
    """Main function to run live LiDAR visualization."""
    # Parse command-line arguments
    parser = argparse.ArgumentParser(description='Live 2D Polar LiDAR Visualization')
    parser.add_argument('--max-distance', type=float, default=6.0,
                       help='Maximum display range in meters (default: 6.0)')
    args = parser.parse_args()
    
    print("Starting Live LiDAR 2D Polar View...")
    
    # Temporarily disable GPIO setup in Config for live view only
    import lib.config as config_module
    original_gpio_setup = config_module.Config.gpio_setup
    config_module.Config.gpio_setup = lambda self, debug=False: None  # No-op
    
    try:
        # Load configuration without GPIO setup (only for LiDAR reading)
        config = Config()
        config.relay_device = None
        config.init(scan_id="_live")
    finally:
        # Restore original method
        config_module.Config.gpio_setup = original_gpio_setup
    
    # Create live view with user-specified or default max distance
    max_dist = args.max_distance * 1000  # Convert m to mm
    live_view = LiveLidarView(max_distance=max_dist, max_points=10000)
    
    # Initialize LiDAR
    print(f"Connecting to LiDAR on {config.PORT}...")
    lidar = Lidar(config, visualization=None)
    
    # Show plot window
    live_view.show()
    
    print("Live view started. Press Ctrl+C to stop.")
    print(f"Max display range: {args.max_distance:.1f} meters")
    print("Keyboard shortcuts: [+] zoom in, [-] zoom out, [r] reset, [q] quit")
    
    # Track last processed point count
    last_point_count = [0]
    
    def callback_with_viz():
        """Callback that extracts new points from lidar and updates visualization."""
        # Extract new points from lidar.points_2d buffer
        current_count = lidar.out_i * lidar.dlength

        # Handle buffer rollover when lidar.out_i resets to 0
        if current_count < last_point_count[0]:
            last_point_count[0] = 0

        if current_count > last_point_count[0]:
            # Get new points since last update
            new_points = lidar.points_2d[last_point_count[0]:current_count]
            for point in new_points:
                x, y, intensity = point
                # Convert cartesian back to polar for display
                distance = np.sqrt(x**2 + y**2)
                angle = np.arctan2(y, x) * 180 / np.pi
                if angle < 0:
                    angle += 360
                live_view.add_point(angle, distance, int(intensity))
            
            # Update z-angle if available
            if lidar.z_angle is not None:
                live_view.update_z_angle(lidar.z_angle)
            
            last_point_count[0] = current_count
    
    try:
        # Start reading LiDAR data (continuous mode - no max_packages)
        lidar.read_loop(callback=callback_with_viz, max_packages=None)
        
    except KeyboardInterrupt:
        print("\n\nStopping live view...")
    
    finally:
        lidar.close()
        live_view.close()
        print("Live view closed.")


if __name__ == "__main__":
    main()
