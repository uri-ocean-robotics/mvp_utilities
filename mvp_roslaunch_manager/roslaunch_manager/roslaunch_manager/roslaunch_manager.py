import subprocess
import threading
import time
import os
import signal
import psutil  
import socket
import sys
import select

class ROSLaunchManager:
    def __init__(self, local_ip, port_num, udp_stream):
        self.node_processes = {}
        self.lock = threading.Lock()
        self.udp_stream = udp_stream

        if self.udp_stream:
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.udp_ip = local_ip
            self.udp_port = port_num
            
            self.sock.setblocking(False)  # This makes the socket non-blocking
            self.running = True  # Flag to control the thread

    def start_launch(self, pkg, launch_file):
        with self.lock:
            key = (pkg, launch_file)
            if key in self.node_processes:
                print(f"Launch package [{pkg}], file [{launch_file}] is already running.")
                return
            env = os.environ.copy()
            if 'ROS_NAMESPACE' in env:
                del env['ROS_NAMESPACE']

            if self.udp_stream:
                file_name = launch_file + '.launch.py'
                process = subprocess.Popen(
                    ['ros2', 'launch', pkg, file_name],
                    env=env,  # You can modify environment variables here
                    stdout=subprocess.PIPE,
                    stderr=subprocess.PIPE,
                    text=True  # Ensure you get strings instead of bytes
                )

                self.node_processes[key] = process
                print(f"Started Package [{pkg}], launch file [{launch_file}].")
                self.running = True  # Flag to control the thread

                # Start a thread to handle output streaming
                output_thread = threading.Thread(target=self._stream_output, args=(process.stdout,))
                output_thread.daemon = True  # Daemon thread exits when the main program exits
                output_thread.start()

                # Optionally handle stderr as well, if you want to stream errors
                error_thread = threading.Thread(target=self._stream_output, args=(process.stderr,))
                error_thread.daemon = True
                error_thread.start()
            else:
                file_name = launch_file + '.launch.py'
                process = subprocess.Popen(['ros2', 'launch', str(pkg), str(file_name)], env=env)
                self.node_processes[key] = process
                print(f"Started Package [{pkg}], launch file [{launch_file}].")



    def _stream_output(self, pipe):
        heartbeat_interval = 5.0
        last_heartbeat_time = time.time()

        while self.running:
            rlist, _, _ = select.select([pipe], [], [], 0.01)  # timeout of 0.1 seconds
            if rlist:
                line = pipe.readline()
                if line:
                    print(line.strip())
                    self.sock.sendto(line.encode(), (self.udp_ip, self.udp_port))

                    if 'WARNING' in line.upper():
                        print(f"Warning: {line.strip()}")
                    elif 'ERROR' in line.upper():
                        print(f"Error: {line.strip()}")
            else:
                # No new line ready — time to maybe print heartbeat
                current_time = time.time()
                if current_time - last_heartbeat_time >= heartbeat_interval:
                    data = "[Stream still active...]\r\n"
                    self.sock.sendto(data.encode(), (self.udp_ip, self.udp_port))
                    # print("[Stream still active...]")
                    last_heartbeat_time = current_time

            time.sleep(0.01)  # Sleep to reduce CPU usage
        pipe.close()  # Close pipe once we're done
        print("Stream finished.")


    def stop_launch(self, pkg, launch_file):
        with self.lock:
            key = (pkg, launch_file)
            process = self.node_processes.pop(key, None)
            if process:
                try:
                    parent = psutil.Process(process.pid)
                    for child in parent.children(recursive=True):
                        child.terminate()
                    parent.terminate()
                    parent.wait()  # Ensure the parent process has terminated
                    print(f"Stopped Package [{pkg}], launch file [{launch_file}].", flush=True)
                except psutil.NoSuchProcess:
                    print(f"Process for Package [{pkg}], launch file [{launch_file}] not found.", flush=True)

                if not self.node_processes:  # All launches stopped
                    self.running = False
                    print("All launch files stopped — shutting down output threads.")
            else:
                print(f"Package [{pkg}], Launch file [{launch_file}] is not running.", flush=True)


    def restart_launch(self, pkg, launch_file):
        with self.lock:
            self.stop_launch(pkg, launch_file)
            time.sleep(1)  # Optionally wait a bit before restarting
            self.start_launch(pkg, launch_file)

    def stop_all_launches(self):
        with self.lock:
            for key in list(self.node_processes.keys()):
                pkg, launch_file = key
                self.stop_launch(pkg, launch_file)
            print("Stopped all running launch files.")

    def list_running_launches(self):
        # with self.lock:
            # return list(self.node_processes.keys())
        with self.lock:
            # Clean up terminated processes
            for key, process in list(self.node_processes.items()):
                if process.poll() is not None:  # Process has terminated
                    self.node_processes.pop(key)
            
            # Return the updated list of running processes
            return list(self.node_processes.keys())


    def shutdown(self):
        self.running = False  # Stop the streaming thread
        with self.lock:
            for key, process in self.node_processes.items():
                process.terminate()
                try:
                    process.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    process.kill()
                print(f"Stopped launch file [{key}].")
            self.node_processes.clear()

        if(self.udp_stream):
            self.sock.close()  # Close the UDP socket
            print("UDP socket closed.")