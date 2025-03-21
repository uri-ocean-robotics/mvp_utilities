import subprocess
import threading
import time
import os
import signal
import psutil  
import socket
import sys


class ROSLaunchManager:
    def __init__(self):
        self.node_processes = {}
        self.lock = threading.Lock()

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
        # Connect to an external server (Google DNS)
            self.sock.connect(('8.8.8.8', 80))
            local_ip = self.sock.getsockname()[0]  # Get the local address used for the connection
        except Exception:
            local_ip = '127.0.0.1'  # Fallback to localhost if no connection can be made
        print(local_ip)
        self.udp_ip = local_ip
        self.udp_port = 3000
        
        self.sock.setblocking(False)  # This makes the socket non-blocking
        self.running = True  # Flag to control the thread


    def start_launch(self, launch_file):
        with self.lock:
            key = (launch_file)
            if key in self.node_processes:
                print(f"Launch file [{launch_file}] is already running.")
                return
            env = os.environ.copy()
            if 'ROS_NAMESPACE' in env:
                del env['ROS_NAMESPACE']

            # process = subprocess.Popen(['roslaunch', str(launch_file)], env=env)
            # self.node_processes[key] = process
            # print(f"Started launch file [{launch_file}].")

            ##testing udp
            process = subprocess.Popen(
                ['roslaunch', launch_file],
                env=env,  # You can modify environment variables here
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True  # Ensure you get strings instead of bytes
            )

            self.node_processes[key] = process
            print(f"Started launch file [{launch_file}].")

            # Start a thread to handle output streaming
            output_thread = threading.Thread(target=self._stream_output, args=(process.stdout,))
            output_thread.daemon = True  # Daemon thread exits when the main program exits
            output_thread.start()

            # Optionally handle stderr as well, if you want to stream errors
            error_thread = threading.Thread(target=self._stream_output, args=(process.stderr,))
            error_thread.daemon = True
            error_thread.start()



    def _stream_output(self, pipe):
        while self.running:  # Keep running while the 'running' flag is True
            line = pipe.readline()
            
            if line:
                print(line.strip())  # Print the line if it's not empty
                self.sock.sendto(line.encode(), (self.udp_ip, self.udp_port))  # Send via UDP
                # sys.stdout.flush()
                # Warning condition
                if 'WARNING' in line.upper():
                    print(f"Warning: {line.strip()}")  # Print the warning
                elif 'ERROR' in line.upper():
                    print(f"Error: {line.strip()}")  # Print the error

            else:
                # If the line is empty, you can introduce a small delay to avoid tight looping
                time.sleep(0.1)  # This allows the loop to check periodically for new lines

        pipe.close()  # Close pipe once we're done
        print("Stream finished.")


    def stop_launch(self, launch_file):
        with self.lock:
            key = (launch_file)
            process = self.node_processes.pop(key, None)
            if process:
                try:
                    parent = psutil.Process(process.pid)
                    for child in parent.children(recursive=True):
                        child.terminate()
                    parent.terminate()
                    parent.wait()  # Ensure the parent process has terminated
                    print(f"Stopped launch file [{launch_file}].", flush=True)
                except psutil.NoSuchProcess:
                    print(f"Process for launch file [{launch_file}] not found.", flush=True)
            else:
                print(f"Launch file [{launch_file}] is not running.", flush=True)


    def restart_launch(self, launch_file):
        with self.lock:
            self.stop_launch(launch_file)
            time.sleep(1)  # Optionally wait a bit before restarting
            self.start_launch(launch_file)

    def stop_all_launches(self):
        with self.lock:
            for key in list(self.node_processes.keys()):
                launch_file = key
                self.stop_launch(launch_file)
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
        self.sock.close()  # Close the UDP socket
        print("UDP socket closed.")