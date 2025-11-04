#!/usr/bin/env python3
import threading
import queue
import time
import os
import sqlite3
import json
import rospy
from geometry_msgs.msg import Point
from std_msgs.msg import Float32
from tsu200.srv import TaskUpload, TaskUploadResponse, TaskOpt, TaskOptResponse
from tsu200.srv import TaskDownload, TaskDownloadResponse
from tsu200.msg import Pos

class MissionManager(object):
    def __init__(self):
        self.db_path = os.path.join(os.path.expanduser('~'), '.ros', 'tsu200_tasks.db')
        self._init_db()
        
        self.lock = threading.Lock()
        self.todo_q = queue.Queue()
        self.current_task = None
        self.task_paused = False
        self.low_battery = False
        self.battery_voltage = 0.0
        self.pause_point = None
        
        # Start worker thread
        self.worker_thread = threading.Thread(target=self._worker)
        self.worker_thread.daemon = True
        
        # Start battery monitor thread
        self.battery_thread = threading.Thread(target=self._battery_monitor)
        self.battery_thread.daemon = True
        
        self.running = True
        self.worker_thread.start()
        self.battery_thread.start()
        rospy.loginfo('MissionManager worker and battery monitor threads started')

    def _init_db(self):
        """Initialize SQLite database with tasks table"""
        os.makedirs(os.path.dirname(self.db_path), exist_ok=True)
        with sqlite3.connect(self.db_path) as conn:
            c = conn.cursor()
            # tasks table: store task details and status
            c.execute('''CREATE TABLE IF NOT EXISTS tasks
                        (id INTEGER PRIMARY KEY,
                         home_pos TEXT,
                         pos_list TEXT,
                         status TEXT DEFAULT 'new',
                         created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
                         updated_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP)''')
            conn.commit()
        rospy.loginfo('Database initialized at %s', self.db_path)

    def _store_task(self, task_id, home_pos, pos_list):
        """Store task in SQLite database"""
        home_pos_json = json.dumps({'x': home_pos.x, 'y': home_pos.y, 'z': home_pos.z})
        pos_list_json = json.dumps([{
            'x': p.x, 'y': p.y, 'z': p.z,
            'task_type': p.task_type,
            'info': p.info
        } for p in pos_list])
        
        with sqlite3.connect(self.db_path) as conn:
            c = conn.cursor()
            c.execute('''INSERT OR REPLACE INTO tasks (id, home_pos, pos_list, status, updated_at)
                        VALUES (?, ?, ?, 'new', CURRENT_TIMESTAMP)''',
                     (task_id, home_pos_json, pos_list_json))
            conn.commit()

    def _load_task(self, task_id):
        """Load task from SQLite database"""
        with sqlite3.connect(self.db_path) as conn:
            c = conn.cursor()
            c.execute('SELECT home_pos, pos_list, status FROM tasks WHERE id = ?', (task_id,))
            row = c.fetchone()
            if not row:
                return None
            
            home_pos_data = json.loads(row[0])
            pos_list_data = json.loads(row[1])
            status = row[2]
            
            home_pos = Point(
                x=float(home_pos_data['x']),
                y=float(home_pos_data['y']),
                z=float(home_pos_data['z'])
            )
            
            pos_list = []
            for p_data in pos_list_data:
                p = Pos()
                p.x = float(p_data['x'])
                p.y = float(p_data['y'])
                p.z = float(p_data['z'])
                p.task_type = p_data['task_type']
                p.info = p_data['info']
                pos_list.append(p)
            
            return {
                'HomePos': home_pos,
                'PosList': pos_list,
                'status': status
            }

    def _update_task_status(self, task_id, status):
        """Update task status in database"""
        with sqlite3.connect(self.db_path) as conn:
            c = conn.cursor()
            c.execute('''UPDATE tasks 
                        SET status = ?, updated_at = CURRENT_TIMESTAMP
                        WHERE id = ?''', (status, task_id))
            conn.commit()

    def _battery_monitor(self):
        """Monitor battery voltage and trigger return-to-home when voltage is low"""
        rospy.Subscriber('battery_voltage', Float32, self._battery_callback)
        rospy.loginfo('Battery monitor started')
        
        while self.running and not rospy.is_shutdown():
            if self.low_battery and self.current_task is not None:
                if not self.task_paused:
                    rospy.logwarn('Low battery detected (%.1fV)! Pausing current task and returning to home', 
                                self.battery_voltage)
                    self._pause_current_task()
                    if self.current_task:
                        # Return to home position
                        try:
                            home_pos = self.current_task['HomePos']
                            self._move_to_point(home_pos, label='HomePos (Emergency RTH)')
                            rospy.loginfo('Successfully returned to home position')
                        except Exception as e:
                            rospy.logerr('Failed to execute return-to-home: %s', e)
            rospy.sleep(1.0)

    def _battery_callback(self, msg):
        """Callback for battery voltage updates"""
        self.battery_voltage = msg.data
        # Check if voltage is below 18V
        self.low_battery = self.battery_voltage < 18.0

    def _pause_current_task(self):
        """Pause the current task and save position"""
        with self.lock:
            if self.current_task and not self.task_paused:
                self.task_paused = True
                task_id = self.current_task.get('id')
                self._update_task_status(task_id, 'paused')
                rospy.loginfo('Task %d paused due to low battery', task_id)

    def _resume_current_task(self):
        """Resume the current task if it was paused"""
        with self.lock:
            if self.current_task and self.task_paused:
                self.task_paused = False
                task_id = self.current_task.get('id')
                self._update_task_status(task_id, 'running')
                rospy.loginfo('Task %d resumed', task_id)

    def stop(self):
        self.running = False
        # put a sentinel to unblock queue.get()
        try:
            self.todo_q.put_nowait(None)
        except Exception:
            pass
        self.worker_thread.join(timeout=2.0)
        self.battery_thread.join(timeout=2.0)

    # Service handler to upload tasks
    def handle_taskupload(self, req):
        rospy.loginfo('TaskUpload received id=%d PosNum=%d', req.id, req.PosNum)
        if req.PosNum != len(req.PosList):
            msg = 'PosNum does not match PosList length (%d vs %d)' % (req.PosNum, len(req.PosList))
            rospy.logwarn(msg)
            return TaskUploadResponse(req.id, False, 2, msg)

        try:
            self._store_task(req.id, req.HomePos, req.PosList)
            rospy.loginfo('Stored task id=%d in database', req.id)
            return TaskUploadResponse(req.id, True, 0, 'Uploaded and stored in database')
        except Exception as e:
            msg = 'Failed to store task: %s' % str(e)
            rospy.logerr(msg)
            return TaskUploadResponse(req.id, False, -1, msg)

    # Service handler to opt/execute a task
    def handle_taskopt(self, req):
        rospy.loginfo("TaskOpt request: opt='%s' id=%d", req.opt, req.id)
        task = self._load_task(req.id)
        if not task:
            msg = 'No uploaded task with id=%d' % req.id
            rospy.logwarn(msg)
            return TaskOptResponse(False, 1, msg)
        
        if task['status'] == 'running':
            msg = 'Task %d is already running' % req.id
            rospy.logwarn(msg)
            return TaskOptResponse(False, 3, msg)
        
        # enqueue the id and opt
        self.todo_q.put((req.id, req.opt))
        self._update_task_status(req.id, 'queued')
        rospy.loginfo('Enqueued task id=%d for execution', req.id)
        return TaskOptResponse(True, 0, 'Enqueued')

    # Worker thread: process queued tasks sequentially
    def _worker(self):
        rospy.loginfo('Mission worker running')
        while self.running and not rospy.is_shutdown():
            item = None
            try:
                item = self.todo_q.get()
            except Exception as e:
                rospy.logerr('Queue get exception: %s', e)
                break
            if item is None:
                # sentinel for shutdown
                rospy.loginfo('Worker received shutdown sentinel')
                break
            task_id, opt = item
            rospy.loginfo('Worker popped task id=%d opt=%s', task_id, opt)
            
            # Load task from database
            task = self._load_task(task_id)
            if not task:
                rospy.logwarn('No stored task for id=%d, skipping', task_id)
                continue

            try:
                # Update task status to running
                self._update_task_status(task_id, 'running')
                # Execute mission
                self._execute_task(task_id, task, opt)
                # Update task status to completed
                self._update_task_status(task_id, 'completed')
            except Exception as e:
                rospy.logerr('Error executing task id=%d: %s', task_id, e)
                self._update_task_status(task_id, 'failed')

    def _execute_task(self, task_id, task, opt):
        home = task['HomePos']
        pos_list = task['PosList']
        rospy.loginfo('Starting mission id=%d to HomePos=(%.3f,%.3f,%.3f) with %d points', task_id, home.x, home.y, home.z, len(pos_list))

        # Set current task context
        with self.lock:
            self.current_task = task
            self.current_task['id'] = task_id
            self.task_paused = False

        try:
            # Simulate takeoff / move to HomePos
            self._move_to_point(home, label='HomePos')

            for idx, p in enumerate(pos_list):
                # Check if task should be paused
                while self.task_paused:
                    if not self.running or rospy.is_shutdown():
                        return
                    rospy.sleep(0.5)
                
                # Check if we should continue execution
                if not self.running or rospy.is_shutdown():
                    break

                # move to point
                self._move_to_point(p, label='Pos[%d]' % idx)
                # execute task type at this pos
                self._execute_pos_task(p, idx)

            rospy.loginfo('Mission id=%d completed', task_id)
        finally:
            # Clear current task context
            with self.lock:
                self.current_task = None
                self.task_paused = False

    def _move_to_point(self, p, label=''):
        # p can be geometry_msgs/Point or tsu200/Pos
        if hasattr(p, 'x'):
            x = p.x
            y = p.y
            z = p.z
        else:
            x = y = z = 0.0
        rospy.loginfo('Moving to %s (x=%.3f y=%.3f z=%.3f)...', label, x, y, z)
        # Simulate movement time proportional to distance (naive)
        # For demo just sleep a fixed small time
        time.sleep(1.0)
        rospy.loginfo('Reached %s', label)

    def _execute_pos_task(self, pos, idx):
        task_type = getattr(pos, 'task_type', 'unknown')
        info = getattr(pos, 'info', '')
        rospy.loginfo('Executing task at Pos[%d]: type=%s info=%s', idx, task_type, info)
        # Simulate different durations
        if task_type == 'takeoff':
            time.sleep(0.5)
            rospy.loginfo('Takeoff simulated')
        elif task_type == 'survey':
            time.sleep(1.5)
            rospy.loginfo('Survey simulated')
        else:
            time.sleep(0.7)
            rospy.loginfo('Generic action simulated')

    def handle_taskdownload(self, req):
        """Service handler to download task details"""
        rospy.loginfo('TaskDownload request for id=%d', req.id)
        
        # Load task from database
        task = self._load_task(req.id)
        if not task:
            msg = 'No task found with id=%d' % req.id
            rospy.logwarn(msg)
            return TaskDownloadResponse(False, 1, msg, Point(), [], 0)
        
        return TaskDownloadResponse(
            True,  # success
            0,    # error_code
            'Task found',  # message
            task['HomePos'],  # HomePos
            task['PosList'],  # PosList
            len(task['PosList'])  # PosNum
        )

def main():
    rospy.init_node('mission_manager')
    manager = MissionManager()

    # Register all services
    upload_srv = rospy.Service('task_upload', TaskUpload, manager.handle_taskupload)
    opt_srv = rospy.Service('task_opt', TaskOpt, manager.handle_taskopt)
    download_srv = rospy.Service('task_download', TaskDownload, manager.handle_taskdownload)

    rospy.loginfo('mission_manager ready: services task_upload, task_opt, and task_download registered')

    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo('Shutting down mission_manager')
    finally:
        manager.stop()

if __name__ == '__main__':
    main()
