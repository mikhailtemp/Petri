from controller import Supervisor
import threading, time, json

class Driver(Supervisor):
    timeStep = 128

    def __init__(self):
        super(Driver, self).__init__()
        self.emitter  = self.getDevice("emitter")
        self.receiver = self.getDevice("receiver")
        self.receiver.enable(self.timeStep)
        self.keyboard = self.getKeyboard()
        self.keyboard.enable(self.timeStep)
        self.robot_names = ["robot1", "robot2", "robot3", "robot4"]
        self.robot_defs  = ["ROBOT1", "ROBOT2", "ROBOT3", "ROBOT4"]
        self.robots = {n: self.getFromDef(d)
                       for n,d in zip(self.robot_names, self.robot_defs)}

        ring = [
            (-0.8,  0.8),  # 0: верхний левый
            (-0.8,  0.0),  # 1: вход zone_A  <-- критическая
            (-0.8, -0.8),  # 2: нижний левый
            ( 0.8, -0.8),  # 3: нижний правый
            ( 0.8,  0.0),  # 4: вход zone_B  <-- критическая
            ( 0.8,  0.8),  # 5: верхний правый
        ]
        self.waypoints = {
            "robot1": ring,
            "robot2": ring[2:] + ring[:2],
            "robot3": ring[3:] + ring[:3],
            "robot4": ring[5:] + ring[:5],
        }

        def make_resources(wps):
            r = {}
            for i, pt in enumerate(wps):
                if pt == (-0.8, 0.0):
                    r[i] = "zone_A"
                elif pt == (0.8, 0.0):
                    r[i] = "zone_B"
            return r

        self.step_resource = {
            n: make_resources(self.waypoints[n])
            for n in self.robot_names
        }
        self.passage_locks = {
            "zone_A": threading.Lock(),
            "zone_B": threading.Lock(),
        }
        self.stop_flag     = threading.Event()
        self.arrived_event = {n: threading.Event() for n in self.robot_names}

        # Флаг: спавн завершён, можно запускать потоки
        self.init_done = False

    def run(self):
        # ---- INIT в главном потоке ----
        print("[INIT] Placing robots at start positions")
        for name in self.robot_names:
            start = self.waypoints[name][0]
            node  = self.robots[name]
            node.getField('translation').setSFVec3f([start[0], 0.025, start[1]])
            node.getField('rotation').setSFRotation([0, 1, 0, 0])

        # Даём физике 30 шагов успокоиться — всё в главном потоке
        for _ in range(30):
            self.step(self.timeStep)

        # ---- RUN: запускаем потоки ----
        print("[RUN] Starting robot threads")
        for name in self.robot_names:
            c = RobotController(self, name)
            t = threading.Thread(target=c.run)
            t.daemon = True
            t.start()

        # Главный цикл симуляции
        while True:
            self.process_receiver()
            if self.step(self.timeStep) == -1:
                break
            if self.keyboard.getKey() == ord("Q"):
                self.stop_flag.set()
                break

        # ---- FINISH ----
        print("[FINISH] Stopping all robots")
        for name in self.robot_names:
            cmd = {"type": "stop", "robot": name}
            self.emitter.send(json.dumps(cmd).encode())

    def process_receiver(self):
        while self.receiver.getQueueLength() > 0:
            data = self.receiver.getString()
            self.receiver.nextPacket()
            try:
                msg = json.loads(data)
                if msg["type"] == "arrived":
                    robot = msg["robot"].lower()
                    if robot in self.arrived_event:
                        self.arrived_event[robot].set()
            except Exception as e:
                print("[DRIVER] Error: " + str(e))


class RobotController:
    def __init__(self, driver, name):
        self.driver          = driver
        self.name            = name
        self.waypoints       = driver.waypoints[name]
        self.step_to_passage = driver.step_resource.get(name, {})
        self.passage_locks   = driver.passage_locks
        self.current_step    = 0

    def run(self):
        while not self.driver.stop_flag.is_set():
            next_wp   = (self.current_step + 1) % len(self.waypoints)
            lock_name = self.step_to_passage.get(next_wp)

            if lock_name:
                self.passage_locks[lock_name].acquire()
                print("[" + self.name + "] Locked " + lock_name)

            target = self.waypoints[next_wp]
            self.driver.arrived_event[self.name].clear()
            cmd = {"type": "move", "robot": self.name, "target": target}
            self.driver.emitter.send(json.dumps(cmd).encode())

            arrived = False
            while not self.driver.stop_flag.is_set():
                if self.driver.arrived_event[self.name].wait(timeout=0.1):
                    arrived = True
                    break

            if not arrived:
                if lock_name:
                    self.passage_locks[lock_name].release()
                break

            self.current_step = next_wp
            if lock_name:
                self.passage_locks[lock_name].release()
                print("[" + self.name + "] Released " + lock_name)


controller = Driver()
controller.run()
