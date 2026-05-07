"""
slave.py – контроллер мобильного робота.
Принимает JSON-команды от супервизора через receiver,
едет к целевой точке, отправляет подтверждение 'arrived'.
"""

from controller import Robot
import json
import math

robot = Robot()
TIME_STEP = 64
MAX_SPEED = 2.5                # снижено с 4.0 — меньше опрокидываний
ARRIVAL_THRESHOLD = 0.05       # метры

receiver = robot.getDevice('receiver')
emitter  = robot.getDevice('emitter')
receiver.enable(TIME_STEP)

left_motor  = robot.getDevice('left wheel motor')
right_motor = robot.getDevice('right wheel motor')
left_motor.setPosition(float('inf'))
right_motor.setPosition(float('inf'))
left_motor.setVelocity(0)
right_motor.setVelocity(0)

gps = robot.getDevice('gps')
gps.enable(TIME_STEP)

compass = robot.getDevice('compass')
compass.enable(TIME_STEP)

robot_name = robot.getName()
target = None
moving = False


def get_bearing_to(tx, tz):
    """Угол от текущей позиции до цели в плоскости XZ."""
    pos = gps.getValues()   # [x, y, z]
    dx = tx - pos[0]
    dz = tz - pos[2]
    return math.atan2(dx, dz)


def get_heading():
    """Текущий курс по компасу. Компас даёт вектор на север в плоскости XZ."""
    v = compass.getValues()  # [x, y, z] вектор на север
    return math.atan2(v[0], v[2])


def get_distance_to(tx, tz):
    pos = gps.getValues()
    dx = tx - pos[0]
    dz = tz - pos[2]
    return math.sqrt(dx*dx + dz*dz)


def send_arrived():
    msg = json.dumps({'type': 'arrived', 'robot': robot_name})
    emitter.send(msg.encode('utf-8'))


while robot.step(TIME_STEP) != -1:

    # --- Читаем входящие команды ---
    while receiver.getQueueLength() > 0:
        data = receiver.getString()
        receiver.nextPacket()
        try:
            msg = json.loads(data)
            if msg.get('robot', '').lower() != robot_name.lower():
                continue

            if msg['type'] == 'move':
                target = tuple(msg['target'])
                moving = True

            elif msg['type'] == 'stop':
                target = None
                moving = False
                left_motor.setVelocity(0)
                right_motor.setVelocity(0)

        except Exception as e:
            print('[' + robot_name + '] parse error: ' + str(e))

    # --- Движение к цели ---
    if moving and target is not None:
        dist = get_distance_to(*target)

        if dist < ARRIVAL_THRESHOLD:
            left_motor.setVelocity(0)
            right_motor.setVelocity(0)
            moving = False
            target = None
            send_arrived()
        else:
            bearing = get_bearing_to(*target)
            heading = get_heading()
            error   = bearing - heading

            # Нормализуем в [-pi, pi]
            while error >  math.pi: error -= 2 * math.pi
            while error < -math.pi: error += 2 * math.pi

            if abs(error) > 0.3:
                # Фаза 1: разворачиваемся на месте
                turn = max(-MAX_SPEED, min(MAX_SPEED, error * 2.0))
                left_motor.setVelocity(-turn)
                right_motor.setVelocity(turn)
            else:
                # Фаза 2: едем вперёд с мягкой коррекцией курса
                speed = min(MAX_SPEED, dist * 3.0)
                turn  = error * 1.5
                left_v  = max(-MAX_SPEED, min(MAX_SPEED, speed - turn))
                right_v = max(-MAX_SPEED, min(MAX_SPEED, speed + turn))
                left_motor.setVelocity(left_v)
                right_motor.setVelocity(right_v)
