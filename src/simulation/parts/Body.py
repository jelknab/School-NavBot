import math

from src.Bot import BotCommand
from src.simulation.ISimulating import ISimulating


class Command(BotCommand):
    def __init__(self, command_id: int, command: str, value: float, time_seconds: float):
        super().__init__(command_id, command, value)
        self.timestamp = time_seconds
        self.progress = 0

    def time_elapsed(self, time_passed):
        return time_passed - self.timestamp


class Body(ISimulating):
    def __init__(self, speed_change_m_s: float, max_speed_m_s: float):
        self.x = 0
        self.y = 0
        self.yaw = math.pi / 2
        self.command: Command = Command(-1, '', 0, 0)
        self.max_speed_m_s = max_speed_m_s
        self.speed_change_m_s = speed_change_m_s

    def receive_command(self, command: BotCommand):
        self.command = command

    def request_command(self):
        pass

    def position_at_time(self, t):
        speed_change_duration = self.max_speed_m_s / self.speed_change_m_s
        speed_change_distance = 0.5 * self.speed_change_m_s * speed_change_duration ** 2

        constant_speed_distance = self.command.value - speed_change_distance * 2
        constant_speed_duration = constant_speed_distance / self.max_speed_m_s

        acceleration_relative_time = min(t, speed_change_duration)
        constant_speed_relative_time = max(0, min(t - speed_change_duration, constant_speed_duration))
        deceleration_relative_time = max(0, min(t - speed_change_duration - constant_speed_duration, speed_change_duration))

        acceleration_distance = 0.5 * self.speed_change_m_s * acceleration_relative_time ** 2
        constant_distance = self.max_speed_m_s * constant_speed_relative_time
        deceleration_distance = 0.5 * self.speed_change_m_s * deceleration_relative_time ** 2

        return acceleration_distance + constant_distance + deceleration_distance

    def simulate(self, bot, seconds_delta: float, seconds_passed: float):
        if self.command.id == -1 and math.floor(seconds_passed) % 2 == 0:
            # self.request_command()  # should be done
            self.command = Command(1, 'f', 5, seconds_passed)
            pass

        if self.command.command == 'f':
            t_old = self.command.time_elapsed(seconds_passed - seconds_delta)
            t = self.command.time_elapsed(seconds_passed)

            old_distance = self.position_at_time(t_old)
            distance = self.position_at_time(t)

            difference = distance - old_distance

            self.x += math.sin(self.yaw) * difference * 1000
            self.y += math.cos(self.yaw) * difference * 1000



