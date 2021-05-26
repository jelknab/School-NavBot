class BotCommand:
    def __init__(self, command_id: int, command: str, value: float):
        self.id = command_id
        self.command = command
        self.value = value


class Bot:
    def __init__(self, id: int):
        self.id: int
        self.estimated_position = (0, 0)
        self.active_command = BotCommand(-1, '', 0)
