from src.simulation.parts import Bot


class ISimulating:
    def simulate(self, bot: Bot, seconds_delta: float, seconds_passed: float):
        raise NotImplementedError('Class has no simulation implementation')
