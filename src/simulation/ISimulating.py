from src.simulation.parts import Body


class ISimulating:
    def simulate(self, bot: Body, seconds_delta: float, seconds_passed: float):
        raise NotImplementedError('Class has no simulation implementation')
