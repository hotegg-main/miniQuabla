class Payload:

    def __init__(self, config):

        self.exist = config['exist_payload']
        self.mass  = config['mass_payload']
        self.vel_descent = config['vel_payload']

    def get_velocity(self, time, altitude):

        return self.vel_descent