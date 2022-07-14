class MetaDataDistributor():
    def __init__(self):
        self.constants = {
            # Constants
            # Earth radius at equator [m]
            'A': 6378137,
            # Earth radius to pole [m]
            'B': 6356752.31425,
            # Rotational velocity [rads/s]
            'OMEGA': 7292115.1467e-11,
            # Pi
            'PI': 3.14159265359
        }
        # Oblateness
        self.constants['F'] = (self.constants['A'] - self.constants['B']) / self.constants['A']
        # Eccentricity
        self.constants['E'] = (self.constants['A'] ** 2 - self.constants['B'] ** 2) ** 0.5 / self.constants['B']
        self.namespaces = {'vars': {}}

    # TO-DO: refine functions
    def set(self, vars, namespace='vars'):
        if (namespace not in self.namespaces):
            self.namespaces[namespace] = {}
        for k, v in vars.items():
            self.namespaces[namespace][k] = v
        return self.namespaces[namespace]

    def get_namespace(self, namespace):
        return self.namespaces[namespace]

    def get_var(self, var, namespace='vars'):
        return self.namespaces[namespace][var]

    def get(self, var, namespace='vars'):
        return self.namespaces[namespace][var]
