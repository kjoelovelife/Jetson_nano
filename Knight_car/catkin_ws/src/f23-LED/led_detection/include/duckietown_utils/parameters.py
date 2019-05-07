import yaml
import numpy as np

class Configurable():
    
    def __init__(self, param_names, configuration):
        if not isinstance(configuration, dict):
            msg = 'Expecting a dict, obtained %r' % configuration
            raise ValueError(msg)
        # check that we set all parameters
        given = list(configuration)
        
        required = list(param_names)
        
        extra = set(given) - set(required)
        missing = set(required) - set(given)
        if extra or missing:
            msg = ('Error while loading configuration for %r from %r.' % 
                   (self, configuration))
            msg += '\n'
            msg += 'Extra parameters: %r\n' % extra
            msg += 'Missing parameters: %r\n' % missing
            raise ValueError(msg)

        assert set(given) == set(required)
        for p in param_names:
            value = configuration[p]
            # if the list is 3 numbers, we convert to array
            if isinstance(value, list) and len(value) == 3:
                value = np.array(value)
            configuration[p] = value
            
        for p in param_names:
            setattr(self, p, configuration[p])
            
        return configuration