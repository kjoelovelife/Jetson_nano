from . import logger

__all__ = [
    'expand_environment',
]

def expand_environment(s):
    ''' Expands ~ and ${ENV} in the string. '''

    import os
    s = os.path.expandvars(s)
    s = os.path.expanduser(s)
    if '$' in s:
        msg = 'Unresolved environment variable in string %r.' % s
        logger.error(msg)
        raise ValueError(msg)
    return s
