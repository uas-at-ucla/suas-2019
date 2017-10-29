import logging

logger = logging.getLogger(__name__)


def boolean_param(value):
    """Convert string parameter to boolean

    Raises:
        ValueError: value does not represent a boolean
    """
    if value.lower() == 'true':
        return True
    elif value.lower() == 'false':
        return False

    raise ValueError('Value "%s" does not represent a boolean' % value)
