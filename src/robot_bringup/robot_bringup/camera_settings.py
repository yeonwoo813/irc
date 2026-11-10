"""Portable RGB settings; driver parameter names are resolved at runtime."""

from pathlib import Path

from robot_bringup.ini_settings import _parser, save_ini_section


SECTIONS = {'realsense': 'realsense_rgb', 'webcam': 'webcam_controls'}
AUTO_KEYS = ('enable_auto_exposure', 'enable_auto_white_balance')
MANUAL_AUTO = {'exposure': AUTO_KEYS[0], 'gain': AUTO_KEYS[0],
               'white_balance': AUTO_KEYS[1]}
NUMERIC_KEYS = ('exposure', 'gain', 'white_balance', 'brightness',
                'contrast', 'saturation', 'sharpness')
KEYS = AUTO_KEYS + NUMERIC_KEYS
WEBCAM_ALIASES = {
    'enable_auto_exposure': ('auto_exposure', 'exposure_auto'),
    'enable_auto_white_balance': ('white_balance_automatic',
                                  'white_balance_temperature_auto'),
    'exposure': ('exposure_time_absolute', 'exposure_absolute'),
    'white_balance': ('white_balance_temperature',),
}


def validate_settings(values):
    for key, value in values.items():
        if key not in KEYS:
            raise ValueError(f'Unknown camera setting: {key}')
        expected = bool if key in AUTO_KEYS else int
        if type(value) is not expected:
            raise ValueError(f'{key} must be {expected.__name__}')
        if key in ('exposure', 'white_balance') and value <= 0:
            raise ValueError(f'{key} must be positive')
        if key in ('gain', 'contrast', 'saturation', 'sharpness') and value < 0:
            raise ValueError(f'{key} must be nonnegative')
    return dict(values)


def load_camera_settings(path, camera):
    parser = _parser(Path(path).expanduser().read_text(encoding='utf-8'))
    section = SECTIONS[camera]
    values = {key: parser.getboolean(section, key, fallback=True) for key in AUTO_KEYS}
    for key in NUMERIC_KEYS:
        if parser.has_option(section, key):
            values[key] = parser.getint(section, key)
    return validate_settings(values)


def manual_settings(values):
    """Auto-mode parameter values are setpoints, not live sensor readings."""
    return {key: value for key, value in values.items()
            if key not in MANUAL_AUTO or not values.get(MANUAL_AUTO[key], False)}


def save_camera_settings(path, camera, values):
    save_ini_section(path, SECTIONS[camera], manual_settings(validate_settings(values)))


def realsense_rgb_parameters(path):
    return {'rgb_camera.' + key: value for key, value in
            manual_settings(load_camera_settings(path, 'realsense')).items()}


def resolve_names(camera, names):
    result = {}
    for key in KEYS:
        candidates = (('rgb_camera.' + key,) if camera == 'realsense'
                      else WEBCAM_ALIASES.get(key, (key,)))
        for candidate in candidates:
            if candidate in names:
                result[key] = candidate
                break
    return result


def write_sequence(values):
    """Set each auto flag before its dependent manual controls."""
    values = validate_settings(values)
    ordered = []
    for auto in AUTO_KEYS:
        dependent = {key: value for key, value in values.items()
                     if MANUAL_AUTO.get(key) == auto}
        if auto in values or dependent:
            enabled = values.get(auto, False)
            ordered.append((auto, enabled))
            if not enabled:
                ordered.extend(dependent.items())
    ordered.extend((key, value) for key, value in values.items()
                   if key not in AUTO_KEYS and key not in MANUAL_AUTO)
    return ordered
