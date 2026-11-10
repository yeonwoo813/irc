"""Read depth controls and save them without rewriting the rest of the INI."""

import configparser
import math

from robot_bringup.ini_settings import save_ini_section
from pathlib import Path


SECTION = 'realsense_depth'
DEFAULTS = {
    'enable_auto_exposure': False,
    'exposure': 5000,
    'gain': 16,
    'laser_power': 150.0,
}
PARAMETER_NAMES = tuple('depth_module.' + key for key in DEFAULTS)


def _parser(text):
    parser = configparser.ConfigParser(
        interpolation=None, inline_comment_prefixes=('#', ';'),
    )
    parser.read_string(text)
    return parser


def validate_settings(values):
    """Check types/basic bounds; the camera checks its own hardware limits."""
    values = {key: values[key] for key in DEFAULTS}
    if type(values['enable_auto_exposure']) is not bool:
        raise ValueError('enable_auto_exposure must be true or false')
    for key in ('exposure', 'gain'):
        if type(values[key]) is not int:
            raise ValueError(f'{key} must be an integer')
    if values['exposure'] <= 0 or values['gain'] < 0:
        raise ValueError('exposure must be positive and gain must be nonnegative')
    laser = values['laser_power']
    if isinstance(laser, bool) or not isinstance(laser, (int, float)):
        raise ValueError('laser_power must be a number')
    if not math.isfinite(laser) or laser < 0:
        raise ValueError('laser_power must be finite and nonnegative')
    values['laser_power'] = float(laser)
    return values


def load_depth_settings(path):
    parser = _parser(Path(path).expanduser().read_text(encoding='utf-8'))
    getters = {'enable_auto_exposure': parser.getboolean,
               'exposure': parser.getint, 'gain': parser.getint,
               'laser_power': parser.getfloat}
    return validate_settings({
        key: getters[key](SECTION, key, fallback=default)
        for key, default in DEFAULTS.items()
    })


def depth_parameters(path):
    return {'depth_module.' + key: value
            for key, value in load_depth_settings(path).items()}


def load_depth_display_settings(path):
    parser = _parser(Path(path).expanduser().read_text(encoding='utf-8'))
    result = {key: parser.getfloat('realsense_yolo', key, fallback=default)
              for key, default in [('depth_scale', 0.001),
                                   ('depth_min_m', 0.08), ('depth_max_m', 3.0)]}
    if not all(math.isfinite(v) for v in result.values()):
        raise ValueError('Depth display settings must be finite')
    if result['depth_scale'] <= 0 or not (
            0 <= result['depth_min_m'] < result['depth_max_m']):
        raise ValueError('Invalid depth scale or display range')
    return result


def save_depth_settings(path, values):
    """Atomically update only our four keys, preserving comments/other edits."""
    values = validate_settings(values)
    save_ini_section(path, SECTION, values)
